//
//  ethraw.cpp
//  
//
//  Created by Enrico Eberhard on 26/09/2017.
//
//  This file constitutes a hopefully cross-platform
//	utility for sending and receiving raw ethernet packets.

//	On Linux, this is done by opening a raw socket and
//	manually configuring the packet.
//
//	With help from:
//	http://hacked10bits.blogspot.co.uk/2011/12/sending-raw-ethernet-frames-in-6-easy.html
//
//  On OSX, raw sockets require IP headers. Instead, we
//	operate directly on a BPF device.
//
//	With help from:
//	http://bastian.rieck.ru/howtos/bpf/


//  main utilities should be send_packet and read_packet,
//	and should automatically compile to use distinct methods
//	based on platform

#include "ethraw.h"

#ifdef __linux__

struct sockaddr_ll this_sockaddr;

#endif

const size_t ETHER_PAYLOAD_LEN = ETHER_MAX_LEN - ETHER_HDR_LEN - ETHER_CRC_LEN;

int bpf_buf_len = ETHER_MAX_LEN;

char iface[IFNAMSIZ] = "lo0";
//mac addresses
unsigned char dest_mac[ETHER_ADDR_LEN] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
unsigned char src_mac[ETHER_ADDR_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
//interface protocol
unsigned short protocol = 0x1234;


int open_interface(void) {

	int device = 0;
	
#ifdef __linux__
	
	if ((device = socket(AF_PACKET, SOCK_RAW, htons(protocol))) < 0) {
		perror("Could not open socket");
		exit(1);
	}

#elif __APPLE__
	
	char buf[ 11 ] = { 0 };
	int i = 0;
	for(i = 0; i < 99; i++ )
	{
		sprintf( buf, "/dev/bpf%i", i );
		device = open( buf, O_RDWR );
		if( device != -1 ) {
			printf("Opened device /dev/bpf%i\n", i);
			break;
		}
	}
	if(device == -1) {
		printf("Cannot open any /dev/bpf* device\n");
		exit(1);
	}
	
	
#endif

	return device;
}



int associate_interface(int device, const char* iface) {
	
#ifdef __linux__
	
	//do a bunch of ioctl magic to find properties of the infertace
	//from the given interface name
	int ifindex;
	struct ifreq buffer;
	
	//interface index
	memset(&buffer, 0x00, sizeof(buffer));
	strncpy(buffer.ifr_name, iface, IFNAMSIZ);
	if (ioctl(device, SIOCGIFINDEX, &buffer) < 0) {
		perror("Could not get interface index");
		close(device);
		exit(1);
	}
	ifindex = buffer.ifr_ifindex;
	
	//get source hardware address
	if (ioctl(device, SIOCGIFHWADDR, &buffer) < 0) {
		perror("Could not get interface address");
		close(device);
		exit(1);
	}
	memcpy(src_mac, buffer.ifr_hwaddr.sa_data, ETHER_ADDR_LEN);
	
	//preparing the socket
	memset((void*)&this_sockaddr, 0, sizeof(this_sockaddr));
	this_sockaddr.sll_family = PF_PACKET;
	this_sockaddr.sll_ifindex = ifindex;
	this_sockaddr.sll_halen = ETH_ALEN;
	memcpy((void*)(this_sockaddr.sll_addr), (void*)dest_mac, ETH_ALEN);
	
	return 0;
	
#elif __APPLE__
	
	//get source hardware address
	ifaddrs* iflist;
	if (getifaddrs(&iflist) == 0) {
		for (ifaddrs* cur = iflist; cur; cur = cur->ifa_next) {
			if ((cur->ifa_addr->sa_family == AF_LINK) &&
				(strcmp(cur->ifa_name, iface) == 0) &&
				cur->ifa_addr) {
				sockaddr_dl* sdl = (sockaddr_dl*)cur->ifa_addr;
				memcpy(src_mac, LLADDR(sdl), sdl->sdl_alen);
				break;
			}
		}
		
		freeifaddrs(iflist);
	}
	
	struct ifreq bound_if;
	
	strcpy(bound_if.ifr_name, iface);
	if(ioctl( device, BIOCSETIF, &bound_if ) > 0) {
		printf("Cannot bind bpf device to physical device %s, exiting\n", iface);
		exit(1);
	}
	printf("Bound bpf device to physical device %s\n", iface);
	
	int buf_len = 1;
	// activate immediate mode (therefore, buf_len is initially set to "1")
	if( ioctl( device, BIOCIMMEDIATE, &buf_len ) == -1 ) {
		printf("Cannot set IMMEDIATE mode of bpf device\n");
		exit(1);
	}
	// request buffer length
	if( ioctl( device, BIOCGBLEN, &buf_len ) == -1 ) {
		printf("Cannot get bufferlength of bpf device\n");
		exit(1);
	}
	printf("Buffer length of bpf device: %d\n", buf_len);
	
	bpf_buf_len = buf_len;
	
#endif
	
	return 0;
}




struct ethframe prepare_frame(unsigned char dest[ETHER_HDR_LEN],
				   unsigned char source[ETHER_HDR_LEN],
							  unsigned short proto) {

	struct ethframe *frame;
    frame = (struct ethframe *)malloc(ETHER_MAX_LEN);
	
	memcpy(frame->header.ether_dhost, dest, ETHER_HDR_LEN);
	memcpy(frame->header.ether_shost, source, ETHER_HDR_LEN);
	frame->header.ether_type = htons(proto);
	frame->len = (2*ETHER_ADDR_LEN) + ETHER_TYPE_LEN;
	
	return *frame;
}


int send_frame(int device, unsigned char* data, size_t data_len) {
	
	//remake frame every time, might be faster
	struct ethframe frame = prepare_frame(dest_mac, src_mac, protocol);
	
	//alternatively, take an existing frame and clear data portion
	//bzero((void*)(frame+ETHER_HDR_LEN), ETHER_PAYLOAD_LEN);

	// Copy data into payload
	memcpy(frame.data, data, data_len);
	frame.len += data_len;
	frame.data_len = data_len;
	
#ifdef __linux__
	
	//sending the packet
	if (sendto(device, &frame, frame.len, 0,
			   (struct sockaddr*)&this_sockaddr, sizeof(this_sockaddr)) < 0) {
		perror("Error sending packet");
		close(device);
		exit(1);
	}
	
#elif __APPLE__
	
	ssize_t bytes_sent;
	bytes_sent = write(device, &frame, frame.len);
	if(bytes_sent <= 0) {
		perror("Write failed! Does the device have an IP address?");
		exit(1);
	}
	
#endif
	
	
	//free(frame);
	
	return 0;
}


// Read one or more frames
void read_frames(int device, void (*callback)(unsigned char*))
{
	int data_read = 0;
	unsigned char data[ETHER_PAYLOAD_LEN];
	
#ifdef __linux__
	
	//only open the socket once
	static int sockfd = -1;
	
	if (sockfd < 0)
		if ((sockfd = socket(AF_PACKET, SOCK_RAW, htons(protocol))) < 0)
			perror("socket error");
	
	struct sockaddr_ll this_sockaddr;
	char frame_buf[ETH_FRAME_LEN];
	
	socklen_t sockaddr_len = (socklen_t)sizeof(this_sockaddr);
	data_read = recvfrom(sockfd, frame_buf, ETH_FRAME_LEN, 0x00,
							 (struct sockaddr*)&this_sockaddr, &sockaddr_len);
	
	//extract the data
	memset(data, 0, ETHER_PAYLOAD_LEN);
	strncpy((char *)data, (const char*)frame_buf+sizeof(struct ethhdr), ETHER_PAYLOAD_LEN);


	
#elif __APPLE__
	
	int buf_len = bpf_buf_len;
	struct ethframe *frame;
	struct bpf_hdr *bpf_buf = (struct bpf_hdr*) malloc(buf_len);
	struct bpf_hdr *bpf_packet;
		
	//clear buffer
	memset(bpf_buf, 0, buf_len);
	
	if((data_read = (int)read(device, bpf_buf, buf_len)) > 0)
	{
		// read all packets that are included in bpf_buf.
		char* ptr = (char*)bpf_buf;
		while(ptr < ((char*)(bpf_buf) + data_read))
		{
			bpf_packet = (struct bpf_hdr*)ptr;
			frame = (struct ethframe*)((char*) bpf_packet + bpf_packet->bh_hdrlen);
			frame->len = bpf_packet->bh_caplen;
			frame->data_len = frame->len - ETHER_HDR_LEN;
			
			// Do something with the frame
			//printf("Got packet, length of frame: %ld, length of data: %ld\n",
			//	   frame->len, frame->data_len);
			
			//extract the data
			memset(data, 0, ETHER_PAYLOAD_LEN);
			strncpy((char*)data, (const char*)frame->data + 4, (size_t)frame->data_len);
						
			// BPF_WORDALIGN is used to proceed to the next BPF packet
			// that is available in the buffer.
			ptr += BPF_WORDALIGN(bpf_packet->bh_hdrlen + bpf_packet->bh_caplen);
		}
	} else {
		perror("Couldn't read from bpf device");
		exit(1);
	}
	
#endif
	
	//handle the received data
	(*callback)(data);

}




