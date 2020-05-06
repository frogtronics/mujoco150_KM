//
//  ethraw.h
//  
//
//  Created by Enrico Eberhard on 26/09/2017.
//
//

#ifndef ethraw_h
#define ethraw_h


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>


//PLATFORM DEPENDENT

#ifdef __linux__

#include <linux/if_packet.h>
#include <netinet/ether.h>
#include <sys/mman.h>
extern struct sockaddr_ll this_sockaddr;

#elif __APPLE__

#include <sys/types.h>
#include <sys/uio.h>
#include <fcntl.h>
#include <net/ethernet.h>
#include <net/bpf.h>
#include <net/if_dl.h>
#include <ifaddrs.h>

#endif



struct ethframe {
	struct ether_header header;
	unsigned char data[ETHER_MAX_LEN - ETHER_HDR_LEN];
	ssize_t len;
	ssize_t data_len;
};


// Some convenience constants
extern const size_t ETHER_PAYLOAD_LEN;


//global vars to be defined in source file
extern int bpf_buf_len;

extern char iface[IFNAMSIZ];
//mac addresses
extern unsigned char dest_mac[ETHER_ADDR_LEN];
extern unsigned char src_mac[ETHER_ADDR_LEN];
//interface protocol
extern unsigned short protocol;



//function prototypes

int open_interface(void);
int associate_interface(int device, const char* iface);
struct ethframe prepare_frame(unsigned char dest[ETHER_HDR_LEN],
							  unsigned char source[ETHER_HDR_LEN],
							  unsigned short proto);
int send_frame(int device, unsigned char* data, size_t data_len);
void read_frames(int device, void (*callback)(unsigned char*));



#endif /* ethraw_h */
