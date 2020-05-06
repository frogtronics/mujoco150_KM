//
//  simpipe.cpp
//
//  Created by Enrico Eberhard
//
//	SimPipe simulates passive and active dynamics,
//  and pipes the resulting state information via
//  raw ethernet packets


#include "mujocoToolbox.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "getopt.h"
#include "ethraw.h"

// model
mjModel* m = 0;
mjData* d = 0;


mjtNum Fx = 0;
mjtNum Fy = 0;
mjtNum Ftheta = 0;
void read_callback(unsigned char* data);

struct STATE {
	float x;
	float y;
	float z;
	float xv;
	float yv;
	float zw;
};


int parseArgsLong(int argc, const char **argv);




void getPortalState(mjModel* m, mjData* d, struct STATE* state) {
	
	static int portal = mj_name2id(m, mjOBJ_SITE, "portal");
	static int portalUP = mj_name2id(m, mjOBJ_SITE, "portalUP");
	static int portalv = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "portalv")];
	static int portalw = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "portalw")];
	
	
	state->x = d->site_xpos[portal*3];
	state->y = d->site_xpos[portal*3 + 2];
	state->z = atan2(d->site_xpos[portalUP*3 + 2] - state->y, d->site_xpos[portalUP*3] - state->x) - M_PI_2;

	state->xv = d->sensordata[portalv];
	state->yv = d->sensordata[portalv + 2];
	state->zw = d->sensordata[portalw + 1];
}

enum RUN_MODE {
	SEND,
	RECEIVE
} run_mode = SEND;

void print_data(unsigned char *data) {
	printf("%s\n", data);
}

int main(int argc, const char** argv)
{
	printf("+---------+\n| SIMPIPE |\n+---------+\n");
	
	//set default
	strncpy(iface, "lo0", IFNAMSIZ);
	protocol = 0x1234;

	parseArgsLong(argc, argv);
	
	
	printf("Opening device...\n");
	int device = open_interface();

	printf("Associating interface %s\n", iface);
	associate_interface(device, iface);
	
	//printf("Source MAC %s, destination MAC %s\n", src_mac_str, dest_mac_str);
	printf("Source MAC - %02x:%02x:%02x:%02x:%02x:%02x\n",
			src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5]);
	
	if (run_mode==RECEIVE)
		while(1)
			read_frames(device, print_data);
	
	
	//activate mujoco
	checkAndActivate();
	
	//initialize GLFW and window
	GLFWwindow* window = glfwInitWindow();
	if (!window)
		return 1;
	
	//load model
	loadmodel(window,"../models/pendulum.xml", 0);
	
	char data[ETHER_PAYLOAD_LEN];
	
	STATE state;
	int portalBody = m->site_bodyid[mj_name2id(m, mjOBJ_SITE, "portal")];
	
	paused = false;

	//loop - simulate
	printf("Starting simulation loop\n");
	while(!glfwWindowShouldClose(window))
	{
		
		if(!paused) {
			

			read_frames(device, read_callback);
			

			// slow motion factor: 10x
			mjtNum factor = (slowmotion ? 10 : 1);
			
			// advance effective simulation time by 1/refreshrate
			mjtNum startsimtm = d->time;
			while((d->time-startsimtm)*factor < 1.0/refreshrate) {
				
				//apply perturbations
				mju_zero(d->xfrc_applied, 6*m->nbody);
				if( pert.select>0 )
					mjv_applyPerturbForce(m, d, &pert);

				//add received x,y force and z torque to xfrc_applied
				d->xfrc_applied[portalBody+0] += Fx;
				d->xfrc_applied[portalBody+2] += Fy;
				d->xfrc_applied[portalBody+4] += Ftheta;
				

				// run mj_step and count
				mj_step(m, d);
				
				
			}
			
			//get position and orientation of box body
			getPortalState(m, d, &state);

			//only send this a few times per second for now
			sprintf(data,"X: %+7.2f, Y: %+7.2f, Z: %+5.2f, U: %+5.2f, V: %+5.2f, W: %+5.2f",
					state.x*1000, state.y*1000,state.z,
					state.xv*1000,state.yv*1000,state.zw);

			send_frame(device, (unsigned char *)data, (size_t)strlen((const char*)data));

			

			
		}
		else {
			
			// apply pose perturbations, run mj_forward
			mjv_applyPerturbPose(m, d, &pert, 1);       // move mocap and dynamic bodies
			mj_forward(m, d);
			
		}
		
		// update the window
		render(window);
		
		// handle events (this calls all callbacks)
		glfwPollEvents();
		
		
	}
	
	// delete everything we allocated
	closeAndTerminate();
	
	
	
	return 0;


}

void read_callback(unsigned char* data) {
	//parse data into global mjtNums
	
	float rFx, rFy, rFtheta;
	
	printf("%s\n",data);
	
	if(3==sscanf((const char*)data, "X: %7f, Y: %7f, Z: %5f", &rFx, &rFy, &rFtheta)) {
		Fx = (mjtNum)rFx;
		Fy = (mjtNum)rFy;
		Ftheta = (mjtNum)rFtheta;
	}
	
	//use d->site_pos and d->site_quat to transform force/torque to body frame
	

}



const char help_str[] =
"SIMPIPE HELP\n"
"\t A program for simulating and piping state information over ethernet\n"
""
"Usage:\n"
"\t sudo ./simpipe [-opt] [arg] [[-opt2] [arg2] ...]\n"
"Options:\n"
"\t -d | --dest  {xx:xx:xx:xx:xx:xx}   \t Supply a destination MAC address\n"
"\t -i | --iface {interface e.g. eth0} \t Specify a network interface\n"
"\t -p | --proto {protocol 0x0800}     \t Specify a 2byte protocol in hex format\n"
"\n"
"\t -h | --help         \t Shows options\n"
"\t -r | --receive      \t For debug only, sets receive mode\n"
"\t -s | --send			\t For debug only, sets send mode\n"
;


int parseArgsLong(int argc, const char **argv) {
	
	static struct option long_options[] =
	{
		{"dest",		required_argument, 0, 'd'},
		{"iface",		required_argument, 0, 'i'},
		{"proto",		required_argument, 0, 'p'},
		{"help",		no_argument,       0, 'h'},
		{"receive",		no_argument,	   0, 'r'},
		{"send",		no_argument,	   0, 's'},
		{0, 0, 0, 0}
	};
	
	static const char* short_options = "d:i:p:hrs";
	
	// getopt_long stores the option index here.
	int option_index = 0;
	
	int c;
	
	while ((c = getopt_long(argc, (char**)argv,
							short_options,
							long_options,
							&option_index))		!= -1)
	{
		
		switch (c)
		{
			case 0:
				// If this option set a flag, do nothing else now.
				if (long_options[option_index].flag != 0)
					break;
				printf ("option %s", long_options[option_index].name);
				if (optarg)
					printf (" with arg %s", optarg);
				printf ("\n");
				break;
				
			case 'd':
				int i;
				for(i=0;i<ETHER_ADDR_LEN-1;i++)
					sscanf(optarg + 3*i, "%2hhx:", &dest_mac[i]);
				sscanf(optarg + 3*i, "%2hhx", &dest_mac[i]);
				
				printf("-> destination set to %02x:%02x:%02x:%02x:%02x:%02x\n",
					   dest_mac[0], dest_mac[1], dest_mac[2], dest_mac[3], dest_mac[4], dest_mac[5]);
				
				break;
				
			case 'i':
				strncpy(iface, optarg, IFNAMSIZ);
				printf("-> iface set to %s\n", iface);
				break;
				
			case 'p':
				sscanf(optarg, "0x%4hx", &protocol);
				printf("-> protocol set to 0x%.4x\n", protocol);
				break;
				
			case 'h':
				printf(help_str);
				break;
				
			case 'r':
				run_mode = RECEIVE;
				break;
				
			case 's':
				run_mode = SEND;
				break;
				
			case '?':
				// getopt_long already printed an error message.
				printf("Use -h or --help for options\n");
				break;
				
			default:
				abort ();
		}
	}
	
	
	// Print any remaining command line arguments (non-options).
	if (optind < argc)
	{
		printf ("non-option ARGV-elements: ");
		while (optind < argc)
			printf ("%s ", argv[optind++]);
		putchar ('\n');
	}
	
	printf("\n");
	
	return 0;
	
}


