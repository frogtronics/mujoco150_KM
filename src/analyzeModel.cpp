//
//  analyseModel.cpp
//
//  Created by Enrico Eberhard


//  This code opens an xml or binary model and
//  prints out relevant information about it
//  to the terminal


#include "mujocoToolbox.h"
#include "unistd.h"
#include "getopt.h"
#include "stdio.h"
#include "string.h"
#include "math.h"


int parseArgsLong(int argc, const char **argv);

//-------------------------------- global variables -------------------------------------

// model
mjModel* m = 0;
mjData* d = 0;

int constraints = 0;

char *dfile = NULL;
char *ifile = NULL;
char *mfile = (char*)"../models/Kassina/Kassina - basu.xml";

//-------------------------------- main function ----------------------------------------

int main(int argc, const char** argv)
{
    //activate mujoco
    checkAndActivate();
	
	//initialize GLFW and window
	GLFWwindow* window = glfwInitWindow();
	if (!window)
		return 1;
	
	
	//-- parse input options --//
	parseArgsLong(argc, argv);
	
	
	
    // load model file
	loadmodel(window,mfile, 0);
	
	
	
	printf("-----------------\n");
	
	printf("Total degrees of freedom: %i\n",m->nv);
	printf("Total number of inputs for joints: %i\n",m->nq);
	printf("Total number of controls for joints: %i\n",m->nu);
	

    const char *JointTypes[] = { "mjJNT_FREE", "mjJNT_BALL", "mjJNT_SLIDE", "mjJNT_HINGE" };
	const char *TrnTypes[] = {"mjTRN_JOINT", "mjTRN_JOINTINPARENT", "mjTRN_SLIDERCRANK", "mjTRN_TENDON", "mjTRN_SITE"};
	
    printf("\nJoint information\n(ID)\tqAdr\tDoF\tTYPE\t\tNAME\n");
    for(int jointID=0;jointID<m->njnt;jointID++)
    {
		printf("(%i)\t%i\t%i\t%s\t%s\n",jointID,
			   m->jnt_qposadr[jointID],
			   m->jnt_dofadr[jointID],
			   JointTypes[m->jnt_type[jointID]],
			   mj_id2name(m, mjOBJ_JOINT, jointID));
    }
	printf("\n");
	
	
	if(m->nu > 0) {
		printf("\nActuator information\n(ID)\tTRN_ID\tTYPE\t\tNAME\n");
		for(int actID=0;actID<m->nu;actID++)
		{
			printf("(%i)\t%i:%i\t%s\t%s\n",actID,
				   m->actuator_trnid[2*actID], m->actuator_trnid[2*actID+1],
				   TrnTypes[m->actuator_trntype[actID]],
				   mj_id2name(m, mjOBJ_ACTUATOR, actID));
		}
		printf("\n");
	}
	
	if ((d->nefc > 0)&&(constraints)) {
	
		printf("\nConstraint information:\n");
		printf("Total number of constraints: %i\n",	d->nefc);
		
		
		printf("Constraint Jacobian in initial pose:\n");
		mju_printMat(d->efc_J, d->nefc, m->nv);
	}
	
		
	
	if (m->ntendon > 0) {
		
		printf("\nTendon information:\n");
		
		printf("Total number of tendons: %i\n",m->ntendon);
		
		
		printf("Tendon (id) and names:\n");
		for(int tendID = 0; tendID < m->ntendon; tendID++)
			printf("(%i) %s\n",tendID, mj_id2name(m, mjOBJ_TENDON, tendID));
		
	}
	
	
	if (0) {
		
		 for (int bod=0; bod < m->nbody; bod++)
			 printf("%s\t", mj_id2name(m, mjOBJ_BODY, bod));
		 printf("\n");
		 
		 mj_forward(m, d);
		 
		 printf("body mass:\n");
		 mju_printMat(m->body_mass,1,m->nbody);
		 printf("body CoMs:\n");
		 mju_printMat(m->body_ipos, 3, m->nbody);
		
		 
		 printf("%i inertias:\n", m->nM);
		 mju_printMat(d->qM, 1, m->nM);
		 
	}
	
	
	
	//printf("First tendon name: %s\n",m->name_tendonadr[0]);
	
	
    
    printf("\n-----------------\n");
	
	
	if (ifile && ifile[0]) //something in this string
		printf("Printing model info to %s\n", ifile);//mj_printModel(m, mfile);
	if (dfile && dfile[0])
		printf("Printing model data to %s\n", dfile);//mj_printData(m, d, dfile);
	
    // delete everything we allocated
    closeAndTerminate();
    
    return 0;
}


int parseArgsLong(int argc, const char **argv) {
	
	static struct option long_options[] =
	{
		{"constraints",	  no_argument,	&constraints, 1},
		{"data",	required_argument,	0, 'd'},
		{"info",	required_argument,	0, 'i'},
		{"model",	required_argument,	0, 'm'},
		{0, 0, 0, 0}
	};
	
	/* getopt_long stores the option index here. */
	int option_index = 0;
	int c;
	
	while ((c = getopt_long(argc, (char**)argv, "d:i:m:",
				long_options, &option_index)) != -1) {
		
	
		switch (c) {
			case 'c':
				printf ("Showing constraint info\n");
				constraints = 1;
				break;
			case 'd':
				printf ("Saving data to '%s'\n", optarg);
				dfile = optarg;
				break;
				
			case 'i':
				printf ("Saving info to '%s'\n", optarg);
				ifile = optarg;
				break;
				
			case 'm':
				printf ("Analysing model '%s'\n", optarg);
				mfile = optarg;
				break;
		}
	}
	
	
	/* Print any remaining command line arguments (not options). */
	if (optind < argc)
	{
		printf ("non-option ARGV-elements: ");
		while (optind < argc)
			printf ("%s ", argv[optind++]);
		putchar ('\n');
	}
	
	return 0;
	
}




