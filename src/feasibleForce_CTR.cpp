//
//  feasibleForce.cpp
//
//  Created by Enrico Eberhard
//
//	This is a program to calculate a feasible force
//	polyhedron


#include "mujocoToolbox.h"
#include "unistd.h"
#include "getopt.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

int setPose(char *file, int row);
int eeFileWrite(char* filename, mjtNum* fEE);
void updateEEForces(mjtNum* fEE, bool verbose);
void updateMMTandJac(mjtNum* mmt, mjtNum* jac);
int mmtJacFileWrite(mjtNum* mmt, mjtNum* jac);
int parseArgsLong(int argc, const char **argv);

//-------------------------------- global variables -------------------------------------

// model
mjModel* m = 0;
mjData* d = 0;

//flags
int displayWindow = 1;

int poseRow = 0;

//input option args
char *mfile = NULL;
char *ifile = NULL;
char *efile = NULL;

int currentrow = 0;
FILE* eeFileOutput;

//-------------------------------- main function ----------------------------------------

int main(int argc, const char** argv)
{
	//---- Initialization ----//
	
	//activate mujoco
	checkAndActivate();
	
	//initialize GLFW and window
	GLFWwindow* window = glfwInitWindow();
	if (!window)
		return 1;
	
	
	//parse input options
	parseArgsLong(argc, argv);
	
	//open model file
	loadmodel(window,mfile, 0);
	

	//FILE* mmtFile = fopen("~/Desktop/SEBDATA/KM04_HOP_07_MMT.txt", "w");
	
	eeFileOutput = fopen("../input/tendondata.txt", "w");
	//---- Acquisition of Data ----//

	int r, ncol, nrow = 1;
	
	FILE* inputFile = fopen(ifile, "r");
	if (!inputFile) {
		printf("Could not open input file %s\n",ifile);
		return 1;
	}
	
	if(!poseRow) {
		poseRow = 1;
		sizeofCSV(&nrow, &ncol, inputFile);
		fclose(inputFile);
	}
	
	printf("Writing EE for input pose rows %i to %i (%i total rows)\n",
		   poseRow, poseRow + nrow, nrow*3);
	
	for(r=poseRow; r<poseRow + nrow; r++) {
		currentrow = r;
		setPose(ifile, r);
		
		mjtNum fEE[3*m->ntendon];
		updateEEForces(fEE, (bool)(poseRow-1));
		eeFileWrite(efile, fEE);
		
		/*
		mjtNum mmt[m->ntendon*m->nv];
		mjtNum jac[3*m->nv];
		
		updateMMTandJac(mmt, jac);
		if(mmtJacFileWrite(d->ten_moment, jac))
			return -1;
		*/
		
	}
	
	
	
	
	
	//---- Render model if required, otherwise finish  ----//
	
	
	if( !displayWindow ) {
		closeAndTerminate();
		return 0;
	}
	
	
	
	paused = true;
	render(window);
	
	// main loop
	while( !glfwWindowShouldClose(window) ) {
		
		
		//simulateFrame(m,d);
		
		//updateEEForces();
		
		// update the window
		render(window);

		// handle events (this calls all callbacks)
		glfwPollEvents();
		
	}

    // delete everything we allocated
    closeAndTerminate();
    return 0;
}



void updateEEForces(mjtNum* fEE, bool verbose) {
	
	//---- Acquisition of Data ----//
	
	static const int ee = mj_name2id(m, mjOBJ_SITE, "EE");
	static const int ee_body = mj_name2id(m, mjOBJ_BODY, "TarsalsL");
	//static const int ee_body = m->site_bodyid[ee];
	
	int tID, id;
	mjtNum scaleForce = 1;
	
	
	mjtNum jacp[3*m->nv], jacr[3*m->nv];
	
	mjtNum tEE[3*m->ntendon];
	mjtNum maxf[m->ntendon*m->ntendon];
	//mjtNum fEE[3*m->ntendon];
	
	

	
	//get jacobian for end-effector
	mj_jac(m, d, jacp, jacr, &d->site_xpos[ee*3], ee_body);
	
	//multiply moments by jacobian to get a tendon force to EE force transform
	mju_mulMatMatT(tEE, jacp, d->ten_moment, 3, m->nv, m->ntendon);
	
	//make an ntend x ntend diagonal matrix of tendon max forces
	mju_zero(maxf, m->ntendon*m->ntendon);
	for(tID = 0; tID < m->ntendon; tID++) {
		maxf[tID*m->ntendon + tID] = m->tendon_user[tID*m->nuser_tendon];
	}
	
	
	//multiply unit EE force transform by negated max forces
	//	(because contraction is negative force)
	mju_scl(maxf, maxf, -scaleForce, m->ntendon*m->ntendon);
	mju_mulMatMat(fEE, tEE, maxf, 3, m->ntendon, m->ntendon);
	
	
	
	
	/*/update site positions to visualize force vectors
	mjtNum fvT[m->ntendon*3];
	mju_transpose(fvT, fv, 3, m->ntendon);
	for (tID=0; tID<m->ntendon; tID++) {
		id = jID[tID];
		if(id != -1) {
			mju_copy(&d->qpos[id], &fvT[tID*3], 3);
			mju_add(&d->qpos[id], &d->qpos[id], &d->site_xpos[ee*3], 3);
		}
	}
	/*/
	
	
	if (verbose) {
		
		//printf("Tendon moment arms (ntend x ndof):\n");
		//mju_printMat(d->ten_moment, m->ntendon, m->nv);
		
		mjtNum p[3];
		//get jacobian for end-effector
		mju_copy3(p, &d->site_xpos[ee*3]);
		printf("EE at %f, %f, %f, body id %i\n",p[0], p[1], p[2], ee_body);
		
		printf("EE Jacobian (3 x ndof):\n");
		mju_printMat(jacp, 3, m->nv);
		
		
		printf("EE unit force transform (3 x ntend):\n");
		mju_printMat(tEE, 3, m->ntendon);
		
		printf("Tendon max forces (1 x ntend):\n");
		mju_printMat(m->tendon_user, 1, m->ntendon);
		
		printf("\n\n\nTendon forces on EE (3 x ntend):\n");
		mju_printMat(fEE, 3, m->ntendon);
		
		
	}
	
}

//mmt (ntend x ndof) and jac (3 x ndof)
void updateMMTandJac(mjtNum* mmt, mjtNum* jac) {
	
	//---- Acquisition of Data ----//
	
	static const int ee = mj_name2id(m, mjOBJ_SITE, "EE");
	static const int ee_body = mj_name2id(m, mjOBJ_BODY, "TarsalsL");
	//static const int ee_body = m->site_bodyid[ee];
	
	int tID, id;
	mjtNum scaleForce = 1;
	
	
	mjtNum jacp[3*m->nv], jacr[3*m->nv];
	
	mjtNum tEE[3*m->ntendon];
	mjtNum maxf[m->ntendon*m->ntendon];
	//mjtNum fEE[3*m->ntendon];
	
	
	
	
	//get jacobian for end-effector
	mj_jac(m, d, jac, jacr, &d->site_xpos[ee*3], ee_body);
	
	
	//make an ntend x ntend diagonal matrix of tendon max forces
	mju_zero(maxf, m->ntendon*m->ntendon);
	for(tID = 0; tID < m->ntendon; tID++) {
		maxf[tID*m->ntendon + tID] = m->tendon_user[tID*m->nuser_tendon];
	}
	
	//multiply moment arms by negated max forces
	//	(because contraction is negative force)
	mju_scl(maxf, maxf, -scaleForce, m->ntendon*m->ntendon);
	mju_mulMatMat(mmt, maxf, d->ten_moment, m->ntendon, m->ntendon, m->nv);
	
	
	
	
	/*/update site positions to visualize force vectors
	 mjtNum fvT[m->ntendon*3];
	 mju_transpose(fvT, fv, 3, m->ntendon);
	 for (tID=0; tID<m->ntendon; tID++) {
		id = jID[tID];
		if(id != -1) {
	 mju_copy(&d->qpos[id], &fvT[tID*3], 3);
	 mju_add(&d->qpos[id], &d->qpos[id], &d->site_xpos[ee*3], 3);
		}
	 }
	 /*/
	
}

			
int setPose(char *file, int row) {
	
	mjtNum q[m->nq];

	//get id of ee joint
	int ee_joint = mj_name2id(m, mjOBJ_JOINT, "EE");

	//determine model state

	if (file) {
		int ret;
		if((ret=getPoseFromFile(m, file, q, row)))
			return ret;
	}
	else
		mju_unit4(q);


	//set EE position
	//mju_copy(&d->qpos[m->jnt_qposadr[ee_joint]], &q[m->nq - 7], 3);


	//set model state
	mju_copy(d->qpos,q,m->nq);
	mj_forward(m,d);
	
	return row;
}

int eeFileWrite(char* filename, mjtNum* fEE) {

	
	//create output file for end-effector transform
	//static FILE* eeFile = NULL;
	//printf("%i\n", currentrow);
	static FILE* eeFile;
	eeFile = fopen("../input/tendondata.txt", "w");
	
	if (!eeFile && filename) {
		eeFile = fopen(filename, "w");
		printf("not eeFile \n");
		if (!eeFile) {
			printf("Problem creating output file %s\n", filename);
			return 1;
		}
	}
	
	int xyz, tID;
	if (eeFile) {

		for(xyz = 0; xyz < 3; xyz++) {
			for(tID = 0; tID < m->ntendon-1; tID++)
				fprintf(eeFileOutput,"%2.5f,", fEE[xyz*m->ntendon + tID]*1000.0);
			printf("%i,   %f\n", currentrow, abs(fEE[xyz*m->ntendon + tID]*1000 ));
			fprintf(eeFileOutput,"%2.5f\n", 1.0);//fEE[xyz*m->ntendon + tID]*1000);
		}

		// for(xyz = 0; xyz < 3; xyz++) {
		// 	for(tID = 0; tID < m->ntendon-1; tID++)
		// 		//fprintf(eeFileOutput,"%f,",fEE[xyz*m->ntendon + tID]*1000);
		// 		fprintf(eeFileOutput,"%i,", currentrow);
		// 	if (xyz == 2) {
		// 		fprintf(eeFileOutput,"%f\n",fEE[xyz*m->ntendon + tID]*1000);
		// 	}
		// 	else
		// 		fprintf(eeFileOutput,"%f\n",fEE[xyz*m->ntendon + tID]*1000);
		// }


	}
	
	return 0;
}

int mmtJacFileWrite(mjtNum* mmt, mjtNum* jac) {
	//create output file for end-effector transform
	static FILE* mmtFile = NULL;
	static FILE* jacFile = NULL;
	
	if (!mmtFile) {
		mmtFile = fopen("../output/KM04_HOP_07_MMT.txt", "w");
		if (!mmtFile) {
			printf("Problem creating output file\n");
			return 1;
		}
	}
	
	if (!jacFile) {
		jacFile = fopen("../output/KM04_HOP_07_JAC.txt", "w");
		if (!jacFile) {
			printf("Problem creating output file\n");
			return 1;
		}
	}
	
	
	
	int dof, tID;
	if (mmtFile) {
		for(tID = 0; tID < m->ntendon; tID++) {
			for(dof = 0; dof < m->nv-1; dof++)
				fprintf(mmtFile,"%f,",mmt[tID*m->nv + dof]);
			
			fprintf(mmtFile,"%f\n",mmt[tID*m->nv + dof]);
		}
	}
	
	int xyz;
	if (jacFile) {
		for(xyz = 0; xyz < 3; xyz++) {
			for(dof = 0; dof < m->nv-1; dof++)
				fprintf(jacFile,"%f,",jac[xyz*m->nv + dof]);
			
			fprintf(jacFile,"%f\n",jac[xyz*m->nv + dof]);
		}
	}
	
	return 0;
}

int parseArgsLong(int argc, const char **argv) {
	
	
	static struct option long_options[] =
	{
		{"hide",	no_argument,		&displayWindow, 0},
		{"model",	required_argument,	0, 'm'},
		{"input",	required_argument,	0, 'i'},
		{"output",	required_argument,	0, 'o'},
		{"pose",	required_argument,	0, 'p'},
		{0, 0, 0, 0}
	};
	
	/* getopt_long stores the option index here. */
	int option_index = 0;
	int c;
	
	while ((c = getopt_long(argc, (char**)argv, "hm:i:o:p:",
							long_options, &option_index)) != -1) {
		
		
		switch (c) {
			case 'h':
				displayWindow = 0;
				break;
				
			case 'm':
				printf ("Using model '%s'\n", optarg);
				mfile = optarg;
				break;
				
			case 'i':
				ifile = optarg;
				printf ("Using input file '%s'\n", optarg);
				break;
				
			case 'o':
				efile = optarg;
				printf ("Saving weighted end-effector transform output to '%s'\n", optarg);
				break;
				
			case 'p':
				poseRow = atoi(optarg);
				printf ("Using only row %i of input file\n", poseRow);
				break;
		}
	}
	
	
	/* Print any remaining command line arguments (not options). 
		treat this as model_file input_file [ignore] */
	
	int nonopts;
	
	if (optind < argc)
	{
		nonopts = 1;
		while (optind < argc) {
			
			switch (nonopts) {
				case 1:
					if (!mfile)
						mfile = (char *)argv[optind++];
				case 2:
					if (!ifile)
						ifile = (char *)argv[optind++];
			}
		}
	}
	
	
	//defaults
	
	if (!mfile) {
		
		#ifdef _WIN32
		char *mfile = (char*)"..\\models\\Kassina\\Kassina.xml";
		#else
		char *mfile = (char*)"../models/Kassina/Kassina.xml";
		#endif
		
	}
	
	
	return 0;
	
}


