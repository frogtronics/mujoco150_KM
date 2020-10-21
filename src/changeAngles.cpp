//
//  changeAngles.cpp
//
//  Created by Enrico Eberhard, modified by Chris Richards, 2020
//
//	This is a simple forward kinematics
//	utility program - from a text file of
//	joint orientations, muscle lengths and
//	moment arms are reported


#include "mujocoToolbox.h"
#include "unistd.h"
#include "getopt.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "rollingJoint.h"
//#include "fileToolbox.h"


int parseArgsLong(int argc, const char **argv);

//-------------------------------- global variables -------------------------------------

// model
mjModel* m = 0;
mjData* d = 0;

//flags
int displayWindow = 1;

//input option args
char *ffile = NULL;//output foot file (not currently used)
char *ifile = NULL;//input quaternions file
char *tfile = NULL;//tendon length file
char *rfile = NULL;//moment arm file
char *map_file = (char*)"../models/Kassina/Kassina_map.txt";

//defaults
#ifdef _WIN32
char *mfile = (char*)"..\\models\\Kassina\\Kassina.xml";
//char *tfile = (char*)"..\\output\\kassinaTendons.txt";
#else
char *mfile = (char*)"../models/Kassina/Kassina.xml";
//char *tfile = (char*)"../output/kassinaTendons.txt";
#endif


//-------------------------------- main function ----------------------------------------

int main(int argc, const char** argv)
{
	//---- Initialization ----//
	
	//activate mujoco
	checkAndActivate();
	printf("USAGE: changeAngles -m modelfile -i inputfile\noptional additional arguments\n-t tendondataOutputdile \n-r momentarmOutputfile\n");
	//initialize GLFW and window
	GLFWwindow* window = glfwInitWindow();
	if (!window)
		return 1;
	
	
	//-- parse input options --//
	parseArgsLong(argc, argv);
	
	
	//open model file
	loadmodel(window,mfile, 0);

	
	
	//open input file
	FILE* inputFile;
	if (ifile)
		inputFile = fopen(ifile, "r");
	if (!inputFile) {
		//printf("Could not open input file %s\n",ifile);
		printf("FAILED\nUSAGE: changeAngles -m modelfile -i inputfile\noptional additional arguments\n-t tendondataOutputdile \n-r momentarmOutputfile\n");
		return 1;
	}
	
	
	//create output file for tendons
	FILE* tendonFile;
	if (tfile) {
		tendonFile = fopen(tfile, "w");
		if (!tendonFile) {
			printf("Could not create tendon file %s\n",tfile);
			return 1;
		}
	}
	
	//create output file for moment arms
	FILE* momentarmFile;
	if (rfile) {
		momentarmFile = fopen(rfile, "w");
		if (!momentarmFile) {
			printf("Could not create moment arm file %s\n",rfile);
			return 1;
		}
	}

	
	//create output file for foot pos if needed
	FILE* footFile;
	if (ffile) {
		footFile = fopen(ffile, "w");
		if (!footFile) {
			printf("Problem creating output file %s\n", ffile);
			return 1;
		}
	}
	

	//create output file for foot pos if needed
	FILE* mapfile;
	mapfile = fopen(map_file, "r");
	//CSV2qpos_mapped(m, mjtNum* q, int rows, int cols, FILE* file, std::string map_filename);

    printf("nbod %i\n", m->nbody);
    printf("nq = %i\n", m->nq);
    printf("nv = %i\n", m->nv);
    printf("nj = %i\n", m->njnt);
    printf("nu = %i\n", m->nu);
    printf("ngeom = %i\n", m->ngeom);
    printf("inertia matrix nM = %i\n", m->nM);


    for (int i = 0; i < m -> nbody; i++) {
        printf("body index %i, name %s  \n", i, mj_id2name(m, mjOBJ_BODY, i));//, m->(jnt_qposadr
    }   
    for (int i = 0; i < m -> njnt; i++) {
        printf("joint index %i, name %s  qpos address %i\n", i, mj_id2name(m, mjOBJ_JOINT, i), (m->jnt_qposadr)[i] );//, m->(jnt_qposadr
    }
    for (int i = 0; i < m -> njnt; i++) {
        printf("joint index %i, name %s  qvel address %i\n", i, mj_id2name(m, mjOBJ_JOINT, i), (m->jnt_dofadr)[i] );//, m->(jnt_qposadr
    }
    for (int i = 0; i < m -> ngeom; i++) {
        printf("geom index %i, name %s\n", i, mj_id2name(m, mjOBJ_GEOM, i) );//, m->(jnt_qposadr
    }
    // for (int i = 0; i < m -> nsite; i++) {
    //     printf("site index %i, name %s\n", i, mj_id2name(m, mjOBJ_SITE, i) );//, m->(jnt_qposadr
    // }
	
	std::string highlight_filename = "../input/muscle_highlight.txt";// This file shows which trial set to point to and how to name output files
    //std::string marker_filename = trialSetInfo[0].c_str();
    int n_highlights = numberOfLines(highlight_filename);
    std::string *highlight_list = new std::string[n_highlights];
    file2StringList(highlight_filename, highlight_list);
    bool highlightQ = false;
    bool allsitesQ = false;
    if (n_highlights > 0)
    {
    	highlightQ = true;
    }
     if (highlight_list[0] == "allsites")
    {
    	allsitesQ = true;
    }
	
	//---- Pre-processing ----//
	
	//determine size of input file
	int rows, cols;
	sizeofCSV(&rows, &cols, inputFile);
	printf("Rows: %i, Cols: %i\n",rows,cols);
	
	
	//allocate array now that we know the size of the input
	//...use model size nq rather than cols
	
	#ifdef _WIN32	//windows is weird about compiling unkown array size
	mjtNum q[10000];//enough for 344 input rows with Kassina model (nq = 29)
	#else
	mjtNum q[rows*m->nq];
	#endif
	
	//get positions from file
	//CSV2qpos(m,q,rows,cols,inputFile);
	CSV2qpos_mapped(m,q,rows,cols,inputFile, "../input/mapdata_k.txt", "../input/mapdata_m.txt");

	
	
	
	//print header row for output file
	//basically, each tend1 length, moment0,...momentM, ... tendN length, moment...
	int tID,tMmtID, jID;
	int dof, jdof;
	
	const char *axes = "123xyz";
	
	if(tfile) {
		
	for(tID = 0; tID < m->ntendon; tID++) {
		
		fprintf(tendonFile,"%s_len,",mj_id2name(m, mjOBJ_TENDON, tID));
		
		for(tMmtID = tID*m->nv; tMmtID < (tID+1)*m->nv; tMmtID++) {
			
			dof = tMmtID%m->nv; jID = m->dof_jntid[dof];
			jdof = dof - m->jnt_dofadr[jID];
			
			fprintf(tendonFile,"%s_mmt_%s",
					mj_id2name(m, mjOBJ_TENDON, tID), //tendon name
					mj_id2name(m, mjOBJ_JOINT, m->dof_jntid[dof])); //joint name
			
			//for ball and free joints, append an x/y/z
			if (m->jnt_type[jID] == mjJNT_BALL)
				 fprintf(tendonFile,"_%c",axes[jdof+3]);
			else if (m->jnt_type[jID] == mjJNT_FREE)
				fprintf(tendonFile,"_%c",axes[jdof]);

			
			if( (tID >= m->ntendon-1) && (tMmtID >= (tID+1)*m->nv - 1) )
				fprintf(tendonFile,"\n");
			else
				fprintf(tendonFile,",");
			
		}
	}
		
	}

	
	//get id of sites
	int origin = mj_name2id(m, mjOBJ_BODY, "Spine");
	int contact = mj_name2id(m, mjOBJ_SITE, "TarsalsLZ");
	int contact_mir = mj_name2id(m, mjOBJ_SITE, "TarsalsLZ_mir");
	
	
	bool firstTime = true;
	int r = 0;
	
    if( displayWindow )
		render(window);
	

	//find muscle highlights from highlights file
	int muscle_highlight_id;
	int highlight_id_list[n_highlights];
	for (int i = 0; i < n_highlights; i++) 
	{
		int muscle_highlight_id = mj_name2id(m, mjOBJ_TENDON, highlight_list[i].c_str());
		highlight_id_list[i] = muscle_highlight_id;
		printf("highlighted tendons: %s, id = %i\n", highlight_list[i].c_str(), muscle_highlight_id);
	}
	//find site highlights from highlights file
	int site_highlight_id;
	int site_id_list[n_highlights];
	for (int i = 0; i < n_highlights; i++) 
	{
		int site_highlight_id = mj_name2id(m, mjOBJ_SITE, highlight_list[i].c_str());
		site_id_list[i] = site_highlight_id;
		printf("highlighted sites: %s, id = %i\n", highlight_list[i].c_str(), site_highlight_id);
	}
	//find geom highlights from highlights file
	int geom_highlight_id;
	int geom_id_list[n_highlights];
	for (int i = 0; i < n_highlights; i++) 
	{
		int geom_highlight_id = mj_name2id(m, mjOBJ_GEOM, highlight_list[i].c_str());
		geom_id_list[i] = geom_highlight_id;
		printf("highlighted sites: %s, id = %i\n", highlight_list[i].c_str(), geom_highlight_id);
	}
	//-------------------------------
	//-------------------------------
	// Initialise pose --------------
	//-------------------------------

	mju_copy(d->qpos,&q[0*m->nq],m->nq);
	mj_forward(m,d);
	// main loop
	while( !glfwWindowShouldClose(window) ) {
		
		if(!paused) {

			//-------------------------------
			//-------------------------------
			// Make all tendons transparent 
			//except for one(s) to highlight
			//-------------------------------
			//-------------------------------
			if (highlightQ)
			{
				//---------HIGHLIGHT TENDONS
				// first make all tendons transparent
				for (int i = 0; i <  m->ntendon; i ++ )
				{
					for (int j = 0; j < 4; j ++)
					{
						if (j == 3)
						{
							m->tendon_rgba[ i * 4 + j] = 0;
						}
					}
				}
				// fnow highlight specified tendns
				for (int i = 0; i <  n_highlights; i ++ )
				{
					for (int j = 0; j < 4; j ++)
					{
						if (j == 3)
						{
							m->tendon_rgba[ highlight_id_list[i] * 4 + j] = 1;
						}
					}
				}
				//---------HIGHLIGHT SITES
				// first make all sites transparent
				if (!allsitesQ)
				{
					for (int i = 0; i <  m->nsite; i ++ )
					{
						for (int j = 0; j < 4; j ++)
						{
							if (j == 3)
							{
								m->site_rgba[ i * 4 + j] = 0;
							}
						}
					}
				}
				// fnow highlight specified sites
				for (int i = 0; i <  n_highlights; i ++ )
				{
					for (int j = 0; j < 4; j ++)
					{
						if (j == 3)
						{
							m->site_rgba[ site_id_list[i] * 4 + j] = 1;
						}
					}
				}
				//---------HIGHLIGHT GEOMS
				// fnow highlight specified geoms
				for (int i = 0; i <  n_highlights; i ++ )
				{
					for (int j = 0; j < 4; j ++)
					{
						if (j == 3)
						{
							m->geom_rgba[ geom_id_list[i] * 4 + j] = 1;
						}
					}
				}


			}
			//-------------------------------
			//-------------------------------
			//-------------------------------
			//-------------------------------
			//-------------------------------
			//-------------------------------


			//------- experimental - try a knee slide joint
			int slideX_joint = mj_name2id(m, mjOBJ_JOINT, "j_kneeL_rollingX");
			int slideX_index = m->jnt_qposadr[slideX_joint];
			int slideY_joint = mj_name2id(m, mjOBJ_JOINT, "j_kneeL_rollingY");
			int slideY_index = m->jnt_qposadr[slideY_joint];
			int slideZ_joint = mj_name2id(m, mjOBJ_JOINT, "j_kneeL_rollingZ");
			int slideZ_index = m->jnt_qposadr[slideZ_joint];
			//printf("%i\n", r);

			

			mju_copy(d->qpos,&q[r*m->nq],m->nq);
			//mju_copy(d->qpos,&q[0*m->nq],m->nq);// for testing

			// // ----------
			// // ----------
			// // FOR TESTING: isolate a joint - only have that one joint move and others are fixed at a frame
			// // ----------
			// // ----------
			// int knee_id = mj_name2id(m, mjOBJ_JOINT, "j_kneeL");
			// int knee_dof_id = m->jnt_qposadr[knee_id];	
   //  		int joint_sizes[4] = {7, 4, 1, 1};
   //  		int knee_size = joint_sizes[m->jnt_type[knee_id]];
   //  		// animate knee joint, but leave all other joints frozen at frame 1
   //  		for (int i = 0; i < knee_size; i ++)
   //  		{
   //  			d->qpos[knee_dof_id + i] = q[r * m->nq + knee_dof_id + i];
   //  		}
			//printf("%i\n", knee_size);	
			// ----------
			// ----------
			// Get knee 3D angle
			// ----------
			// ----------
			int hip_marker_id = mj_name2id(m, mjOBJ_SITE, "s_ctr_hipL");
			int kne_marker_id = mj_name2id(m, mjOBJ_SITE, "s_ctr_kneL");
			int ank_marker_id = mj_name2id(m, mjOBJ_SITE, "s_ctr_ankL");
			int test_marker_id = mj_name2id(m, mjOBJ_SITE, "s_test");	

			mjtNum hipXYZ[3];
			mjtNum kneXYZ[3];
			mjtNum ankXYZ[3];
			mjtNum fem_vec[3];
			mjtNum tib_vec[3];

			//printf("%i  %i   %i\n", hip_marker_id, kne_marker_id, ank_marker_id);	
			// get XYZ coordinates of hip, knee and ankle markers	
			for (int i = 0; i < 3; i ++)
			{
				hipXYZ[i] = d->site_xpos[hip_marker_id * 3 + i];
				kneXYZ[i] = d->site_xpos[kne_marker_id * 3 + i];
				ankXYZ[i] = d->site_xpos[ank_marker_id * 3 + i];
			}



			mju_sub3(fem_vec, hipXYZ, kneXYZ);
			mju_sub3(tib_vec, ankXYZ, kneXYZ);
			mju_normalize3(fem_vec);
			mju_normalize3(tib_vec);

			mjtNum kne_angle = unitVectorAngle(tib_vec, fem_vec, 3);
			//printf("%f\n", kne_angle);

			mjtNum rollingXYZ[3];

			//------FOR TESTING 
			// USE THIS FOR TESTING AND ADJUSTING POSITION OF ELLIPSE
			//-----------------
			///----------------
			//mju_copy(d->qpos,&q[0*m->nq],m->nq);
			//rollingCenter(rollingXYZ, mj_id2name(m, mjOBJ_JOINT, slideX_joint), ((float)r )/rows);
			jointAngle2Roll(rollingXYZ, mj_id2name(m, mjOBJ_JOINT, slideX_joint), kne_angle);
			d->qpos[slideX_index] = -rollingXYZ[0];
			d->qpos[slideY_index] = -rollingXYZ[1];
			d->qpos[slideZ_index] = -rollingXYZ[2];
			//}
			//int test_geom_id = mj_name2id(m, mjOBJ_GEOM, "g_test");


			//-----------------
			///---------------			
			///----------------



			mj_forward(m,d);

			// for (int i = 0; i < 3; i ++)
			// {
			// 	d->site_xpos[test_marker_id * 3 + i] = ankXYZ[i];
			// }


			//on the first loop through, print all tendon information to file
			if(firstTime) {
				
				if(tfile) {
					//first line is header with labels, now just print numbers
					// for(tID = 0; tID < m->ntendon; tID++)
					// {
					// 	fprintf(tendonFile,"%f,", d->ten_length[tID]);
						
					// 	for(tMmtID = tID*m->nv; tMmtID < (tID+1)*m->nv; tMmtID++) {
					// 		fprintf(tendonFile,"%f",d->ten_moment[tMmtID]);
							
					// 		if( (tID >= m->ntendon-1) && (tMmtID >= (tID+1)*m->nv - 1) )
					// 			fprintf(tendonFile,"\n");
					// 		else
					// 			fprintf(tendonFile,",");
					// 	}
					// }
					for (int i = 0; i < (m->ntendon);i++) {
						fprintf(tendonFile, "%2.6f", d->ten_length[i]);
						if (i < (m->ntendon-1))
							fprintf(tendonFile, ",");
						else
							fprintf(tendonFile, "\n");
					}
				}
				if (rfile) {
					for(int ntendof = 0; ntendof < (m->nv)*(m->ntendon); ntendof++) {
						fprintf(momentarmFile,"%2.6f", d->ten_moment[ntendof]);
						if (ntendof < ((m->nv)*(m->ntendon) -1))
							fprintf(momentarmFile,",");
						else
							fprintf(momentarmFile,"\n");
					}
				}

				
				
				if(ffile) {
				
					fprintf(footFile,"%f,%f,%f,",d->xpos[origin*3],
							d->xpos[origin*3+1],d->xpos[origin*3+2]);
					fprintf(footFile,"%f,%f,%f,",d->site_xpos[contact*3],
							d->site_xpos[contact*3+1],d->site_xpos[contact*3+2]);
					fprintf(footFile,"%f,%f,%f\n",d->site_xpos[contact_mir*3],
							d->site_xpos[contact_mir*3+1],d->site_xpos[contact_mir*3+2]);
					
				}
			}
			
			
			//loop around
			if (r<rows-1)
				r++;
			else
			{
				r=0;
				
				if(firstTime) {
					
					if(tfile) {
						printf("First loop complete - printed tendon lengths to file\n");
						fclose(tendonFile);
					}
					
					if(displayWindow == 0) {
						printf("Done!\n");
						closeAndTerminate();
						return 0;
					}
					
					firstTime = false;
				}
			}
			
			
		} //end if(!paused)
		
		
		
		if( displayWindow )
		{
			// update the window
			render(window);

			// handle events (this calls all callbacks)
			glfwPollEvents();
		}
		
		
		waitSeconds(0.05);
		
	}
    
	
     
    
    

    // delete everything we allocated
    closeAndTerminate();
    
    return 0;
}


int parseArgsLong(int argc, const char **argv) {
	
	static struct option long_options[] =
	{
		{"hide",	no_argument,		&displayWindow, 0},
		{"model",	required_argument,	0, 'm'},
		{"input",	required_argument,	0, 'i'},
		{"momentarm", required_argument, 0, 'r'},
		// {"foot",	required_argument,	0, 'f'},
		{"tendon",	required_argument,	0, 't'},
		{0, 0, 0, 0}
	};
	
	/* getopt_long stores the option index here. */
	int option_index = 0;
	int c;
	
	while ((c = getopt_long(argc, (char**)argv, "hf:m:t:i:r:",
							long_options, &option_index)) != -1) {
		
		
		switch (c) {
			case 'h':
				displayWindow = 0;
				break;
				
			// case 'f':
			// 	printf ("Saving foot pos to '%s'\n", optarg);
			// 	ffile = optarg;
			// 	break;
				
			case 'm':
				printf ("Using model '%s'\n", optarg);
				mfile = optarg;
				break;

			case 'r':
				printf ("Saving moment arm info to '%s'\n", optarg);
				rfile = optarg;
				break;
				
			case 't':
				printf ("Saving tendon info to '%s'\n", optarg);
				tfile = optarg;
				break;
				
			case 'i':
				ifile = optarg;
				printf ("Using input file '%s'\n", optarg);
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
					mfile = (char *)argv[optind++];
				case 2:
					ifile = (char *)argv[optind++];
			}
		}
	}
	
	return 0;
	
}


