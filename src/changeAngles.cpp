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
#include <unistd.h>
#include "getopt.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "jointToolbox.h"
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
char *kfile = (char*)"../input/KM06_HOP_08.txt";//NULL;//input quaternions file
char *ifile = (char*)"../input/instructionsFile.txt";//NULL;//input instructions file
char *tfile = (char*)"../output/ten_length.csv";//NULL;//tendon length file
char *rfile = (char*)"../output/momentarm.csv";//NULL;//moment arm file
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
	// File name constants for output files
	std::string momentarmOutputfile;
	std::string momentarmOutputfile_suffix = "_momentarm.csv";
	std::string tendondataOutputdile;
	std::string tendondataOutputdile_suffix ="_ten_length.csv";
	//---- Initialization ----//
	
	//activate mujoco
	checkAndActivate();

	std::string msg_usage = "---\n---\n---\nUSAGE: changeAngles -m modelfile -k kinematics_inputfile\noptional additional arguments\n-t tendondataOutputdile \n-r momentarmOutputfile\n";
	std::string msg_usage_alt = "ALT USAGE: changeAngles -i inputfile where inputfile is the instructions file\nsee ../documentationREADME_workflow.txt\n---\n---\n---\n";
	printf("%s\n", msg_usage.c_str());
	printf("%s\n", msg_usage_alt.c_str());


	//initialize GLFW and window
	GLFWwindow* window = glfwInitWindow();
	if (!window)
		return 1;
	
	
	//-- parse input options --//
	parseArgsLong(argc, argv);
	
	//----- LOAD INSTRUCTIONS FILE TO FILL IN EMPTY ARGUMENTS------	
	//------ if the user has left the arguments blank -------------
	// ------ OPEN MODEL FILE -------------------------------------

	//open input file
	FILE* inputFile;
	if (kfile)
		inputFile = fopen(kfile, "r");
	if (!inputFile) {
		printf("Could not open input file %s\n",kfile);
		//return 1;
	}


	std::string model_filename;
	std::string *instructions_list = new std::string[3];
	file2StringList(ifile, instructions_list);
	model_filename = instructions_list[0];
	std::string trials_filename = instructions_list[1];
	int n_trials = numberOfLines(trials_filename);
	std::string *filenames_list = new std::string[n_trials];
	file2StringList(trials_filename, filenames_list);
	if (n_trials == 0)
		n_trials = 1;
	// check number of arguments - if usage is changeAngles -i instructionsfile then do the  following...
	if (argc == 3)
	{
		printf("Using Model %s\n", model_filename.c_str());
		printf("Trials to be processed:  %i\n", n_trials);
		printf("first trial loaded: %s\n", filenames_list[0].c_str());


		loadmodel(window,model_filename.c_str(), 0);

		inputFile = fopen(filenames_list[0].c_str(), "r");
	}
	else
	{
		loadmodel(window,mfile, 0);
	}
	
	//create output files for tendon and moment arm data
	FILE* tendonFile;
	//FILE* tendonFile_default = fopen(tfile, "w");
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




    printf("nbod %i\n", m->nbody);
    printf("nq = %i\n", m->nq);
    printf("nv = %i\n", m->nv);
    printf("nj = %i\n", m->njnt);
    printf("nu = %i\n", m->nu);
    printf("ntend = %i\n", m->ntendon);
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
    // bool alltendonsQ = false;

    if (n_highlights > 0)
    {
    	highlightQ = true;
    }
    if (highlight_list[0] == "allsites")
    {
    	allsitesQ = true;
    }
    // else if (highlight_list[0] == "alltendons")
    // {
    // 	alltendonsQ = true;
    // }
	
	//---- Pre-processing ----//
	
	//determine size of input file
	// int rows, cols;
	// sizeofCSV(&rows, &cols, inputFile);
	// printf("Rows: %i, Cols: %i\n",rows,cols);
	
	
	//allocate array now that we know the size of the input
	//...use model size nq rather than cols
	
	#ifdef _WIN32	//windows is weird about compiling unkown array size
	mjtNum q[22500];//enough for 500 input rows with Kassina model (nq = 45)
	#else
	mjtNum q[22500];
	#endif
	
	//get positions from file
	//CSV2qpos(m,q,rows,cols,inputFile);
	mj_forward(m,d);//just put the model in null pose for testing


	
	
	
	//print header row for output file
	//basically, each tend1 length, moment0,...momentM, ... tendN length, moment...
	int tID,tMmtID, jID;
	int dof, jdof;
	
	const char *axes = "123xyz";
	

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
	int *highlight_id_list=new int[n_highlights];
	for (int i = 0; i < n_highlights; i++) 
	{
		int muscle_highlight_id = mj_name2id(m, mjOBJ_TENDON, highlight_list[i].c_str());
		highlight_id_list[i] = muscle_highlight_id;
		printf("highlighted tendons: %s, id = %i\n", highlight_list[i].c_str(), muscle_highlight_id);
	}
	//find site highlights from highlights file
	int site_highlight_id;
	int *site_id_list=new int[n_highlights];
	for (int i = 0; i < n_highlights; i++) 
	{
		int site_highlight_id = mj_name2id(m, mjOBJ_SITE, highlight_list[i].c_str());
		site_id_list[i] = site_highlight_id;
		printf("highlighted sites: %s, id = %i\n", highlight_list[i].c_str(), site_highlight_id);
	}
	//find geom highlights from highlights file
	int geom_highlight_id;
	int *geom_id_list=new int[n_highlights];
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
	//int trial = 0;//number of trials to load
	bool fwdYes = true;// for testing will halt any advance of the model	
	if (fwdYes)
		mj_forward(m,d);	
	
	for (int trial = 0; trial < n_trials; trial ++) 
	{
		// FOR EVERY TRIAL ....
		// Create files for saving data
		//create output file for tendons
		std::string filename = filenames_list[trial];
		inputFile = fopen(filename.c_str(), "r");
		int filename_length = filename.size();
		std::string filePrefix = filename.substr(0, filename_length - 4);
		std::string tendon_filename = filePrefix + tendondataOutputdile_suffix;
		std::string momentarm_filename = filePrefix + momentarmOutputfile_suffix;
		std::string hopORrun = filePrefix.substr(14,3);
		//replace input with output so it goes to appropriate output folder
		if (hopORrun == "HOP")
		{
			findAndReplaceAll(tendon_filename, "input", "output/jumping/ten_length");
			findAndReplaceAll(momentarm_filename, "input", "output/jumping/momentarm");

		}
		if (hopORrun == "RUN")
		{
			if ( filePrefix.size() > 24 ) // filenames with "fixed" are separated into a folder
			{
				findAndReplaceAll(tendon_filename, "input", "output/walking/Fixed pelvis/ten_length");
				findAndReplaceAll(momentarm_filename, "input", "output/walking/Fixed pelvis/momentarm");	
			}
			else
			{
				findAndReplaceAll(tendon_filename, "input", "output/walking/Mobile pelvis/ten_length");
				findAndReplaceAll(momentarm_filename, "input", "output/walking/Mobile pelvis/momentarm");				
			}

		}
		//
		//findAndReplaceAll(momentarm_filename, "input", "output/momentarm");
		//printf("%s\n", momentarm_filename.c_str());
		FILE* tendonFile_default = fopen(tfile, "w");
		FILE* momentarmFile_default = fopen(rfile, "w");
		tendonFile = fopen(tendon_filename.c_str(), "w");	
		if (!tendonFile) 
		{
			if (!tfile)
			{
				printf("Could not create tendon file %s\n",tfile);
				return 1;
			}
			else
			{
				tendonFile = fopen(tfile, "w");
			}
		}
		momentarmFile = fopen(momentarm_filename.c_str(), "w");	
		if (!momentarmFile) 
		{
			if (!rfile)
			{
				printf("Could not create moment arm file %s\n", rfile);
				return 1;
			}
			else
			{
				momentarmFile = fopen(rfile, "w");
			}
		}
		/// -----------------------------------------------------
		/// -------------CREATE HEADER ROW FOR TENDON FILE-------
		/// -----------------------------------------------------	
		for(tID = 0; tID < m->ntendon; tID++) 
		{
			
			fprintf(tendonFile,"%s_len,",mj_id2name(m, mjOBJ_TENDON, tID));
			
			for(tMmtID = tID*m->nv; tMmtID < (tID+1)*m->nv; tMmtID++) 
			{
				
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
		/// -----------------------------------------------------
		/// -----------------------------------------------------
		/// -----------------------------------------------------


		// Open trial kinematics file and initialize model
		//printf("%i\n", trial);
		int rows, cols;
		sizeofCSV(&rows, &cols, inputFile);
		//rows = numberOfLines(filenames_list[trial]);
		//printf("%i %i\n", rows, cols);
		CSV2qpos_mapped(m,q,rows,cols,inputFile, "../input/mapdata_k.txt", "../input/mapdata_m.txt");//flag_1
		mju_copy(d->qpos,&q[0*m->nq],m->nq);
		mj_forward(m, d);
		printf("loading trial %s\n", filenames_list[trial].c_str());
		r = 0;

		mjtNum *tendon_len_data=new mjtNum[rows];
		mjtNum *momentarm_data =new mjtNum[rows];

		// main loop
		while( !glfwWindowShouldClose(window) && r < rows - 1) 
		{
					// printf("%i\n", trial);
					// inputFile = fopen(filenames_list[trial].c_str(), "r");
					// sizeofCSV(&rows, &cols, inputFile);
					// rows = numberOfLines(filenames_list[trial]);
					// printf("%i %i\n", rows, cols);
					// CSV2qpos_mapped(m,q,rows,cols,inputFile, "../input/mapdata_k.txt", "../input/mapdata_m.txt");//flag_1
					// mju_copy(d->qpos,&q[0*m->nq],m->nq);
					// printf("trial %s\n", filenames_list[trial].c_str());
			if(!paused) 
			{
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
					// if (!alltendonsQ)
					// {
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
					// }
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
								m->geom_rgba[ geom_id_list[i] * 4 + j] = 0.3;
							}
						}
					}


				}
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

				
				int tendon_id = mj_name2id(m, mjOBJ_TENDON, "t_left_CR_vent");
				
				mju_copy(d->qpos,&q[r*m->nq],m->nq);
				//mju_copy(d->qpos,&q[0*m->nq],m->nq);// for testing

				// ----------
				// ----------
				// FOR TESTING: isolate a joint - only have that one joint move and others are fixed at a frame
				// ----------
				// ----------

				//isolateJoint(m, d, q, "j_kneeL", r, 0);
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


				// int w_knee_id = mj_name2id(m, mjOBJ_GEOM, "g_ctr_kneL");// wrapping cylinder of knee
				// int knee_id = mj_name2id(m, mjOBJ_JOINT, "j_kneeL");// wrapping cylinder of knee
				// for (int i = 0; i < 3; i ++)
				// {
				// 	printf("%f, %f\n", d->geom_xpos[3 * w_knee_id + i], d->xanchor[3 * knee_id + i]);
				// }


				//// DO when you get the chance:
				//// move wrapping cylinder (w_left_knee_side) with rolling joint
				/// need xpos coordinates to assign location of cylinder
				/// confirm if the xpos of the tibfib is where that needs to be - move tibfib slightly distal and put marker there to see

				// int knee_id = mj_name2id(m, mjOBJ_JOINT, "j_kneeL");
				// for (int i = 0; i < 3; i ++)
				// {
				// 	printf("%f\n", d->xaxis[3 * knee_id + i]);
				// }
				// printf("\n");



				//-----------------
				///---------------			
				///----------------
				/////////////// another test section to see if I can trace the border of a wrapping surface
				mjtNum feAxis[3];
				mjtNum xmat_tib[9];
				int tib_id = mj_name2id(m, mjOBJ_BODY, "TibFibL");
				for (int i = 0; i < 9; i ++)
				{
					xmat_tib[i] = d->xmat[9 * tib_id + i];
				}
				xmat2basis(feAxis, xmat_tib, 1);
				

				int knee_id = mj_name2id(m, mjOBJ_JOINT, "j_kneeL");
				mjtNum knee_ctr[3];
				for (int i = 0; i < 3; i ++)
				{
					knee_ctr[i] = d->xanchor[knee_id * 3 + i];
				}
				//mju_printMat(knee_ctr, 1, 3);
				// mjtNum wrap_radius = 0*0.001200;
				// mjtNum wrap_borderXYZ_raw[3];

				int othersite_id = mj_name2id(m, mjOBJ_SITE, "w_left_knee_otherside");
				int knee_wrap_id = mj_name2id(m, mjOBJ_GEOM, "w_left_knee");
				mjtNum wrap_borderXYZ[3];
				mju_scl3(wrap_borderXYZ, feAxis, 0.001250);
			


				if (fwdYes)
					mj_forward(m,d);

				for (int i = 0; i < 3; i ++)
				{
					d->site_xpos[othersite_id * 3 + i] = d->geom_xpos[3 * knee_wrap_id + i] + wrap_borderXYZ[i];//testXYZ[i];
				}	

				//-------------------------
				//-------------------------
				//-------------------------
				// see what tendon is doing
				//-------------------------
				//-------------------------
				//-------------------------

				int knee_ext_id = m->jnt_dofadr[ mj_name2id(m, mjOBJ_JOINT, "j_kneeL") ];
				int hip_ext_id = m->jnt_dofadr[ mj_name2id(m, mjOBJ_JOINT, "j_hipL") ];
				//printf("%i \n", tendon_id * m->nv + knee_ext_id);
				// mjtNum r_CR = d->ten_moment[tendon_id * m->nv + knee_ext_id + 1];//y component is flex-extend
				// mjtNum rh_CR = d->ten_moment[tendon_id * m->nv + hip_ext_id + 1];//y component is flex-extend
				mjtNum r_CR = d->ten_moment[tendon_id * m->nv + knee_ext_id + 1];//x component is lar
				mjtNum rh_CR = d->ten_moment[tendon_id * m->nv + hip_ext_id + 1];//x component is lar
				std::string shortorlength;
				std::string momentarm_glitch;
				mjtNum tendon_len_curr = d->ten_length[tendon_id];
				tendon_len_data[r] = tendon_len_curr;
				momentarm_data[r] = r_CR;

				mjtNum tendon_len_prev = tendon_len_data[r - 1];
				mjtNum momentarm_prev = momentarm_data[r - 1];

				/////////////// another test section to see if I can trace the border of a wrapping surface
				// mjtNum wrap_borderXYZ[3];
				// mju_add3(wrap_borderXYZ, wrap_borderXYZ_raw, kneXYZ);

				// for (int i = 0; i < 3; i ++)
				// {
				// 	d->site_xpos[othersite_id * 3 + i] = wrap_borderXYZ[i];
				// }
				// if (tendon_len_curr < tendon_len_prev)
				// {
				// 	shortorlength = "shortening";
				// }
				// else
				// {
				// 	shortorlength = "lengthening";
				// }
				// if ( abs(r_CR - momentarm_prev) / r_CR > 0.1 )
				// {
				// 	momentarm_glitch = "WARNING!!  moment arm slip";
				// } 
				// else
				// {
				// 	momentarm_glitch = "";
				// }
				// if (r == 0 ){
				// 	printf("test");
				// }
				
				
				//printf("knee length prev %f cur %f mm %s\n", tendon_len_prev, tendon_len_curr, shortorlength.c_str() );


				//on the first loop through, print all tendon information to file

				if(firstTime) 
				{
					//printf("knee moment arm %f mm hip %f mm %s timestep %i \n", 1000*r_CR, 1000*rh_CR, shortorlength.c_str(), r );
					
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

						///  save specific filename
						
						for (int i = 0; i < (m->ntendon);i++) {
							fprintf(tendonFile, "%2.6f", d->ten_length[i]);
							if (i < (m->ntendon-1))
								fprintf(tendonFile, ",");
							else
								fprintf(tendonFile, "\n");
						}
						///  save default filename
						for (int i = 0; i < (m->ntendon);i++) {
							fprintf(tendonFile_default, "%2.6f", d->ten_length[i]);
							if (i < (m->ntendon-1))
								fprintf(tendonFile_default, ",");
							else
								fprintf(tendonFile_default, "\n");
						}
						

					}
					
					
					if (rfile) {
						// save specific filename
						for(int ntendof = 0; ntendof < (m->nv)*(m->ntendon); ntendof++) {
							fprintf(momentarmFile,"%2.6f", d->ten_moment[ntendof]);
							if (ntendof < ((m->nv)*(m->ntendon) -1))
								fprintf(momentarmFile,",");
							else
								fprintf(momentarmFile,"\n");
						}
						// save default filename
						for(int ntendof = 0; ntendof < (m->nv)*(m->ntendon); ntendof++) {
							fprintf(momentarmFile_default,"%2.6f", d->ten_moment[ntendof]);
							if (ntendof < ((m->nv)*(m->ntendon) -1))
								fprintf(momentarmFile_default,",");
							else
								fprintf(momentarmFile_default,"\n");
						}
					}



					
					
					if(ffile) 
					{
					
						fprintf(footFile,"%f,%f,%f,",d->xpos[origin*3],
								d->xpos[origin*3+1],d->xpos[origin*3+2]);
						fprintf(footFile,"%f,%f,%f,",d->site_xpos[contact*3],
								d->site_xpos[contact*3+1],d->site_xpos[contact*3+2]);
						fprintf(footFile,"%f,%f,%f\n",d->site_xpos[contact_mir*3],
								d->site_xpos[contact_mir*3+1],d->site_xpos[contact_mir*3+2]);
						
					}
				}
				
				printf("trial %i / %i, row %i / %i\n", trial, n_trials, r, rows);
				//printf("knee moment arm %f mm hip %f mm %s, timestep %i \n", 1000*r_CR, 1000*rh_CR, shortorlength.c_str(), r);
				r++;

				// //loop around
				// // if (r<rows-1)
				// // 	r++;
				// // else
				// if (r == rows - 1)
				// {
				// 	r=0;
				// 	// Load current trial 
				// 	//sif (trial < n_trials - 1)
				// 	//{
				// 	trial ++;				
				// 	printf("%i\n", trial);
				// 	inputFile = fopen(filenames_list[trial].c_str(), "r");
				// 	//sizeofCSV(&rows, &cols, inputFile);
				// 	rows = numberOfLines(filenames_list[trial]);
				// 	printf("%i %i\n", rows, cols);
				// 	CSV2qpos_mapped(m,q,rows,cols,inputFile, "../input/mapdata_k.txt", "../input/mapdata_m.txt");//flag_1
				// 	mju_copy(d->qpos,&q[0*m->nq],m->nq);
				// 	printf("trial %s\n", filenames_list[trial].c_str());
				// 	//}

				// 	if(firstTime) {
						
				// 		if(tfile) {
				// 			printf("First loop complete - printed tendon lengths to file\n");
				// 			fclose(tendonFile);
				// 		}
						
				// 		if(displayWindow == 0) {
				// 			printf("Done!\n");
				// 			closeAndTerminate();
				// 			return 0;
				// 		}
						
				// 		firstTime = false;
				// 	}
				// }
				if(displayWindow == 0) 
				{
					printf("Done!\n");
					closeAndTerminate();
					return 0;
				}			
				
			} //end if(!paused)
			
			
			
			if( displayWindow )
			{
				// update the window
				render(window);

				// handle events (this calls all callbacks)
				glfwPollEvents();
			}
			
			
			//waitSeconds(0.1);
			
		}//end While
		fclose(tendonFile_default);
		fclose(tendonFile);
		fclose(momentarmFile_default);
		fclose(momentarmFile);
    }//end trial loop
	
     
    
    

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
		{"kinematics",	required_argument,	0, 'k'},
		{0, 0, 0, 0}
	};
	
	/* getopt_long stores the option index here. */
	int option_index = 0;
	int c;
	
	while ((c = getopt_long(argc, (char**)argv, "hf:m:t:i:r:k",
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
				
			case 'k':
				kfile = optarg;
				printf ("Using kinematics file '%s'\n", optarg);
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


