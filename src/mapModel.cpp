//
//  mapModel.cpp
//
//  Created by Chris Richards, based on analyzeModel.cpp by Enrico Eberhard


//  This code opens an xml or binary model and
//  saves a "map" file which is all of the qpos addreses and their indices
// for convenience, map is saved both as CSV and as a multi-line file


#include "mujocoToolbox.h"
#include "unistd.h"
#include "getopt.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include <string>



int parseArgsLong(int argc, const char **argv);

//-------------------------------- global variables -------------------------------------

// model
mjModel* m = 0;
mjData* d = 0;

int constraints = 0;

char *dfile = NULL;
char *ifile = NULL;
//char *mfile = (char*)"../models/Kassina/Kassina - basu.xml";
char *mfile = NULL;
char *ofile = NULL;//output file





//-------------------------------- main function ----------------------------------------

int main(int argc, const char** argv)
{
	printf("USAGE: changeAngles -m modelfile -o outputfile");
    //activate mujoco
    checkAndActivate();
	
	//initialize GLFW and window
	GLFWwindow* window = glfwInitWindow();
	if (!window)
		return 1;
	
	
	//-- parse input options --//
	parseArgsLong(argc, argv);
	

	// get input filename and save it as two files, one as multi line format and another qs csv
	// for example, if filename is "mapfile.txt" outputs will be mapfile.txt and mapfile_csv
	std::string filestring = tail(ofile, 200).c_str();//sloppy way to get filename as string
	int file_len = filestring.size();
	std::string filestring_multi = filestring.substr(0, file_len - 4) + ".txt";
	std::string filestring_csv = filestring.substr(0, file_len - 4) + ".csv";
	printf("%i %s\n", file_len, filestring_csv.c_str());
	//create output file for model map

	FILE* outputFile;
	FILE* outputFile_csv;
	if (ofile) {
		outputFile = fopen(ofile, "w");
		outputFile_csv = fopen(filestring_csv.c_str(), "w");
		if (!outputFile || !outputFile_csv) {
			//printf("Could not open input file %s\n",ifile);
			printf("USAGE: changeAngles -m modelfile -o outputfile\n");
			return 1;
		}
	}
	
    // load model file
	loadmodel(window,mfile, 0);
	
	
	
	printf("-----------------\n");
	
	printf("Total degrees of freedom: %i\n",m->nv);
	printf("Total number of inputs for joints: %i\n",m->nq);
	printf("Total number of controls for joints: %i\n",m->nu);
	

    const char *JointTypes[] = { "mjJNT_FREE", "mjJNT_BALL", "mjJNT_SLIDE", "mjJNT_HINGE" };
	const char *TrnTypes[] = {"mjTRN_JOINT", "mjTRN_JOINTINPARENT", "mjTRN_SLIDERCRANK", "mjTRN_TENDON", "mjTRN_SITE"};
	
    printf("\nJoint information\n(ID)\tqAdr\tDoF\tTYPE\t\tNAME\n");
    int joint_sizes[4] = {7, 4, 1, 1};
    for(int jointID=0;jointID<m->njnt;jointID++)
    {
		printf("(%i)\t%i\t%i\t%s\t%s\n",jointID,
			   m->jnt_qposadr[jointID],
			   m->jnt_dofadr[jointID],
			   JointTypes[m->jnt_type[jointID]],
			   mj_id2name(m, mjOBJ_JOINT, jointID));

		int joint_size = joint_sizes[m->jnt_type[jointID]];
		//printf("%s\n", tail(JointTypes[m->jnt_type[jointID]], 4).c_str());
		for (int i = 0; i < joint_size; i ++)
		{
			if (jointID == m->njnt - 1 && i == joint_size - 1)
			{
				//write to multi line
				fprintf(outputFile,"%s", mj_id2name(m, mjOBJ_JOINT, jointID));
				fprintf(outputFile,"_%s", std::to_string(i).c_str());

				//write to csv
				fprintf(outputFile_csv,"%s", mj_id2name(m, mjOBJ_JOINT, jointID));
				fprintf(outputFile_csv,"_%s", std::to_string(i).c_str());
			}
			else 
			{
				//write to multi line
				fprintf(outputFile,"%s", mj_id2name(m, mjOBJ_JOINT, jointID));
				fprintf(outputFile,"_%s", std::to_string(i).c_str());
				fprintf(outputFile,"\n");

				//write to csv
				fprintf(outputFile_csv,"%s", mj_id2name(m, mjOBJ_JOINT, jointID));
				fprintf(outputFile_csv,"_%s", std::to_string(i).c_str());
				fprintf(outputFile_csv,",");
			}
		}
    }
	printf("\n");




	
	//std::to_string(3.1415926).c_str()
	
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
	
	while ((c = getopt_long(argc, (char**)argv, "o:i:m:",
				long_options, &option_index)) != -1) {
		
	
		switch (c) {
			case 'c':
				printf ("Showing constraint info\n");
				constraints = 1;
				break;
			case 'o':
				printf ("Saving data to '%s'\n", optarg);
				ofile = optarg;
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




