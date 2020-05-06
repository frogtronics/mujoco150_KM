//
//  twoseg.cpp
//
//  Created by Enrico Eberhard
//
//	This is a program that simulates the
//	forward dynamics of a two-segment jumper.

//	clang -O2 -I../inc -L../_OSX -std=c++11 -lstdc++ -mavx ../src/twoseg.cpp -lmujoco150 -lglfw.3 -o twoseg

#include "mujocoToolbox.h"
#include "mjmuscles.h"
#include "stdio.h"
#include "string.h"
#include "math.h"


void initFigures(void);
void updateFigures(void);
void renderFigures(GLFWwindow* window);

void twoSegExtraKeys(int key, int act);

//-------------------------------- global variables -------------------------------------

// model
mjModel* m = 0;
mjData* d = 0;


mjvFigure ctrlFigure;
mjvFigure tendFigure;
mjvFigure posFigure;


//-------------------------------- main function ----------------------------------------

int main(int argc, const char** argv)
{
	
	//activate mujoco
	checkAndActivate();

	//initialize GLFW and window
	GLFWwindow* window = glfwInitWindow();
	if (!window)
		return 1;

	
	//load model
	loadmodel(window,"../models/two_segment.xml", 0);
	
	//m->opt.timestep = 0.0005;

	//
	glfwSetExtraKeyCallback(twoSegExtraKeys);
	
	//add custom actuator callbacks
	use_muscle_model();
	
	
	//have live graphs that show	1) ctrl, activation, and force
	//								2) length, vel of tendon
	//								3) x/z
	
	initFigures();
	
	
	
	//set initial stance
	d->qpos[1] = -0.1;
	d->qpos[2] = -0.5;
	d->qpos[4] = 1;
	
	
	
	paused = false; slowmotion = true;
	
	//loop - simulate
	while(!glfwWindowShouldClose(window))
	{
		
		if(!paused) {
			
			// slow motion factor: 10x
			mjtNum factor = (slowmotion ? 10 : 1);
			
			// advance effective simulation time by 1/refreshrate
			mjtNum startsimtm = d->time;
			while((d->time-startsimtm)*factor < 1.0/refreshrate) {
				
				//apply perturbations
				mju_zero(d->xfrc_applied, 6*m->nbody);
				if( pert.select>0 )
					mjv_applyPerturbForce(m, d, &pert);
				
				
				
				// run mj_step and count
				mj_step(m, d);
				
				updateFigures();
				
				
			}
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





//-------------------------------- figure functions ------------------------------------

void initFigures(void)
{
	//set each figure to default params
	mjv_defaultFigure(&ctrlFigure);
	mjv_defaultFigure(&tendFigure);
	mjv_defaultFigure(&posFigure);
	
	
	//title
	strcpy(ctrlFigure.title, "Control");
	strcpy(tendFigure.title, "Tendon");
	strcpy(posFigure.title, "Body Trajectory");
	
	//x label
	strcpy(ctrlFigure.xlabel, "Frame");
	strcpy(tendFigure.xlabel, "Frame");
	
	strcpy(posFigure.xlabel, "X pos");
	
	//y tick number format
	strcpy(ctrlFigure.yformat, "%.3f");
	strcpy(tendFigure.yformat, "%.3f");
	
	strcpy(posFigure.yformat, "%.3f");
	
	//figure colour
	float rgba[4] = {0,0,0.2,1};
	memcpy(ctrlFigure.figurergba, rgba, 4*sizeof(float));
	memcpy(tendFigure.figurergba, rgba, 4*sizeof(float));
	
	memcpy(posFigure.figurergba, rgba, 4*sizeof(float));
	
	
	//legend
	strcpy(ctrlFigure.linename[0], "Input");
	strcpy(ctrlFigure.linename[1], "Activation");
	strcpy(ctrlFigure.linename[2], "Force/MaxForce"); //etc
	
	//legend
	strcpy(tendFigure.linename[0], "Length");
	strcpy(tendFigure.linename[1], "Velocity");
	
	//legend
	strcpy(posFigure.linename[0], "Body");
	
	
	//grid sizes (number of grid lines in range)
	ctrlFigure.gridsize[0] = 5; //(x)
	ctrlFigure.gridsize[1] = 5; //(y)
	
	//grid sizes (number of grid lines in range)
	tendFigure.gridsize[0] = 5; //(x)
	tendFigure.gridsize[1] = 5; //(y)
	
	//grid sizes (number of grid lines in range)
	posFigure.gridsize[0] = 5; //(x)
	posFigure.gridsize[1] = 5; //(y)
	
	
	//grid range (xmin xmax ymin ymax)
	ctrlFigure.range[0][0] = -200;
	ctrlFigure.range[0][1] = 0;
	ctrlFigure.range[1][0] = 0;
	ctrlFigure.range[1][1] = 1;
	
	//grid range (xmin xmax ymin ymax)
	tendFigure.range[0][0] = -200;
	tendFigure.range[0][1] = 0;
	tendFigure.range[1][0] = -1;
	tendFigure.range[1][1] = 1;
	
	//grid range (xmin xmax ymin ymax)
	posFigure.range[0][0] = -2;
	posFigure.range[0][1] = 2;
	posFigure.range[1][0] = -2;
	posFigure.range[1][1] = 2;
	
	
	int i, n;
	
	// initialize x values for n lines
	for( n=0; n<3; n++ )
	for( i=0; i<mjMAXLINEPNT; i++ )
	{
		ctrlFigure.linedata[n][2*i] = (float)-i;
	}
	
	// initialize x values for n lines
	for( n=0; n<2; n++ )
		for( i=0; i<mjMAXLINEPNT; i++ )
		{
			tendFigure.linedata[n][2*i] = (float)-i;
		}
	
	// initialize x/y values for n lines
	for( n=0; n<3; n++ ) {
		posFigure.linepnt[n] = 0;
		for( i=0; i<mjMAXLINEPNT; i++ )
		{
			posFigure.linedata[n][2*i] = 0; //x
			posFigure.linedata[n][2*i + 1] = 0; //y
		}
	}
	
	
	//set the extra render function pointer in mujocoToolbox
	glfwSetExtraRenderFun(renderFigures);
}


void updateFigures(void) {
	
	//Control Figure
	int m_id = 0;
	// get data
	mjtNum ctrlData[3];
	
	ctrlData[0] = d->ctrl[m_id];
	ctrlData[1] = d->act[m_id];
	ctrlData[2] = - d->actuator_force[m_id] / m->actuator_user[m_id*m->nuser_actuator];
	
	
	// update number of visible points
	//	(empty, fills up over 200 frames)
	int n,i, pnt;
 
	
	pnt = mjMIN(201, ctrlFigure.linepnt[0]+1);
	
	//shift linedata and assign new
	for(n=0; n<3; n++)
	{
		// shift data
		for(i=pnt-1; i>0; i--)
			ctrlFigure.linedata[n][2*i+1] = ctrlFigure.linedata[n][2*i-1];
		
		// assign new
		ctrlFigure.linepnt[n] = pnt;
		ctrlFigure.linedata[n][1] = ctrlData[n];
	}
	
	
	mjtNum tendData[2];
	tendData[0] = d->actuator_length[0];
	tendData[1] = d->actuator_velocity[0];
	
	pnt = mjMIN(201, tendFigure.linepnt[0]+1);
	
	//shift linedata and assign new
	for(n=0; n<2; n++)
	{
		// shift data
		for(i=pnt-1; i>0; i--)
		tendFigure.linedata[n][2*i+1] = tendFigure.linedata[n][2*i-1];
		
		// assign new
		tendFigure.linepnt[n] = pnt;
		tendFigure.linedata[n][1] = tendData[n];
	}
	
	
	
	mjtNum posData[6];
	posData[0] = d->xpos[mj_name2id(m, mjOBJ_BODY, "seg1")*3];		//x
	posData[1] = d->xpos[mj_name2id(m, mjOBJ_BODY, "seg1")*3 + 2];	//z
	
	posData[2] = d->xpos[mj_name2id(m, mjOBJ_BODY, "seg2")*3];		//x
	posData[3] = d->xpos[mj_name2id(m, mjOBJ_BODY, "seg2")*3 + 2];	//z
	
	posData[4] = d->subtree_com[0]; //com x
	posData[5] = d->subtree_com[2]; //com z
	
	pnt = mjMIN(201, posFigure.linepnt[0]+1);
	
	//shift linedata and assign new
	for(n=0; n<3; n++)
	{
		// shift data
		for(i=pnt-1; i>0; i--) {
			posFigure.linedata[n][2*i+1] = posFigure.linedata[n][2*i-1]; //shift x
			posFigure.linedata[n][2*i] = posFigure.linedata[n][2*i-2];  //shift y
		}
		
		// assign new
		posFigure.linepnt[n] = pnt;
		posFigure.linedata[n][0] = posData[n*2 + 0];
		posFigure.linedata[n][1] = posData[n*2 + 1];
	}
	
}


// overlay figures in window
void renderFigures(GLFWwindow* window)
{
	// get current framebuffer rectangle
	mjrRect rect = {0, 0, 0, 0};
	glfwGetFramebufferSize(window, &rect.width, &rect.height);
	
	//scale rectangle to top right corner of screen
	mjrRect viewport = {rect.width - rect.width/5, rect.height - rect.height/4, rect.width/5, rect.height/4};
	
	mjr_figure(viewport, &ctrlFigure, &con);
	
	viewport.bottom -= rect.height/4;
	mjr_figure(viewport, &tendFigure, &con);
	
	viewport.bottom -= rect.height/4;
	mjr_figure(viewport, &posFigure, &con);
}


//-------------------------------- utility functions ------------------------------------
void twoSegExtraKeys(int key, int act){
	
	switch(key) {
		
		case GLFW_KEY_PERIOD:
		// do not act on release
			if( act==GLFW_RELEASE ) {
				d->ctrl[0] = 0;
				//d->ctrl[2] = 0;
			}
			else {
				d->ctrl[0] = 1;
				//d->ctrl[2] = 1;
			}
		break;
		
		case GLFW_KEY_COMMA:
			if( act==GLFW_RELEASE )
				d->ctrl[1] = 0;
			else
				d->ctrl[1] = 1;
		break;
		
	}
	
}
