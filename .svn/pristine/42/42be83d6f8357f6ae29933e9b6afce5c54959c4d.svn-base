//
//  limits.h
//  mjpro150
//
//  Created by Enrico Eberhard on 21/06/2017.
//  Copyright © 2017 Enrico Eberhard. All rights reserved.
//

#ifndef mj_limits_h
#define mj_limits_h

#include "mujocoToolbox.h"
#include <stdio.h>


//-------------------------------- Target class ------------------------------------

class Target {
	
#define TAR_NBODY 100
#define TAR_S_LEN 25
#define TAR_NQ 100
#define TAR_DEF_VAL -1
private:
	mjModel* mtar = 0;
	mjData* dtar = 0;
	
	mjtNum qpos[TAR_NQ];
public:
	//default to global model/data
	Target(void): mtar(m), dtar(d) {reset();}
	//or construct with specific model/data
	Target (mjModel * m, mjData *d): mtar(m), dtar(d) {reset();}
	
	char tarNames[TAR_NBODY][TAR_S_LEN];
	int tarIDs[TAR_NBODY];
	int numIDs = 0;
	
	int addTarget(const char *);
	int checkTarget(int);
	int checkTarget(const char *);
	int rmvTarget(const char *);
	
	void saveQ(void);
	void setQ(void);
	
	void matchQ(const char*, const char*);
	void setQ0(const char*, const char*);
	
	void reset(void);
};



//initializes ids to -1
void Target::reset(void) {
	for (int ii=0; ii<TAR_NBODY; ii++) {
		tarIDs[ii] = TAR_DEF_VAL;
		strcpy(tarNames[ii], "");
	}
	numIDs = 0;
}


//given a body name, adds the body ID to array
int Target::addTarget(const char *tarName) {
	
	//printf("Adding target %s\n", tarName);
	int tarID = mj_name2id(mtar, mjOBJ_BODY, tarName);
	
	if (tarID != -1) {
		tarIDs[numIDs++] = tarID;
		strcpy(tarNames[numIDs], tarName);
	}
	
	return numIDs;
}


//check by body id if a target is defined
int Target::checkTarget(int tarID) {
	
	for (int ii=0; ii<numIDs; ii++) {
		if (tarID == tarIDs[ii])
			return 1;
	}
	
	return 0;
}

//check by name if a target is defined
int Target::checkTarget(const char * tarName) {
	
	int tarID = mj_name2id(mtar, mjOBJ_BODY, tarName);
	
	if (tarID != -1)
		return checkTarget(tarID);
	
	return 0;
}


//remove a named target from the id array
int Target::rmvTarget(const char *tarName) {
	
	int tarID = mj_name2id(mtar, mjOBJ_BODY, tarName);
	
	if (tarID != -1) {
		
		bool shift = false;
		
		for (int ii=0; ii<numIDs; ii++) {
			
			if ((tarID == tarIDs[ii]) || shift) {
				
				if (!shift) {
					shift = true;
					numIDs --;
				}
				
				if (ii < numIDs-1)
					tarIDs[ii] = tarIDs[ii+1];
				else
					tarIDs[ii] = TAR_DEF_VAL;
			}
		}
	}
	
	return numIDs;
}



void Target::saveQ(void) {
	int ii, adr;
	
	for (ii=0; ii < numIDs; ii++) {
		adr = mtar->jnt_qposadr[mtar->body_jntadr[tarIDs[ii]]];
		mju_copy(&qpos[adr], &dtar->qpos[adr], 7);
	}
}

void Target::setQ(void) {
	
	int ii, adr;
	
	for (ii=0; ii < numIDs; ii++) {
		adr = mtar->jnt_qposadr[mtar->body_jntadr[tarIDs[ii]]];
		mju_copy(&dtar->qpos[adr], &qpos[adr], 7);
		mju_zero(&dtar->qvel[mtar->body_dofadr[tarIDs[ii]]], 6);
	}
}

//sets the qpos of the target to match the xpos and xquat of the body
void Target::matchQ(const char* target, const char* body) {
	
	mju_copy(&dtar->qpos[mtar->jnt_qposadr[mtar->body_jntadr[mj_name2id(mtar, mjOBJ_BODY, target)]]],
			 &dtar->xpos[mj_name2id(mtar, mjOBJ_BODY, body)*3], 3);
	mju_copy(&dtar->qpos[mtar->jnt_qposadr[mtar->body_jntadr[mj_name2id(mtar, mjOBJ_BODY, target)]]+3],
			 &dtar->xquat[mj_name2id(mtar, mjOBJ_BODY, body)*4], 4);
	
}

//weld constraints are defined by body offset in qpos0
//matchQ0 : set d->qpos to m->qpos0, mjforward, matchQ except with q0, reset to d->qpos
//sets the qpos0 of the target to match the xpos0 and xquat0 of the body
void Target::setQ0(const char* target, const char* body) {
	
	mjtNum q[mtar->nq];
	
	//save current dtar->qpos
	mju_copy(q, dtar->qpos, mtar->nq);
	
	//set model to 0 pose
	mju_copy(dtar->qpos, mtar->qpos0, mtar->nq);
	//forward kinematics to find xpos and xquat
	mj_forward(mtar, dtar);
	
	//copy specified body 0 pose into target 0 pose
	mju_copy(&mtar->qpos0[mtar->jnt_qposadr[mtar->body_jntadr[mj_name2id(mtar, mjOBJ_BODY, target)]]],
			 &dtar->xpos[mj_name2id(mtar, mjOBJ_BODY, body)*3], 3);
	mju_copy(&mtar->qpos0[mtar->jnt_qposadr[mtar->body_jntadr[mj_name2id(mtar, mjOBJ_BODY, target)]]+3],
			 &dtar->xquat[mj_name2id(mtar, mjOBJ_BODY, body)*4], 4);
	
	//reload saved dtar->qpos
	mju_copy(dtar->qpos, q, mtar->nq);
	mj_forward(mtar, dtar);
	
}


//-------------------------------- global variables -------------------------------------

// model
mjModel* m = 0;
mjData* d = 0;

Target targets;

#define N_FIG_LINES 1
mjvFigure figure;


//-------------------------------- utility functions ------------------------------------

void renderFigure(GLFWwindow* window);

// init profiler
void initFigure(void)
{
	//set figure to default params
	mjv_defaultFigure(&figure);
	
	//title
	strcpy(figure.title, "Data");
	
	//x label
	strcpy(figure.xlabel, "Frame");
	
	//y tick number format
	strcpy(figure.yformat, "%.3f");
	
	//figure colour
	float rgba[4] = {0,0,0.2,1};
	memcpy(figure.figurergba, rgba, 4*sizeof(float));
	
	//legend
	strcpy(figure.linename[0], "Body Z");
	//strcpy(figure.linename[1], "Data2"); //etc
	
	//grid sizes (number of grid lines in range)
	figure.gridsize[0] = 5; //(x)
	figure.gridsize[1] = 5; //(y)
	
	//grid range (xmin xmax ymin ymax)
	figure.range[0][0] = -200;
	figure.range[0][1] = 0;
	figure.range[1][0] = 0;
	figure.range[1][1] = 0.05;
	
	int i, n;
	
	// initialize x values for n lines
	for( n=0; n<N_FIG_LINES; n++ )
		for( i=0; i<mjMAXLINEPNT; i++ )
		{
			figure.linedata[n][2*i] = (float)-i;
		}
	
	
	//set the extra render function pointer in mujocoToolbox
	glfwSetExtraRenderFun(renderFigure);
}


void updateFigure(void) {
	
	// get data
	float data[N_FIG_LINES] = {
		(float)d->qpos[2] //z position of body
	};
	
	// update number of visible points
	//	(empty, fills up over 200 frames)
	int n,i, pnt = mjMIN(201, figure.linepnt[0]+1);
	
	//shift linedata and assign new
	for( n=0; n<1; n++ )
	{
		// shift data
		for( i=pnt-1; i>0; i-- )
			figure.linedata[n][2*i+1] = figure.linedata[n][2*i-1];
		
		// assign new
		figure.linepnt[n] = pnt;
		figure.linedata[n][1] = data[n];
	}
	
}

// show profiler
void renderFigure(GLFWwindow* window)
{
	// get current framebuffer rectangle
	mjrRect rect = {0, 0, 0, 0};
	glfwGetFramebufferSize(window, &rect.width, &rect.height);
	
	//scale rectangle to top right corner of screen
	mjrRect viewport = {rect.width - rect.width/5, rect.height - rect.height/4, rect.width/5, rect.height/4};
	
	mjr_figure(viewport, &figure, &con);
	
}


//-------------------------------- callback functions ------------------------------------


// mouse button
void mouse_button_limited(GLFWwindow* window, int button, int act, int mods)
{
	// past data for double-click detection
	static int lastbutton = 0;
	static double lastclicktm = 0;
	
	// update button state
	button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
	button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
	button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);
	
	
	// Alt: swap left and right
	if( (mods & GLFW_MOD_ALT) )
	{
		bool tmp = button_left;
		button_left = button_right;
		button_right = tmp;
		
		if( button==GLFW_MOUSE_BUTTON_LEFT )
			button = GLFW_MOUSE_BUTTON_RIGHT;
		else if( button==GLFW_MOUSE_BUTTON_RIGHT )
			button = GLFW_MOUSE_BUTTON_LEFT;
	}
	
	
	// update mouse position
	glfwGetCursorPos(window, &lastx, &lasty);
	
	// require model
	if( !m )
		return;
	
	// set perturbation
	int newperturb = 0;
	if( act==GLFW_PRESS && (mods & GLFW_MOD_CONTROL) && pert.select>0 )
	{
		// right: translate;  left: rotate
		if( button_right )
			newperturb = mjPERT_TRANSLATE;
		else if( button_left )
			newperturb = mjPERT_ROTATE;
		
		// perturbation onset: reset reference
		if( newperturb && !pert.active )
			mjv_initPerturb(m, d, &scn, &pert);
	}
	pert.active = newperturb;
	
	// detect double-click (250 msec)
	if( act==GLFW_PRESS && glfwGetTime()-lastclicktm<0.25 && button==lastbutton )
	{
		// determine selection mode
		int selmode;
		if( button==GLFW_MOUSE_BUTTON_LEFT )
			selmode = 1;
		else if( mods & GLFW_MOD_CONTROL )
			selmode = 3;
		else
			selmode = 2;
		
		// get current window size
		int width, height;
		glfwGetWindowSize(window, &width, &height);
		
		// find geom and 3D click point, get corresponding body
		mjtNum selpnt[3];
		int selgeom = mjv_select(m, d, &vopt,
								 (mjtNum)width/(mjtNum)height,
								 (mjtNum)lastx/(mjtNum)width,
								 (mjtNum)(height-lasty)/(mjtNum)height,
								 &scn, selpnt);
		int selbody = (selgeom>=0 ? m->geom_bodyid[selgeom] : 0);
		
		// set lookat point, start tracking is requested
		if( selmode==2 || selmode==3 )
		{
			// copy selpnt if geom clicked
			if( selgeom>=0 )
				mju_copy3(cam.lookat, selpnt);
			
			// switch to tracking camera
			if( selmode==3 && selbody )
			{
				cam.type = mjCAMERA_TRACKING;
				cam.trackbodyid = selbody;
				cam.fixedcamid = -1;
			}
		}
		
		// set body selection
		else
		{
			if( selbody )
			{
				// record selection
				pert.select = selbody;
				
				// compute localpos
				mjtNum tmp[3];
				mju_sub3(tmp, selpnt, d->xpos+3*pert.select);
				mju_mulMatTVec(pert.localpos, d->xmat+9*pert.select, tmp, 3, 3);
			}
			else
				pert.select = 0;
		}
		
		// stop perturbation on select
		pert.active = 0;
	}
	
	// save info
	if( act==GLFW_PRESS )
	{
		lastbutton = button;
		lastclicktm = glfwGetTime();
	}
}


void limitsExtraKeys(int key, int act){
	
	int ii;
	
	static float site_rgba[750];
	static int site_on = 2;
	
	static float tend_rgba[250];
	static int tend_on = 2;
	
	static bool eq_on = true;
	
	if (site_on == 2) {
		memcpy(site_rgba, m->site_rgba, m->nsite*4*sizeof(float));
		site_on = 1;
	}
	
	if (tend_on == 2) {
		memcpy(tend_rgba, m->tendon_rgba, m->ntendon*4*sizeof(float));
		tend_on = 1;
	}
	
	switch(key) {
			
		case GLFW_KEY_PERIOD: //toggle constraints (also toggles sites?)
		{
			for(ii=0;ii<targets.numIDs;ii++) {
				//get geom start addr from body ID (assume one geom each)
				
				int gID = m->body_geomadr[targets.tarIDs[ii]];
				
				if (eq_on)
					//set rgba[4] of each geom to 0
					m->geom_rgba[gID*4 + 3] = 0;
				
				else
					m->geom_rgba[gID*4 + 3] = 0.4;
				
			}
			
			eq_on = !eq_on;
			
			/*
			int eqFootL = mj_name2id(m, mjOBJ_EQUALITY, "weld_footL");
			printf("eqFootL: %i\t",m->eq_active[eqFootL]);
			printf("body 1: %s\t",mj_id2name(m, mjOBJ_BODY, m->eq_obj1id[eqFootL]));
			printf("body 2: %s\t",mj_id2name(m, mjOBJ_BODY, m->eq_obj2id[eqFootL]));
			m->eq_active[eqFootL] = !m->eq_active[eqFootL];
			printf("eqFootL now: %i\n",m->eq_active[eqFootL]);
			break;
			 */
			
			
			
		}
			
		case GLFW_KEY_COMMA: //toggle sites
		{
			if (site_on) {
				memset(m->site_rgba, (float)0, m->nsite*4*sizeof(float));
				site_on = 0;
			} else {
				memcpy(m->site_rgba, site_rgba, m->nsite*4*sizeof(float));
				site_on = 1;
			}
			break;
		}
			
		case GLFW_KEY_M: //toggle tendons
		{
			if (tend_on) {
				memset(m->tendon_rgba, (float)0, m->ntendon*4*sizeof(float));
				tend_on = 0;
			} else {
				memcpy(m->tendon_rgba, tend_rgba, m->ntendon*4*sizeof(float));
				tend_on = 1;
			}
			break;
		}
	}
}


void printBodyX(const char * body) {
	
	//print pos and quat of a body
	printf("%s xpos and xquat: \t%1.4f %1.4f %1.4f | %1.4f %1.4f %1.4f %1.4f\n",
	body,
	d->xpos[mj_name2id(m, mjOBJ_BODY, body)*3],
	d->xpos[mj_name2id(m, mjOBJ_BODY, body)*3+1],
	d->xpos[mj_name2id(m, mjOBJ_BODY, body)*3+2],
	d->xquat[mj_name2id(m, mjOBJ_BODY, body)*4],
	d->xquat[mj_name2id(m, mjOBJ_BODY, body)*4+1],
	d->xquat[mj_name2id(m, mjOBJ_BODY, body)*4+2],
	d->xquat[mj_name2id(m, mjOBJ_BODY, body)*4+3]);
}





#endif /* mj_limits_h */
