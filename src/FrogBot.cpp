//
//  FrogBot.cpp
//  mjpro150
//
//  Created by Enrico Eberhard on 23/01/2018.
//  Copyright © 2018 Enrico Eberhard. All rights reserved.
//

#include "mujocoToolbox.h"
#include <stdio.h>
#include <unistd.h>

void dynamics(mjtNum* u, const mjtNum* q, const mjtNum* a);
void controller(mjModel* m, mjData* d, mjtNum* x, mjtNum* v);
void setpoint(mjModel* m, mjData* d, mjtNum* x, mjtNum* v);
int inv_kin(mjtNum q[3], const mjtNum x[3]);
void inv_vel(mjtNum w[3], const mjtNum q[3], const mjtNum v[3]);

void initFigures(void);
void updateFigures(void);
void renderFigures(GLFWwindow* window);

mjvFigure ctrlFigure;
mjvFigure posFigure;
mjvFigure errFigure;

// model
mjModel* m = 0;
mjData* d = 0;


mjtNum x[3], v[3];


int main(int argc, char** argv) {
	
	
	GLFWwindow* window = startMuJoCo("../models/FrogBot/FrogBot.xml");
	
	initFigures();
	
	int trackID = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "Tz")];
	
	
	paused = true;
	
	//loop - simulate
	while(!glfwWindowShouldClose(window))
	{
		
		
		if(!paused) {
			// slow motion factor: 10x
			mjtNum factor = (slowmotion ? 10 : 1);
				
			// advance effective simulation time by 1/refreshrate
			mjtNum startsimtm = d->time;
			while((d->time-startsimtm)*factor < 1.0/refreshrate) {
				
				//generate some target x, v
				setpoint(m, d, x, v);
				
				//command motors to target
				controller(m, d, x, v);
				
				
				//apply perturbations
				mju_zero(d->xfrc_applied, 6*m->nbody);
				if( pert.select>0 )
					mjv_applyPerturbForce(m, d, &pert);
				
				//counter target ball force from gravity
				d->qfrc_applied[trackID] = 9.81*0.001;
				
				// run mj_step and count
				mj_step(m, d);
				
			}
		}
		else {
			
			// apply pose perturbations, run mj_forward
			mjv_applyPerturbPose(m, d, &pert, 1);       // move mocap and dynamic bodies
			mj_forward(m, d);
			
		}
		
		
		// update the window
		render(window);
		
		updateFigures();
		
		// handle events (this calls all callbacks)
		glfwPollEvents();
		
	}


// delete everything we allocated
closeAndTerminate();

	return 0;
}




//generate a target position/velocity in task space
void setpoint(mjModel* m, mjData* d, mjtNum* x, mjtNum* v) {
	
	int mode = 2;
	
	
	int section;
	
	switch (mode) {
			
		case 0:
			x[0] = 0.05 - 0.05*mju_cos(1.5*d->time);
			x[1] = 0;
			x[2] = 0.05*mju_sin(1.5*d->time) - 0.05;
			
			v[0] = -0.05*mju_sin(1.5*d->time);
			v[1] = 0;
			v[2] = 0.05*mju_cos(1.5*d->time);
			break;
			
		case 1:
			mju_zero3(v);
			
			section = (int)(d->time) % 4;
			
			switch(section) {
				case 0:
					x[0] = 0;
					x[2] = 0.01;
					break;
				case 1:
					x[0] = 0.02;
					x[2] = 0.01;
					break;
				case 2:
					x[0] = 0.02;
					x[2] = -0.01;
					break;
				case 3:
					x[0] = 0;
					x[2] = -0.01;
					break;
			}
			
			x[1] = 0;
			break;
			
		case 2:
			x[0] = d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "Tx")]];
			x[1] = 0;
			x[2] = d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "Tz")]];
			
			v[0] = d->qvel[m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "Tx")]];
			v[1] = 0;
			v[2] = d->qvel[m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "Tz")]];
			break;
	}
	
	/*/
	 
	 
	 /*/
}









void controller(mjModel* m, mjData* d, mjtNum* x, mjtNum* v) {
	
	static const mjtNum Kp[3] = {1000, 1000, 1000};
	static const mjtNum Kd[3] = {100, 100, 10};
	
	mjtNum tq[3], tw[3];
	
	mju_zero3(tw);
	
	//convert targets to joint space
	if(inv_kin(tq, x) < 0) {
		mju_copy3(tq, d->actuator_length); //if unreachable, stay at current pos
		mju_scl(tq, tq, (mjtNum)1/30, 2);
	}
	else
		inv_vel(tw, tq, v);
	
	
	//find current joint space state
	mjtNum q[3], w[3];
	
	//get current joint state
	mju_copy3(q, d->actuator_length);
	mju_copy3(w, d->actuator_velocity);
	
	//reduce motor 0,1 length/velocity by 30 (gear) to joint space
	mju_scl(q, q, (mjtNum)1/30, 2);
	mju_scl(w, w, (mjtNum)1/30, 2);
	
	//define desired joint acceleration based on targets
	mjtNum a[3];
	for(int dof = 0; dof < 3; dof++) {
		a[dof] = Kp[dof]*(tq[dof] - q[dof]) + Kd[dof]*(tw[dof] - w[dof]);
	}
	
	
	//find the motor torque necessary to achieve desired joint acceleration
	mjtNum u[3];
	dynamics(u, q, a);
	
	//apply it!
	mju_copy3(d->ctrl, u);
	
}




//returns motor torque for desired joint space acceleration
void dynamics(mjtNum* u, const mjtNum* q, const mjtNum* a) {
	static const mjtNum m[5] = {0.1958, 0.3075, 0.1148, 0.0668, 0.0668};
	static const mjtNum c[5] = {0.0737, 0.1888, 0,      0.1250, 0.1250};
	static const mjtNum l[3] = {0.2500, 0.2500, 0.0500};
	static const mjtNum I[5] = {0.0067, 0.0138, 0.0043, 0.0016, 0.0016};
	static const mjtNum Iee  = 0.00001;
	static const mjtNum mee  = 0.001;
	static const mjtNum cxee  = 0;
	static const mjtNum czee  = 0;
	static const mjtNum g = -9.812;
	
	//pre-calculated constants
	static const mjtNum Kappa = m[1]*l[0]*l[0] + I[0] + I[3] + I[4];
	static const mjtNum Lambda = g*(c[0]*m[0] + m[1]*l[0] + c[3]*m[3] + c[4]*m[4]);
	static const mjtNum Zeta = (m[3] + m[4])*l[2]*l[2] + I[1] + I[2];
	static const mjtNum Eta = g*c[1]*m[1];
	
	/*
	printf("Kappa: %5.5f\n", Kappa);
	printf("Lambda: %5.5f\n", Lambda);
	printf("Zeta: %5.5f\n", Zeta);
	printf("Eta: %5.5f\n", Eta);
	*/
	
	u[0] = a[0]*Kappa + sin(q[0])*Lambda;
	u[1] = a[1]*Zeta + cos(q[1])*Eta;
	u[2] = Iee*(a[1] + a[2]);
	
	/* additional torque from end-effector load mee, Iee */
	if(mee > 0.005) {
		u[0] += l[0]*mee*(a[0]*l[0] + g*sin(q[0]) + a[1]*l[1]*sin(q[0] - q[1]));
		
		u[1] += a[0]*l[0]*l[1]*mee*sin(q[0] - q[1]) + a[1]*(Iee + l[1]*l[1]*mee) + Iee*a[2];
		u[1] += g*mee*(cxee*cos(q[1] + q[2]) + czee*sin(q[1] + q[2]) + l[1]*cos(q[1]));
		
		u[2] += Iee*a[1] + Iee*a[2] + g*mee*(cxee*cos(q[1] + q[2]) + czee*sin(q[1] + q[2]));
	}
	
	//reduce torque by 30 (gear) for first two
	mju_scl(u, u, (mjtNum)1.0/30, 2);
}






int inv_kin(mjtNum q[3], const mjtNum x[3]) {
	
#define LINK_LENGTH 	0.250
#define	R_MIN			0.005
#define R_MAX			0.495
	
	mjtNum L = LINK_LENGTH;
	
	//perform calculations on substitutes
	mjtNum qs[3], xs[3];
	
	mju_copy3(xs, x);
	
	xs[0] += L; xs[2] += L;
	
	mjtNum r = sqrt(xs[0]*xs[0] + xs[2]*xs[2]);
	
	//error checking - is this reachable? (avoid numerical singularities)
	if (r < (mjtNum)R_MIN) {
		/*
		printf("Error: commanded position (%1.1f, %1.1f) unreachable!\n"
			   "Endpoint too close to base (%.0f mm)\n",
			   xs[0] - L, xs[2] - L, R_MIN*1000);
		fflush(stdout);
		 */
		return -1;
	}
	
	if (r > (mjtNum)R_MAX) {
		/*
		printf("Error: commanded position (%1.1f, %1.1f) unreachable!\n"
			   "Endpoint too far from base (%.0f mm)\n",
			   xs[0] - LINK_LENGTH, xs[2] - L, R_MAX*1000);
		fflush(stdout);
		 */
		return -2;
	}
	
	
	mjtNum phi, gamma, lambda;
	
	mjtNum L22 = 2*L*L; //precalcluate 2*L^2
	
	phi = atan2(xs[2],xs[0]); //angle from base to endpoint around y axis
	
	gamma = acos((r*r)/(2*L*r));	//angle remaining from theta to link 1
	lambda = acos(1 - (r*r)/L22);	//angle between link 1 and 2
	
	
	qs[0] = M_PI/2 - gamma - phi;
	
	qs[1] = M_PI/2 + qs[0] - lambda;	//absolute link angle
	//qs[1] = M_PI/2 - lambda;		//relative link angle
	
	//keep in bounds
	if(qs[0] > M_PI)
		qs[0] -= M_PI;
	else if(qs[0] <= -M_PI)
		qs[0] += M_PI;
	
	if(qs[1] > M_PI)
		qs[1] -= M_PI;
	else if(qs[1] <= -M_PI)
		qs[1] += M_PI;
	
	//check limits
	if(qs[0] < -0.3) {
		//printf("M1 close to lower limit!\n");
		//fflush(stdout);
		return -3;
	}
	else if(qs[0] > 1.35) {
		//printf("M1 close to upper limit!\n");
		//fflush(stdout);
		return -4;
	}
	
	if(mju_abs(qs[1] - qs[0]) > 0.85) {
		//printf("lambda %1.2f\n", qs[1] - qs[0]);
		//printf("M2 close to limit!\n");
		//fflush(stdout);
		return -5;
	}
	
	//assign theta only once all bounds have been checked
	qs[2] = xs[1] - qs[1];
	
	mju_copy3(q,qs);
	
	return 0;
}

void fwd_vel(mjtNum q[3], mjtNum w[3], mjtNum v[3]) {

#define LINK_LENGTH 	0.250
	
	mjtNum L = LINK_LENGTH;
	
	v[0] = L*(w[0]*cos(q[0]) - w[1]*sin(q[1]));
	v[2] = L*(-w[0]*sin(q[0]) - w[1]*cos(q[1]));

	v[1] = w[0] + w[1] + w[2];
}

//find joint angular velocities w from end-effector linear velocities v
void inv_vel(mjtNum w[3], const mjtNum q[3], const mjtNum v[3]) {
	
	//input v is in form	{xv, yw, zv} and needs to be in:
	//						{xv, yv, zv, xw, yw, zw}
	
	static const mjtNum l[3] = {0.2500, 0.2500, 0.0500};
	
	mjtNum lv[6];
	mju_zero(lv,6);
	
	lv[0] = v[0];
	lv[2] = v[2];
	lv[4] = v[1];
	
	//End-effector Jacobian
	mjtNum Jee[18] =	{  l[0]*cos(q[0]), -l[1]*sin(q[1]),		0,
									0,           0,				0,
						  -l[0]*sin(q[0]),  l[1]*cos(q[1]),		0,
									0,           0,				0,
									0,           1,				1,
									0,           0,				0};
	
	mju_mulMatTVec(w, Jee, lv, 6, 3);
}


//-------------------------------- figure functions ------------------------------------

void initFigures(void)
{
	//set each figure to default params
	mjv_defaultFigure(&ctrlFigure);
	mjv_defaultFigure(&posFigure);
	mjv_defaultFigure(&errFigure);
	
	
	//title
	strcpy(ctrlFigure.title, "Control (Nm)");
	strcpy(posFigure.title, "Trajectory (mm)");
	strcpy(errFigure.title, "Error (mm)");
	
	//x label
	strcpy(ctrlFigure.xlabel, "Frame");
	strcpy(posFigure.xlabel, "");
	strcpy(errFigure.xlabel, "Frame");
	
	//y tick number format
	strcpy(ctrlFigure.yformat, "%.3f");
	strcpy(posFigure.yformat, "%.3f");
	strcpy(errFigure.yformat, "%.3f");
	
	//figure colour
	float rgba[4] = {0,0,0.2,1};
	memcpy(ctrlFigure.figurergba, rgba, 4*sizeof(float));
	memcpy(posFigure.figurergba, rgba, 4*sizeof(float));
	memcpy(errFigure.figurergba, rgba, 4*sizeof(float));
	
	
	//legend
	strcpy(ctrlFigure.linename[0], "Shoulder");
	strcpy(ctrlFigure.linename[1], "Elbow");
	strcpy(ctrlFigure.linename[2], "Wrist"); //etc
	
	//legend
	strcpy(posFigure.linename[0], "Endpoint");
	strcpy(posFigure.linename[1], "Target");
	
	//legend
	strcpy(errFigure.linename[0], "Target Err");
	
	
	//grid sizes (number of grid lines in range)
	ctrlFigure.gridsize[0] = 5; //(x)
	ctrlFigure.gridsize[1] = 5; //(y)
	
	//grid sizes (number of grid lines in range)
	posFigure.gridsize[0] = 7; //(x)
	posFigure.gridsize[1] = 7; //(y)
	
	//grid sizes (number of grid lines in range)
	errFigure.gridsize[0] = 5; //(x)
	errFigure.gridsize[1] = 5; //(y)
	
	
	//grid range (xmin xmax ymin ymax)
	ctrlFigure.range[0][0] = -mjMAXLINEPNT;
	ctrlFigure.range[0][1] = 0;
	ctrlFigure.range[1][0] = -0.05;
	ctrlFigure.range[1][1] = 0.05;
	
	//grid range (xmin xmax ymin ymax)
	posFigure.range[0][0] = -150;
	posFigure.range[0][1] = 150;
	posFigure.range[1][0] = -150;
	posFigure.range[1][1] = 150;
	
	//grid range (xmin xmax ymin ymax)
	errFigure.range[0][0] = -2;
	errFigure.range[0][1] = 2;
	errFigure.range[1][0] = -5;
	errFigure.range[1][1] = 5;
	
	
	int i, n;
	
	// initialize x values for n lines
	for( n=0; n<3; n++ )
		for( i=0; i<mjMAXLINEPNT; i++ )
		{
			ctrlFigure.linedata[n][2*i] = (float)-i;
		}
	
	// initialize x/y values for n lines
	for( n=0; n<2; n++ ) {
		posFigure.linepnt[n] = 0;
		for( i=0; i<mjMAXLINEPNT; i++ )
		{
			posFigure.linedata[n][2*i] = 0; //x
			posFigure.linedata[n][2*i + 1] = 0; //y
		}
	}
	
	// initialize x values for n lines
	for( n=0; n<1; n++ )
		for( i=0; i<mjMAXLINEPNT; i++ )
		{
			errFigure.linedata[n][2*i] = (float)-i;
		}
	
	
	//set the extra render function pointer in mujocoToolbox
	glfwSetExtraRenderFun(renderFigures);
}


void updateFigures(void) {
	
	//Control Figure
	
	
	// update number of visible points
	//	(empty, fills up over mjMAXLINEPNT frames)
	int n,i, pnt;
 
	
	pnt = mjMIN(mjMAXLINEPNT, ctrlFigure.linepnt[0]+1);
	
	//shift linedata and assign new
	for(n=0; n< m->nu; n++)
	{
		// shift data
		for(i=pnt-1; i>0; i--)
			ctrlFigure.linedata[n][2*i+1] = ctrlFigure.linedata[n][2*i-1];
		
		// assign new
		ctrlFigure.linepnt[n] = pnt;
		ctrlFigure.linedata[n][1] = d->actuator_force[n];
	}
	
	
	mjtNum posData[4];
	//actual
	posData[0] = d->site_xpos[mj_name2id(m, mjOBJ_SITE, "EE")*3] * 1000 - 250;		//x (mm)
	posData[1] = d->site_xpos[mj_name2id(m, mjOBJ_SITE, "EE")*3 + 2] * 1000 - 250;	//z (mm)
	
	//target
	posData[2] = x[0] * 1000;		//x (mm)
	posData[3] = x[2] * 1000;	//z (mm)
	
	pnt = mjMIN(mjMAXLINEPNT, posFigure.linepnt[0]+1);
	
	//shift linedata and assign new
	for(n=0; n<2; n++)
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
	
	mjtNum errData[2];
	
	mju_sub(errData, &posData[0], &posData[2], 2);
	
	mjtNum rad = mju_norm(&posData[2], 2) - mju_norm(&posData[0], 2);
	
	mjtNum phase = mju_atan2(posData[3],posData[2]) - mju_atan2(posData[1],posData[0]);
	
	pnt = mjMIN(mjMAXLINEPNT, errFigure.linepnt[0]+1);
	
	//shift linedata and assign new
	for(n=0; n<1; n++)
	{
		// shift data
		for(i=pnt-1; i>0; i--) {
			errFigure.linedata[n][2*i+1] = errFigure.linedata[n][2*i-1];
			errFigure.linedata[n][2*i] = errFigure.linedata[n][2*i-2];  //shift y
		}
		// assign new
		errFigure.linepnt[n] = pnt;
		errFigure.linedata[n][0] = rad*cos(phase);
		errFigure.linedata[n][1] = rad*sin(phase);
	}
		
	
}


// overlay figures in window
void renderFigures(GLFWwindow* window)
{
	// get current framebuffer rectangle
	mjrRect rect = {0, 0, 0, 0};
	glfwGetFramebufferSize(window, &rect.width, &rect.height);
	
	//scale rectangle to top right corner of screen
	mjrRect viewport = {rect.width - rect.height/3, rect.height - rect.height/3, rect.height/3, rect.height/3};
	
	mjr_figure(viewport, &ctrlFigure, &con);
	
	viewport.bottom -= rect.height/3;
	mjr_figure(viewport, &posFigure, &con);
	
	viewport.bottom -= rect.height/3;
	mjr_figure(viewport, &errFigure, &con);
}

