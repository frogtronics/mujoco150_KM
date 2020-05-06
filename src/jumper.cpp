//
//  jumper.cpp
//
//  Created by Enrico Eberhard
//
//	This is a program that simulates the
//	forward dynamics of kassina jumping.
//	It uses hard and soft targets to achieve
//	a jump from an initial stance. The
//	parameters of the controller can be
//	optimized in an external algorithm



#include "mujocoToolbox.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

void jumperExtraKeys(int key, int act);
mjtNum usr_act_gain(const mjModel* m, const mjData* d, int id);
void maintainPoseController(const mjModel* m, const mjData* d);
void maintainPoseController(const mjModel* m, const mjData* d, const char *filename, int fr);

//-------------------------------- global variables -------------------------------------

// model
mjModel* m = 0;
mjData* d = 0;

mjModel* m2 = 0;


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
	loadmodel(window,"../models/Kassina/KassinaFoot.xml", 0);
	
	const char *infilename = "../input/hop_toes_full_filt.txt";
	
	//m->opt.timestep = 0.0005;

	//add custom actuator callback
	mjcb_act_gain = usr_act_gain;
	
	//add extra key callbacks
	glfwSetExtraKeyCallback(jumperExtraKeys);
	
	int ret;
	//set qpos to the pre-jump stance
	if((ret=getPoseFromFile(m, infilename, d->qpos, 25)))
		return ret;
	mj_forward(m,d);
	
	int toeTipID = mj_name2id(m, mjOBJ_SITE, "s_ToeL_tip");
	mjtNum x[3];
	mju_copy(x, &d->site_xpos[toeTipID*3], 3);
	
	printf("Toe (L --- R): %1.4f, %1.4f, %1.4f\t---\t\%1.4f, %1.4f, %1.4f\n",
		   x[0],x[1],x[2]-0.0007, 0., 0., 0.); //subtract toe radius to represent true ground
	
	
	
	
	//get qpos and qvel from file
	//determine size of input file
	
	FILE* inputFile;
	inputFile = fopen(infilename, "r");
	
	int rows, cols;
	sizeofCSV(&rows, &cols, inputFile);
	printf("Rows: %i, Cols: %i\n",rows,cols);
	
	
	//allocate arrays now that we know the size of the input
	//...use model size nq/nv rather than cols
	mjtNum q[rows*m->nq];
	mjtNum v[rows*m->nv];
	
	//get positions from file
	CSV2qpos(m,q,rows,cols,inputFile);
	
	//calculate joint velocities
	qpos2qvel(m, q, v, rows, 0.004);
	
	//mju_printMat(v,3,m->nv);
	
	//
	int dofIS = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "j_ISX")];
	int dofTMTL = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "j_tmtL")];
	int dofHipR = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "j_hipL_mir")];
	int dofTMTR = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "j_tmtR")];
	
	int qIS = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "j_ISX")];
	int qTMTL = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "j_tmtL")];
	int qHipR = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "j_hipL_mir")];
	int qTMTR = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "j_tmtR")];
	//
	
	
	//turn tendons and sites off
	memset(m->tendon_rgba, (float)0, m->ntendon*4*sizeof(float));
	memset(m->site_rgba, (float)0, m->nsite*4*sizeof(float));
	
	printf("jump\n");
	paused = true; slowmotion = true;
	
	int fr = 0;
	
	mjtNum timer = 0;
	mjtNum time;
	
	
	
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
				
				
				time = d->time - timer;
				
				
				//while the time is below a single frame, keep maintaining the pose
				/*/ of that frame
				if (time < 0.004) {
					if ((fr > 0) && (fr < 56))
						maintainPoseController(m, d, infilename, fr);
					else
						mju_zero(d->qfrc_applied,m->nv); //do nothing
				}
				else {
					timer = d->time;
					fr++;
					
					if (fr >= 150) {
						fr = 0;
						if((ret=getPoseFromFile(m, infilename, d->qpos, 1)))
							return ret;
						mju_zero(d->qvel,m->nv);
					}
				}
				/*/
				
				
				//when the time exceeds a single frame, increment the frame and apply
				//that frame's position and velocity
				if (time >= 0.004) {
					timer = d->time;
					fr++;
					
					if (fr <= 0)
						mju_zero(d->qfrc_applied,m->nv); //do nothing
					
					else if ((fr > 0) && (fr < 37)) {
						
						
						mju_copy(d->qpos,&q[fr*m->nq],qIS);
						mju_copy(&d->qpos[qIS],&q[fr*m->nq + qIS],qTMTL - qIS);
						mju_copy(&d->qpos[qHipR],&q[fr*m->nq + qHipR],qTMTR - qHipR);

						mju_copy(d->qvel,&v[fr*m->nv],dofIS);
						mju_copy(&d->qvel[dofIS],&v[fr*m->nv + dofIS],dofTMTL - dofIS);
						mju_copy(&d->qvel[dofHipR],&v[fr*m->nv + dofHipR],dofTMTR - dofHipR);
						
					}
					else if (fr <= 300)
						mju_zero(d->qfrc_applied,m->nv); //do nothing
					else {
						if((ret=getPoseFromFile(m, infilename, d->qpos, 1)))
							return ret;
						mju_zero(d->qvel,m->nv);
						fr = 0;
					}
				}
				//
				
				
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

		// handle events (this calls all callbacks)
		glfwPollEvents();
		
	}
	

    // delete everything we allocated
    closeAndTerminate();
	
    
    return 0;
}


//-------------------------------- control functions ------------------------------------

// Many types of control strategies exist to achieve a jump. Each strategy has its own
// function that takes the current state of the model and calculates joint torques.
// The weighted contribution of multiple controllers gives the final actuation.


//Control aim: Extend legs
// this control simply tries to maximally extend legs (apply torque to move joint to 180 degrees)
//	- position based


//Control aim: Move feet further from body
// this is similar to extension, but instead of aiming for a final extended pose, it aims for
// whatever instantaneous direction will increase foot distance
//	- Jacobian based (what joint velocity gives desired foot cartesian velocity direction?)


//Control aim: Keep body pointed at some jump angle
//

//Control aim: Keep legs pointed at some jump angle
//


//Macro controller: treat ground contact as origin, and actuate joints so body has
// orientation and velocity along the jump angle

//



//-------------------------------- utility functions ------------------------------------

// custom force gain dynamics
mjtNum usr_act_gain(const mjModel* m, const mjData* d, int id) {
	
	//return 1, force is equal to control input
	return 1;
	
}

void maintainPoseController(const mjModel* m, const mjData* d, const char *filename="../input/hop_toes_full_filt.txt", int fr=0) {
	
	mjtNum target[200];
	
	
	//in future, optimise this by loading the whole file into a static q[fr*m-nq] array
	//and use that as opposed to reloading the same file every time
	getPoseFromFile(m, filename, target, fr);
	
	
	//find vector between states - depends on type of joint
	//	so do it using existing qpos2qvel function
	mjtNum q[3*m->nq];
	mjtNum v[3*m->nv];
	
	mju_copy(&q[0*m->nq], d->qpos, m->nq);
	mju_copy(&q[1*m->nq], target, m->nq);
	mju_copy(&q[2*m->nq], target, m->nq);
	
	qpos2qvel(m, q, v, 3, 1);
	
	//scale force by inertia of body
	//requires looking up joint IDs to see what body they belong to
	//.. but will be much more stable
	
	//start by scaling just by mass and distance to com
	// (the further away/heavier, the more torque)
	
	int dof,jID,bID;
	
	mjtNum sub_com[3], xanch[3], offset[3];
	mjtNum dist;
	
	//cycle through dofs
	for(dof=0;dof<m->nv;dof++)
	{
		//for each dof, find corresponding joint and body
		jID = m->dof_jntid[dof];
		bID = m->dof_bodyid[dof];
		
		
		//subtract d->subtree_com from d->xanchor, take magnitude
		
		mju_copy(sub_com, &d->subtree_com[bID], 3);
		mju_copy(xanch, &d->xanchor[jID], 3);
		mju_sub3(offset, sub_com, xanch);
		
		dist = mju_norm3(offset);
		
		v[dof] *= 0.02;
		
		//multiply by m->body_subtreemass
		//v[dof] *= 20*(m->body_subtreemass[bID]*dist);
		
		//dampen
		//v[dof] -= 0.01*d->qvel[dof]*m->body_subtreemass[bID]*dist;
		
		if ((v[dof] > 0.1)||(v[dof] < -0.1))
			printf("%1.3f: dof %i qfrc %1.2f\n",d->time,dof,v[dof]);
			
		//limit
		if (v[dof] > 0.1)
			v[dof] = 0.1;
		else if (v[dof] < -0.1)
			v[dof] = -0.1;
	}
	
	
	
	//apply result as force
	// BUT ignore first 6 dofs (free joint)! (and urostyle, and TMT/toes)
	
	int dofIS = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "j_ISX")];
	int dofTMTL = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "j_tmtL")];
	int dofHipR = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "j_hipL_mir")];
	int dofTMTR = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "j_tmtR")];
	
	dofIS = 0;
	mju_copy(&d->qfrc_applied[dofIS],&v[dofIS],dofTMTL - dofIS);
	mju_copy(&d->qfrc_applied[dofHipR],&v[dofHipR],dofTMTR - dofHipR);
	
	//printf("t: %2.4f --- %1.3f, %1.3f, %1.3f\n",d->time,v[3],v[4],v[5]);
	
}


void jumperExtraKeys(int key, int act){
	
	switch(key) {
			
		case GLFW_KEY_BACKSPACE:
			getPoseFromFile(m, "../input/hop_toes_full.txt", d->qpos, 1);
			mju_zero(d->qvel,m->nv);
			break;
			
			
	}
}

