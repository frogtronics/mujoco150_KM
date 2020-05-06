//
//  limits.cpp
//  mjpro150
//
//  Created by Enrico Eberhard on 21/06/2017.
//  Copyright © 2017 Enrico Eberhard. All rights reserved.
//
//	This is an interactive demo of a modified Kassina model
//	in which joint limits can be explored.
//
//	Use free bodies as target constraints for body, knee and feet.
//	Make only targets selectable and perturb them in the sim


#include "mj_limits.h"


//-------------------------------- main function ----------------------------------------

int main(int argc, const char** argv)
{
	
	//activate mujoco
	checkAndActivate();
	
	//initialize GLFW and window
	GLFWwindow* window = glfwInitWindow();
	if (!window)
		return 1;
	
	//replace mouse button callback with custom limited one
	//(this will only allow selection of certain geoms)
	glfwSetMouseButtonCallback(window, mouse_button_limited);
	
	glfwSetExtraKeyCallback(limitsExtraKeys);
	
	//initFigure(); //sets up figure
	
	//load model
	loadmodel(window,"../models/Kassina/KassinaLimits.xml", 0);
	
	FILE *qFile, *footFile;
	qFile = fopen("../output/0.txt", "w");
	footFile = fopen("../output/00.txt", "w");
	
	
	FILE* inputFile;
	//inputFile = fopen("../input/hop_toes_full_filt.txt", "r");
	inputFile = fopen("../input/ScanPoseToes.txt", "r");
	
	
	int rows, cols;
	sizeofCSV(&rows, &cols, inputFile);
	//printf("Rows: %i, Cols: %i\n",rows,cols);
	
	
	//allocate arrays now that we know the size of the input
	//...use model size nq/nv rather than cols
	mjtNum qTar[rows*cols];
	
	//get positions from file
	CSV2qpos(m,qTar,rows,cols,inputFile);
	
	
	
	//Target object holds constraint information
	targets = Target(m,d);
	
	//add constraints
	targets.addTarget("Spine_copy");
	targets.addTarget("KneeTargetL");
	targets.addTarget("KneeTargetR");
	targets.addTarget("AnkleTargetL");
	targets.addTarget("AnkleTargetR");
	targets.addTarget("FootTargetL");
	targets.addTarget("FootTargetR");
	
	
	//setSpreadPose(m,d);
	mj_forward(m,d);
	
	//set qpos to the pre-jump stance
	int ret;
	if((ret=getPoseFromFile(m, "../input/ScanPoseToes.txt", d->qpos, 1)))
		return ret;
	mj_forward(m,d);
	//
	
	//set target qpos to match qpos of bodies
	targets.matchQ("Spine_copy", "Spine");
	targets.matchQ("KneeTargetL", "TibFibL");
	targets.matchQ("KneeTargetR", "TibFibR");
	targets.matchQ("AnkleTargetL", "TarsalsL");
	targets.matchQ("AnkleTargetR", "TarsalsR");
	targets.matchQ("FootTargetL", "Toe2L");
	targets.matchQ("FootTargetR", "Toe2R");
	//
	
	/*/
	mjtNum qAnkleTargetL[7] = {-0.01780609, 0.00870646, 0.00945715, -0.18964318, 0.17804728, -0.89664780, -0.35826994};
	mjtNum qAnkleTargetR[7] = {-0.01867448, -0.00620155, 0.00924743, -0.35606412, -0.88972062, 0.23252736, -0.16597166};
	mjtNum qFootTargetL[7] = {-0.00837618, 0.01933958, 0.00066497, 0.68805694, -0.16241746, 0.68841673, 0.16211300};
	mjtNum qFootTargetR[7] = {-0.01052829, -0.01760674, 0.00014522, -0.19216292, -0.68001669, 0.21181683, -0.67511803};
	
	mju_copy(&d->qpos[m->jnt_qposadr[m->body_jntadr[mj_name2id(m, mjOBJ_BODY, "AnkleTargetL")]]], qAnkleTargetL, 7);
	mju_copy(&d->qpos[m->jnt_qposadr[m->body_jntadr[mj_name2id(m, mjOBJ_BODY, "AnkleTargetR")]]], qAnkleTargetR, 7);
	mju_copy(&d->qpos[m->jnt_qposadr[m->body_jntadr[mj_name2id(m, mjOBJ_BODY, "FootTargetL")]]], qFootTargetL, 7);
	mju_copy(&d->qpos[m->jnt_qposadr[m->body_jntadr[mj_name2id(m, mjOBJ_BODY, "FootTargetR")]]], qFootTargetR, 7);
	/*/
	
	mj_forward(m,d);
	
	//mju_printMat(&d->qpos[m->jnt_qposadr[m->body_jntadr[mj_name2id(m, mjOBJ_BODY, "AnkleTargetR")]]], 1, 7);
	int jBodyTar = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "j_body_copy")];

	mjtNum dotHipQ;
	int hipID = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "j_hipL")];
	mjtNum hipQ[4], hipQ_[4] = {1, 0, 0, 0};
	
	mjtNum HKA[12];
	
	int spineID = mj_name2id(m, mjOBJ_BODY, "Spine");
	int footID = mj_name2id(m, mjOBJ_SITE, "s_ToeL_tip");
	mjtNum spineX[7], footX[3], xdiff[3];
	mjtNum norm, norm_ = 0;
	
	
	//toggle off sites, tendons and constraints
	//limitsExtraKeys(GLFW_KEY_M);
	limitsExtraKeys(GLFW_KEY_PERIOD, 0);
	
	int settleFrames = 100;
	int fr = -settleFrames;
	
	mjtNum timer = 0;
	
	bool firstTime = false;
	
	
	paused = true;
	while(!glfwWindowShouldClose(window)) {
		
		if(!paused) {
			
			//cam.azimuth = d->time*15;
			
			// slow motion factor: 10x
			mjtNum factor = (slowmotion ? 10 : 1);
			
			// advance effective simulation time by 1/refreshrate
			mjtNum startsimtm = d->time;
			while((d->time-startsimtm)*factor < 1.0/refreshrate) {
				
				//apply perturbations
				mju_zero(d->xfrc_applied, 6*m->nbody);
				if( pert.select>0 )
					mjv_applyPerturbForce(m, d, &pert);
				
				
				//don't simulate targets; just perturb pose
				if (targets.checkTarget(pert.select))
					mjv_applyPerturbPose(m, d, &pert, 1);
				
				
				/*/
				if (d->time - timer >= 0.04) {
					timer = d->time;
					fr++;
				
					if ((fr > 0) && (fr < rows)) {
				
					//set qpos of body target to match jump kinematics
					// and keep toe qpos stationary.
					mju_copy(&d->qpos[jBodyTar], &qTar[fr * cols], 7);
						
					}
					else if (fr > 100) {
						if(getPoseFromFile(m, "../input/hop_toes_full_filt.txt", d->qpos, 1))
							return -1;
						mju_copy(&d->qpos[jBodyTar], qTar, 7);
						mju_zero(d->qvel,m->nv);
						fr = -settleFrames;
						
						firstTime = false;
					}
				}
				
				/*/
				
				
				//save qpos of target bodies
				targets.saveQ();
				
				mj_step(m, d);
				
				//reset qpos and qvel of target bodies
				targets.setQ();
				
				
				
				//print out information if it's sufficiently different from previous
				
				//hip range of motion:
				//get qpos of hip
				//compare to previous qpos:
				//	dot q0 and q1
				//	if less than 0.99, new orientation
				//		print line in file
				/*/		save qpos for next time
				
				mju_copy(hipQ, &d->qpos[hipID], 4);
				dotHipQ = mju_dot(hipQ, hipQ_, 4);
				
				if (dotHipQ < 0.999) {
					fprintf(qFile,"%1.2f, %1.2f, %1.2f, %1.2f\n",
						   hipQ[0],hipQ[1],hipQ[2],hipQ[3]);
					mju_copy(hipQ_, hipQ, 4);
				}
				/*/
				
				if (firstTime) {
					mju_copy(HKA, &d->qpos[hipID], 12);
					fprintf(qFile,"%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f\n",
							HKA[0],HKA[1],HKA[2],HKA[3],
							HKA[4],HKA[5],HKA[6],HKA[7],
							HKA[8],HKA[9],HKA[10],HKA[11],d->time);
					/*/
					
					//foot range:
					//get xpos and xmat of spine body
					//get xpos of foot site
					//	norm(spine - foot), compare to previous norm
					//	if more than 0.001
					//		print xpos, xmat, xpos in file
					//		save norm for next time

					mju_copy(spineX, &d->xpos[spineID*3], 3);
					mju_copy(&spineX[3], &d->xquat[spineID*4], 4);
					mju_copy(footX, &d->site_xpos[footID*3], 3);
					
					fprintf(footFile, "%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f\n",
							spineX[0], spineX[1], spineX[2],
							spineX[3], spineX[4],spineX[5], spineX[6],
							footX[0], footX[1], footX[2]);
					
					/*/
					mju_sub3(xdiff, footX, spineX);
					norm = mju_norm3(xdiff);

					if (fabs(norm - norm_) > 0.001) {
						
						//printf("%1.2f\n",norm*1000);
						//fprintf(footFile,"%1.2f, %1.2f, %1.2f, %1.2f\n",
						//		hipQ[0],hipQ[1],hipQ[2],hipQ[3]);
						norm_ = norm;
					}
					
						
				}
				 
				
				
				updateFigure();
				
			}
			
		}
		else {
			mjv_applyPerturbPose(m, d, &pert, 1);
			mj_forward(m, d);
		}
		
		render(window);
		
		glfwPollEvents();
		
	}
	
	fclose(qFile);
	closeAndTerminate();

}


void setSpreadPose(const mjModel* m, const mjData* d) {
	
	mju_copy(d->qpos, m->qpos0, m->nq);
	
	mjtNum qHip[4], qKnee[4], qAnk[4];
	mjtNum ax[3] = {0, 1, 0};
	
	
	mju_axisAngle2Quat(qHip, ax, -mjPI/4);
	mju_axisAngle2Quat(qKnee, ax, mjPI/2 - mjPI/36);
	mju_axisAngle2Quat(qAnk, ax, - mjPI/2 + mjPI/36);
	
	
	mjtNum q90X[4] = {0.7071, 0.7071, 0, 0};
	
	//set qpos
	mju_copy(&d->qpos[3], q90X, 4);
	d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "j_urostyleX")]] = 0.3;
	d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "j_ISX")]] = 0.3;
	
	mju_copy(&d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "j_hipL")]], qHip, 4);
	mju_copy(&d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "j_kneeL")]], qKnee, 4);
	mju_copy(&d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "j_ankleL")]], qAnk, 4);
	
	mju_negQuat(qHip, qHip);
	mju_negQuat(qKnee, qKnee);
	mju_negQuat(qAnk, qAnk);
	
	mju_copy(&d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "j_hipL_mir")]], qHip, 4);
	mju_copy(&d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "j_kneeL_mir")]], qKnee, 4);
	mju_copy(&d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "j_ankleL_mir")]], qAnk, 4);

}


