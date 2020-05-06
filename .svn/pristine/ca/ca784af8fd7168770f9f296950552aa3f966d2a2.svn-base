//
//  mjmuscles.h
//  mjpro150
//
//  Created by Enrico Eberhard on 12/01/2018.
//  Copyright © 2018 Enrico Eberhard. All rights reserved.
//



/* USAGE

 Add the following class definition to the model xml:
 
 <default class="muscle">
	<general ctrllimited="true" ctrlrange="0 1" forcelimited="false" dyntype="user" gaintype="user" biastype="none"/>
 </default>
 
 
 Then, for a tendon element with some name [tend_name], add the following general actuator:
 
 <actuator>
	...
 
	<general class="muscle" tendon="[tend_name]" user="[FL FV Hill]"/>
 
	...
 </actuator>
 

 //parameters:	Fmax	-> max isometric muscle force
 //				ML0		-> length of max f(ML)
 //				MVmax	-> max shortening velocity
 //				Ts		-> tendon slack length (length
 
 //ADVANCED
 Add dynprm="ta td" to the general actuator to set activation (ta) and deactivation (td) time constants
 Default (0.01 0.04)
 
 */


#ifndef mjmuscles_h
#define mjmuscles_h

#include "mujocoToolbox.h"
#include "math.h"


#define mjMUSC_DYN_TA		0.01	//default activation time constant (s)
#define mjMUSC_DYN_TD		0.04	//default de-activation time constant (s)
#define mjMUSC_DYN_A_MIN	0		//lower bound of activation signal

mjtNum muscle_act_dyn(const mjModel* m, const mjData* d, int act_id);
mjtNum muscle_act_gain(const mjModel* m, const mjData* d, int act_id);
mjtNum muscle_act_bias(const mjModel* m, const mjData* d, int act_id);


//define a muscle class
//muscle has parameters:
//Lo,Vmax,Ts,Fmax
//muscle has flv curves
//muscle has states:
//Lm, Lt

struct musc_prm {
	mjtNum Lo, Vmax, Ts, Fmax;
	mjtNum h;
	musc_prm(mjtNum Lo, mjtNum Vmax, mjtNum Ts, mjtNum Fmax) :
		Lo(Lo), Vmax(Vmax), Ts(Ts), Fmax(Fmax) {}
};

struct musc_state {
	mjtNum Lm, Vm, Lt, alpha;
};

struct BezP {
	mjtNum t, f;
	BezP(void) : t(0), f(0) {}
	BezP(mjtNum t, mjtNum f) : t(t), f(f) {}
};


BezP operator + (BezP a, BezP b) {
	return BezP(a.t + b.t, a.f + b.f);
}

BezP operator - (BezP a, BezP b) {
	return BezP(a.t - b.t, a.f - b.f);
}

BezP operator * (mjtNum s, BezP a) {
	return BezP(s * a.t, s * a.f);
}


class BezQuintic {
	BezP Points[6];
	mjtNum Tmin, Trange;
public:
	void setPoints(mjtNum *p, mjtNum min, mjtNum max);
	mjtNum getForce(mjtNum t);
};

void BezQuintic::setPoints(mjtNum *p, mjtNum min, mjtNum max) {
	Tmin = min;
	Trange = max - min;
	
	int i;
	for (i = 0; i < 6; i++) {
		Points[i].t = (mjtNum)i/5 * Trange + Tmin;
		Points[i].f = p[i];
	}
}

mjtNum BezQuintic::getForce(mjtNum t) {
	//scale t between 0 and 1 from defined range
	t = (t - Points[0].t)/(Points[5].t - Points[0].t);
	BezP* tmp = Points;
	memcpy(tmp, Points, 6 * sizeof(BezP));
	int i = 6 - 1;
	while (i > 0) {
		for (int k = 0; k < i; k++)
			tmp[k] = tmp[k] + t * ( tmp[k+1] - tmp[k] );
		i--;
	}
	mjtNum f = tmp[0].f;
	delete[] tmp;
	return f;
	
}



class Muscle {
	musc_prm prms;
	musc_state st;
	BezQuintic FL, FLp, FV, FLt; //force curves: muscle length (active, passive), velocity; tendon length
	BezQuintic iFV; //inverse force velocity curve
public:
	mjtNum muscleForce(mjtNum MTL, mjtNum a, mjtNum dt);
};

mjtNum Muscle::muscleForce(mjtNum MTL, mjtNum a, mjtNum dt) {
	
	//use calculated vm from last step to update muscle length
	st.Lm += st.Vm * dt;
	
	
	//update pennation angle from muscle length
	st.alpha = asin(prms.h/st.Lm);
	
	//find tendon length from MTU length
	st.Lt = MTL - st.Lm;
	
	//pre-calculate some values
	mjtNum c_aFL = a * FL.getForce(st.Lm);	//force from active muscle length
	mjtNum c_FLp = FLp.getForce(st.Lm);		//force from passive muscle length
	mjtNum c_FLt = FLt.getForce(st.Lt);		//force from passive tendon length
	
	//solve for muscle velocity
	st.Vm = iFV.getForce((c_FLt/cos(st.alpha) - c_FLp)/c_aFL);
	
	//Enforce some Lmin
	if ((st.Lm < Lmin) && (st.Vm < 0)) {
		st.Vm = 0;
	}
	
	//return tendon force as muscle force equivalent
	return c_FLt;
}


void use_muscle_model(void){
	mjcb_act_dyn = muscle_act_dyn;
	mjcb_act_gain = muscle_act_gain;
	mjcb_act_bias = muscle_act_bias;
}

//-------------------------------- muscle functions ------------------------------------


//control input u in range [0 - 1]
//delayed activaton signal w(u) in range [0 - 1]
//gain a as force/velocity scale in range [0 - 1]
//force x = a*w(u)*maxforce in range [0 - maxforce]


// custom force activation dynamics
mjtNum muscle_act_dyn(const mjModel* m, const mjData* d, int act_id) {
	
	//figure out how to relate id in ctrl dimension (nu) to activation dimension (na)
	// in order to find d->act for d->ctrl
	
	//simple first order model (Millard et al 2013)
	
	mjtNum a,u;
	mjtNum t, ta, td;
	
	a = (d->act[act_id] - mjMUSC_DYN_A_MIN) / (1 - mjMUSC_DYN_A_MIN);
	u = d->ctrl[act_id];
	
	//get time constants from parameters or default
	if (m->actuator_dynprm[act_id * mjNDYN + 1] != 0) {
		ta = m->actuator_dynprm[act_id * mjNDYN];
		td = m->actuator_dynprm[act_id * mjNDYN + 1];
	}
	else {
		ta = mjMUSC_DYN_TA; //10ms activation time constant
		td = mjMUSC_DYN_TD; //40ms deactivation time constant
	}
	
	//calculate first order time constant depending on activation or deactivation
	if (d->ctrl[act_id] > a) { //control signal higher than activation - activate
		t = ta*(0.5 + 1.5 * a);
	}
	else { //control signal same or lower than activation - deactivate
		t = td/(0.5 + 1.5 * a);
	}
	
	//return the derivative of the activation
	return (d->ctrl[act_id] - a)/t;
	
	
}



// custom force activation gain
mjtNum muscle_act_gain_old(const mjModel* m, const mjData* d, int act_id) {
	
	//this can be optimized later to store user params as static const for nuser_actuator
	mjtNum fmax, vmax, hill, v, num, den;
	
	//get some muscle params
	fmax = m->actuator_user[act_id*m->nuser_actuator];
	vmax = m->actuator_user[act_id*m->nuser_actuator + 1];
	hill = m->actuator_user[act_id*m->nuser_actuator + 2];
	
	//get and negate tendon velocity (so that shortening is positive)
	v = -d->actuator_velocity[act_id];
	
	
	if (v == 0) {
		return -fmax;
	}//lengthening
	else if (v < 0) {
		num = 4*v*(hill + 1);
		den = v*(hill + 5) - 4*hill*vmax;
		
		return -fmax*(1 + num/den);
	}//shortening
	else {
		
		//return -fmax;
		
		num = 1 - v/vmax;
		den = v/(hill*vmax) + 1;
		
		return -fmax*(num/den);
	}
	
	return 0;
	
}

// custom force activation gain
mjtNum muscle_act_gain(const mjModel* m, const mjData* d, int act_id) {
	
	//this can be optimized later to store user params as static const for nuser_actuator
	mjtNum fmax, vmax, hill, v, num, den;
	
	//get some muscle params
	fmax = m->actuator_user[act_id*m->nuser_actuator];
	vmax = m->actuator_user[act_id*m->nuser_actuator + 1];
	hill = m->actuator_user[act_id*m->nuser_actuator + 2];
	
	//get and negate tendon velocity (so that shortening is positive)
	v = -d->actuator_velocity[act_id];
	
	
	if (v == 0) {
		return -fmax;
	}//lengthening
	else if (v < 0) {
		num = 4*v*(hill + 1);
		den = v*(hill + 5) - 4*hill*vmax;
		
		return -fmax*(1 + num/den);
	}//shortening
	else {
		
		//return -fmax;
		
		num = 1 - v/vmax;
		den = v/(hill*vmax) + 1;
		
		return -fmax*(num/den);
	}
	
	return 0;
	
}


// custom force activation bias
mjtNum muscle_act_bias(const mjModel* m, const mjData* d, int act_id) {
	
	//return 0, no bias forces
	return 0;
	
}




#endif /* mjmuscles_h */
