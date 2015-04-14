// Mars lander simulator
// Version 1.5
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, December 2009

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include <cmath>

void autopilot (void)
// Autopilot to adjust the engine throttle, parachute and attitude control
{
	double altitude, Kh_const = 0.02, Kp_const = 0.5, Pout_const,Offset = 0.5, descent_rate;
	descent_rate = velocity*position.norm();
	altitude = position.abs() - MARS_RADIUS;
	Pout_const = Kp_const*(-(0.5 + Kh_const*altitude + descent_rate));

	stabilized_attitude = true;

	if (Pout_const <= -Offset)
	{
		throttle = 0;
	}
	else if (Pout_const < (1 - Offset))
	{
		throttle = Offset + Pout_const;
	}
	else
	{
		throttle = 1;
	}
	if (altitude <= 150000 && safe_to_deploy_parachute())
	{
		parachute_status = DEPLOYED;
	}
}



void numerical_dynamics (void)
// This is the function that performs the numerical integration to update the
// lander's pose. The time step is delta_t (global variable).
{
	double modulusr = position.abs(), modulusv = velocity.abs(), mass;
	vector3d normalr = position.norm();
	vector3d normalv = velocity.norm();
	vector3d gravity_acceleration;
	vector3d drag_lander_acceleration;
	vector3d thrust_acceleration;
	vector3d drag_chute_acceleration;

	mass = (UNLOADED_LANDER_MASS+FUEL_CAPACITY*FUEL_DENSITY*fuel); //Mass of fully fueled lander
	gravity_acceleration = (-GRAVITY*MARS_MASS*normalr/(modulusr*modulusr)); //Acceleration due to gravity
	drag_lander_acceleration = (0.5*atmospheric_density(position)*DRAG_COEF_LANDER*3.141592654*LANDER_SIZE*LANDER_SIZE*modulusv*modulusv*normalv)/mass; //Acceleration due to lander drag
	thrust_acceleration = thrust_wrt_world()/mass; //Aceleration due to thrust
	drag_chute_acceleration = 0.5*atmospheric_density(position)*DRAG_COEF_CHUTE*20*LANDER_SIZE*LANDER_SIZE*modulusv*modulusv*normalv/mass; //Acceleration due to chute drag

	if (parachute_status == DEPLOYED)
	{
		vector3d acceleration = gravity_acceleration - drag_lander_acceleration - drag_chute_acceleration + thrust_acceleration;
		position = position + delta_t*velocity;
		velocity = velocity + delta_t*acceleration;
	}
	else
	{
		vector3d acceleration = gravity_acceleration - drag_lander_acceleration + thrust_acceleration;
		position = position + delta_t*velocity;
		velocity = velocity + delta_t*acceleration;
	}

	// Here we can apply an autopilot to adjust the thrust, parachute and attitude
	if (autopilot_enabled) autopilot();

	// Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
	if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
// Lander pose initialization - selects one of 10 possible scenarios
{
	// The parameters to set are:
	// position - in Cartesian planetary coordinate system (m)
	// velocity - in Cartesian planetary coordinate system (m/s)
	// orientation - in lander coordinate system (xyz Euler angles, degrees)
	// delta_t - the simulation time step
	// boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
	// scenario_description - a descriptive string for the help screen

	scenario_description[0] = "circular orbit";
	scenario_description[1] = "descent from 10km";
	scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
	scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
	scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
	scenario_description[5] = "descent from 200km";
	scenario_description[6] = "geostationary orbit";
	scenario_description[7] = "";
	scenario_description[8] = "";
	scenario_description[9] = "";

	switch (scenario) {

	case 0:
		// a circular equatorial orbit
		position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
		velocity = vector3d(0.0, -3247.087385863725, 0.0);
		orientation = vector3d(0.0, 90.0, 0.0);
		delta_t = 0.1;
		parachute_status = NOT_DEPLOYED;
		stabilized_attitude = false;
		autopilot_enabled = false;
		break;

	case 1:
		// a descent from rest at 10km altitude
		position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
		velocity = vector3d(0.0, 0.0, 0.0);
		orientation = vector3d(0.0, 0.0, 90.0);
		delta_t = 0.1;
		parachute_status = NOT_DEPLOYED;
		stabilized_attitude = true;
		autopilot_enabled = false;
		break;

	case 2:
		// an elliptical polar orbit
		position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
		velocity = vector3d(3500.0, 0.0, 0.0);
		orientation = vector3d(0.0, 0.0, 90.0);
		delta_t = 0.1;
		parachute_status = NOT_DEPLOYED;
		stabilized_attitude = false;
		autopilot_enabled = false;
		break;

	case 3:
		// polar surface launch at escape velocity (but drag prevents escape)
		position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
		velocity = vector3d(0.0, 0.0, 5027.0);
		orientation = vector3d(0.0, 0.0, 0.0);
		delta_t = 0.1;
		parachute_status = NOT_DEPLOYED;
		stabilized_attitude = false;
		autopilot_enabled = false;
		break;

	case 4:
		// an elliptical orbit that clips the atmosphere each time round, losing energy
		position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
		velocity = vector3d(4000.0, 0.0, 0.0);
		orientation = vector3d(0.0, 90.0, 0.0);
		delta_t = 0.1;
		parachute_status = NOT_DEPLOYED;
		stabilized_attitude = false;
		autopilot_enabled = false;
		break;

	case 5:
		// a descent from rest at the edge of the exosphere
		position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
		velocity = vector3d(0.0, 0.0, 0.0);
		orientation = vector3d(0.0, 0.0, 90.0);
		delta_t = 0.1;
		parachute_status = NOT_DEPLOYED;
		stabilized_attitude = true;
		autopilot_enabled = false;
		break;

	case 6:
		//geostationary orbit
		position = vector3d(20429635.87, 0.0, 0.0); //pow((GRAVITY*MARS_MASS)/pow(((2*3.141592654)/MARS_DAY),2),(1/3)
		//Could not get this to calculate correctly using cmath, but left formula here as reference
		velocity = vector3d(0.0, 1448.025, 0.0); //
		orientation = vector3d(0.0, 90.0, 0.0);
		delta_t = 0.1;
		parachute_status = NOT_DEPLOYED;
		stabilized_attitude = false;
		autopilot_enabled = false;
		break;

	case 7:
		break;

	case 8:
		break;

	case 9:
		break;

	}
}
