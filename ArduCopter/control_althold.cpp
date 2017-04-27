


#include "Copter.h"

/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Copter::althold_init(bool ignore_checks) {
#if FRAME_CONFIG == HELI_FRAME
	// do not allow helis to enter Alt Hold if the Rotor Runup is not complete
	if (!ignore_checks && !motors.rotor_runup_complete()) {
		return false;
	}
#endif

	// initialize vertical speeds and leash lengths
	pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
	pos_control.set_accel_z(g.pilot_accel_z);

	// initialise position and desired velocity
	if (!pos_control.is_active_z()) {
		pos_control.set_alt_target_to_current_alt();
		pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());
	}

	// stop takeoff if running
	takeoff_stop();

	return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Copter::althold_run() {
	AltHoldModeState althold_state;
	float takeoff_climb_rate = 0.0f;

	// initialize vertical speeds and acceleration
	pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
	pos_control.set_accel_z(g.pilot_accel_z);

	// apply SIMPLE mode transform to pilot inputs
	update_simple_mode();

	// get pilot desired lean angles
	float target_roll, target_pitch;
	get_pilot_desired_lean_angles(channel_roll->get_control_in(),
			channel_pitch->get_control_in(), target_roll, target_pitch,
			attitude_control.get_althold_lean_angle_max());

	// get pilot's desired yaw rate
	float target_yaw_rate = get_pilot_desired_yaw_rate(
			channel_yaw->get_control_in());

	// get pilot desired climb rate
	float target_climb_rate = get_pilot_desired_climb_rate(
			channel_throttle->get_control_in());
	target_climb_rate = constrain_float(target_climb_rate,
			-g.pilot_velocity_z_max, g.pilot_velocity_z_max);


	//take backup of the target climb rate.
	float target_climb_rate_bkp = target_climb_rate;

	// float named target_stop_alt is calculated and represents the altitude
	//the drone would reach if the throttle is fully applied at the time of the code executing.
	//This is achieved using a simple parabolic calculation based on the current velocity and
	//acceleration as below.
    float current_velocity = inertial_nav.get_velocity_z();
	float current_alt = rangefinder_state.alt_cm;
	float acceleration = g.pilot_accel_z; // m/s/s
	float target_stop_alt = current_alt - current_velocity*current_velocity / (2* acceleration) ;

	//we are mowing upwards so target stop alt will be very high if not infinite.
	if(current_velocity >0 ){
		target_stop_alt = 10000;
	}

	//
    if(target_stop_alt>= g2.cp_min_alt*2){
    	// do nothing..
    }
    //in danger zone ( yellow zone) where we attenuate users ability to do drastice changes to altitude.
    else if(
    	(target_stop_alt >= g2.cp_min_alt && target_stop_alt <  g2.cp_min_alt*2 )
    	||
    	(current_alt >= g2.cp_min_alt && current_alt<  g2.cp_min_alt*2 ))
    {
		float alpha = (MIN(current_alt,target_stop_alt) - g2.cp_min_alt) / g2.cp_min_alt;
		alpha = constrain_float(alpha,0,1.0);

		//control max and min speeds with alpha.
		//pos_control.set_speed_z(-1*alpha*g.pilot_velocity_z_max,g.pilot_velocity_z_max);


		//target climb rate can not be below what alpha provides..
		target_climb_rate= constrain_float(target_climb_rate, -1*g.pilot_velocity_z_max*alpha,g.pilot_velocity_z_max);

	  }
    //super danger , only slight positive climb is allowed here.
    if(target_stop_alt < g2.cp_min_alt  || current_alt < g2.cp_min_alt)
	  {
    	  //set minimum velocity of drone to -1
		  pos_control.set_speed_z(-1,g.pilot_velocity_z_max);

		  //5 % seems to work ..
		  target_climb_rate = g.pilot_velocity_z_max*0.1;

	  }

	  //just override user input if its greater..
	  target_climb_rate = MAX(target_climb_rate,target_climb_rate_bkp);


#if FRAME_CONFIG == HELI_FRAME
	// helicopters are held on the ground until rotor speed runup has finished
	bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors.rotor_runup_complete());
#else
	bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif

	// Alt Hold State Machine Determination
	if (!motors.armed() || !motors.get_interlock()) {
		althold_state = AltHold_MotorStopped;
	} else if (takeoff_state.running || takeoff_triggered) {
		althold_state = AltHold_Takeoff;
	} else if (!ap.auto_armed || ap.land_complete) {
		althold_state = AltHold_Landed;
	} else {
		althold_state = AltHold_Flying;
	}

	// Alt Hold State Machine
	switch (althold_state) {

	case AltHold_MotorStopped:

		motors.set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
		attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(
				target_roll, target_pitch, target_yaw_rate,
				get_smoothing_gain());
#if FRAME_CONFIG == HELI_FRAME    
		// force descent rate and call position controller
		pos_control.set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
#else
		attitude_control.reset_rate_controller_I_terms();
		attitude_control.set_yaw_target_to_current_heading();
		pos_control.relax_alt_hold_controllers(0.0f); // forces throttle output to go to zero
#endif
		pos_control.update_z_controller();
		break;

	case AltHold_Takeoff:
		// set motors to full range
		motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

		// initiate take-off
		if (!takeoff_state.running) {
			takeoff_timer_start(
					constrain_float(g.pilot_takeoff_alt, 0.0f, 1000.0f));
			// indicate we are taking off
			set_land_complete(false);
			// clear i terms
			set_throttle_takeoff();
		}

		// get take-off adjusted pilot and takeoff climb rates
		takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

		// call attitude controller
		attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(
				target_roll, target_pitch, target_yaw_rate,
				get_smoothing_gain());

		// call position controller
		pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt,
				false);
		pos_control.add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
		pos_control.update_z_controller();
		break;

	case AltHold_Landed:
		// set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
		if (target_climb_rate < 0.0f) {
			motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
		} else {
			motors.set_desired_spool_state(
					AP_Motors::DESIRED_THROTTLE_UNLIMITED);
		}

		attitude_control.reset_rate_controller_I_terms();
		attitude_control.set_yaw_target_to_current_heading();
		attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(
				target_roll, target_pitch, target_yaw_rate,
				get_smoothing_gain());
		pos_control.relax_alt_hold_controllers(0.0f); // forces throttle output to go to zero
		pos_control.update_z_controller();
		break;

	case AltHold_Flying:
		motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
		// call attitude controller
		attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(
				target_roll, target_pitch, target_yaw_rate,
				get_smoothing_gain());

		// adjust climb rate using rangefinder
		if (rangefinder_alt_ok()) {
			// if rangefinder is ok, use surface tracking
			target_climb_rate = get_surface_tracking_climb_rate(
					target_climb_rate, pos_control.get_alt_target(), G_Dt);
		}



		// call position controller
		pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt,
				false);
		pos_control.update_z_controller();
		break;
	}
}
