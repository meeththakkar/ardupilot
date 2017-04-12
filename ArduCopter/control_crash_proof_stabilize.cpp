#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::crash_proof_stabilize_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors.armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);

    //{0,0,0,new AC_PID(0.5,0.0005,0.1,0.7,50,0.003),40.0, CrashProofStabilizeFlying}

	crash_proof_stabilize_state.current_velocity = 0.0;
	crash_proof_stabilize_state.current_acceleration = 0.0;
	crash_proof_stabilize_state.target_stop_alt = 0.0;
	if (!crash_proof_stabilize_state.pid_initialized) {
		crash_proof_stabilize_state.pid = new AC_PID(0.5, 0.0005, 0.1, 0.7, 50,
				0.003);
		crash_proof_stabilize_state.pid_initialized = true;
	}
	crash_proof_stabilize_state.threshold_altitude = 40.0;
	crash_proof_stabilize_state.state = CrashProofStabilizeFlying;
	return true;

}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::crash_proof_stabilize_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // if not armed set throttle to zero and exit immediately
    //if (!motors.armed() || ap.throttle_zero || !motors.get_interlock()) {
    	if (!motors.armed() || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    int state_old = crash_proof_stabilize_state.state;



    //Crash Proof Code..

    //TODO: filtering externalize variable.
    crash_proof_stabilize_state.current_acceleration = crash_proof_stabilize_state.current_acceleration*0.99 + 0.01*(inertial_nav.get_velocity_z()- crash_proof_stabilize_state.current_velocity) /G_Dt;
    crash_proof_stabilize_state.current_velocity = inertial_nav.get_velocity_z();

    float current_velocity =  crash_proof_stabilize_state.current_velocity;
    float current_acceleration = crash_proof_stabilize_state.current_acceleration;
    float current_alt = rangefinder_state.alt_cm;

    //simulation value came to around 800
    float acceleration = 700; // m/s/s
    crash_proof_stabilize_state.target_stop_alt = current_alt - current_velocity*current_velocity / (2* acceleration) ;
    float target_stop_alt = crash_proof_stabilize_state.target_stop_alt;


    //decide mode...

       if(state_old == CrashProofStabilizeFlying && target_stop_alt < crash_proof_stabilize_state.threshold_altitude ){

    	 printf("Break, vel %f, alt %f\n", current_velocity,current_alt);

    	   crash_proof_stabilize_state.state = CrashProofStabilizeBreaking;
       }

       if(state_old == CrashProofStabilizeBreaking && current_velocity > -20){
    	   crash_proof_stabilize_state.state = CrashProofStabilizeHolding;
       }

       if(state_old == CrashProofStabilizeHolding && current_alt > crash_proof_stabilize_state.threshold_altitude*1.5){
    	   crash_proof_stabilize_state.state = CrashProofStabilizeFlying;
       }



       switch (crash_proof_stabilize_state.state){

       case CrashProofStabilizeFlying:
    	   	   crash_proof_stabilize_state.pid->reset_I();
    	   	   break;

       case CrashProofStabilizeBreaking:
           	   crash_proof_stabilize_state.pid->reset_I();
           	  attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(0.0, 0.0, 0.0, get_smoothing_gain());
           	   pilot_throttle_scaled = 1.0;
           	   break;

       case CrashProofStabilizeHolding:

		float pilot_throttle_scaled_bkp = pilot_throttle_scaled;
		float effort = 0.0;

		crash_proof_stabilize_state.pid->set_desired_rate(0);
		float error = crash_proof_stabilize_state.threshold_altitude
				- target_stop_alt;
		if (error < 0) {
			error = 0;
			crash_proof_stabilize_state.pid->reset_I();
		}
		crash_proof_stabilize_state.pid->set_input_filter_all(error);
		crash_proof_stabilize_state.pid->set_dt(G_Dt);
		effort = crash_proof_stabilize_state.pid->get_pid();
		pilot_throttle_scaled = constrain_float(effort, 0.0, 1.0);
		//  printf("err:%f\t eff:%f\n",error,pilot_throttle_scaled);

		pilot_throttle_scaled = MAX(pilot_throttle_scaled,
				pilot_throttle_scaled_bkp);

		break;
       }


/*
    if(state_old == CrashProofStabilizeUserInputGood && target_stop_alt < crash_proof_stabilize_state.threshold_altitude ){
    	crash_proof_stabilize_state.state = CrashProofStabilizeUserIsCrashing;
    }
*/


   /* crash_proof_stabilize_state.state = CrashProofStabilizeUserIsCrashing;
    switch (crash_proof_stabilize_state.state){

    case CrashProofStabilizeUserIsCrashing:
    	printf("%f CrashProofStabilizeUserIsCrashing\n",target_stop_alt);
    	crash_proof_stabilize_state.pid->set_desired_rate(crash_proof_stabilize_state.threshold_altitude);
    	crash_proof_stabilize_state.pid->set_input_filter_all(500);
    //	effort = crash_proof_stabilize_state.pid->get_pid();
    //	pilot_throttle_scaled = constrain_float(effort,0.0,1.0);
    	break;

    case CrashProofStabilizeUserInputGood:
    	printf("%fCrashProofStabilizeUserInputGood\n",target_stop_alt);
    	break;
    }

    if(state_old == CrashProofStabilizeUserIsCrashing && pilot_throttle_scaled_bkp >  pilot_throttle_scaled  ){
          	crash_proof_stabilize_state.state = CrashProofStabilizeUserInputGood;
          	pilot_throttle_scaled = pilot_throttle_scaled_bkp;
       }*/

   // printf("%f\t %f\t %f\t%f\t %f\n",current_velocity ,current_acceleration,target_stop_alt,current_alt,pilot_throttle_scaled);

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}
