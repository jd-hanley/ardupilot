#include "Copter.h"
#include "AP_Vehicle/AP_Vehicle.h"

#if MODE_DRIFT_ENABLED

/*
 * Init and run calls for drift flight mode
 */

#ifndef DRIFT_SPEEDGAIN
 # define DRIFT_SPEEDGAIN 8.0f
#endif
#ifndef DRIFT_SPEEDLIMIT
 # define DRIFT_SPEEDLIMIT 560.0f
#endif

#ifndef DRIFT_THR_ASSIST_GAIN
 # define DRIFT_THR_ASSIST_GAIN 0.0018f    // gain controlling amount of throttle assistance
#endif

#ifndef DRIFT_THR_ASSIST_MAX
 # define DRIFT_THR_ASSIST_MAX  0.3f    // maximum assistance throttle assist will provide
#endif

#ifndef DRIFT_THR_MIN
 # define DRIFT_THR_MIN         0.213f  // throttle assist will be active when pilot's throttle is above this value
#endif
#ifndef DRIFT_THR_MAX
 # define DRIFT_THR_MAX         0.787f  // throttle assist will be active when pilot's throttle is below this value
#endif

// drift_init - initialise drift controller
bool ModeDrift::init(bool ignore_checks)
{
    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    switch_time = 0.0f;
    disturbance_time = 0.0f;
    disturbance.init();
    // copter.strain.calibrate_all();
    return true;
}

// drift_run - runs the drift controller
// should be called at 100hz or more
void ModeDrift::run()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate();

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHoldModeState::Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AP_AVOIDANCE_ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    disturbance_time += G_Dt;
    switch_time += G_Dt;

    // If we are within the mode switch delay time period or the status of any sensors is not operational, use the original z controller while the sensors are calibrated
    // if ((switch_time - switch_delay < 0) || !copter.strain.get_status_all())
    // {
    //     pos_control->update_z_controller();
    // }
    // // Otherwise, the sensors have had time to calibrate and thus we can use the new z controller
    // else
    // {
    //     float multiplier = disturbance.update(disturbance_time);
    //     pos_control->update_z_controller_strain(multiplier);
    // }
    // float multiplier = disturbance.update(disturbance_time);
    // pos_control->update_z_controller_strain(multiplier);
}

// get_throttle_assist - return throttle output (range 0 ~ 1) based on pilot input and z-axis velocity
float ModeDrift::get_throttle_assist(float velz, float pilot_throttle_scaled)
{
    // throttle assist - adjusts throttle to slow the vehicle's vertical velocity
    //      Only active when pilot's throttle is between 213 ~ 787
    //      Assistance is strongest when throttle is at mid, drops linearly to no assistance at 213 and 787
    float thr_assist = 0.0f;
    if (pilot_throttle_scaled > DRIFT_THR_MIN && pilot_throttle_scaled < DRIFT_THR_MAX) {
        // calculate throttle assist gain
        thr_assist = 1.2f - ((float)fabsf(pilot_throttle_scaled - 0.5f) / 0.24f);
        thr_assist = constrain_float(thr_assist, 0.0f, 1.0f) * -DRIFT_THR_ASSIST_GAIN * velz;

        // ensure throttle assist never adjusts the throttle by more than 300 pwm
        thr_assist = constrain_float(thr_assist, -DRIFT_THR_ASSIST_MAX, DRIFT_THR_ASSIST_MAX);
    }
    
    return constrain_float(pilot_throttle_scaled + thr_assist, 0.0f, 1.0f);
}
#endif
