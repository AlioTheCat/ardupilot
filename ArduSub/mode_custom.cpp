#include "Sub.h"


bool ModeCustom::init(bool ignore_checks) {

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Activation du mode custom !");

    // set target altitude to zero for reporting
    position_control->set_pos_desired_z_cm(0);

    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    sub.set_neutral_controls();

    return true;
}

// manual_run - runs the manual (passthrough) controller
// should be called at 100hz or more
void ModeCustom::run()
{
    // if not armed set throttle to zero and exit immediately
    if (!sub.motors.armed()) {
        sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        return;
    }

    sub.camera.get_fov_status();

    sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    sub.motors.set_roll(channel_roll->norm_input());
    sub.motors.set_pitch(channel_pitch->norm_input());
    sub.motors.set_yaw(channel_yaw->norm_input() * g.acro_yaw_p / ACRO_YAW_P);
    sub.motors.set_throttle(channel_throttle->norm_input());
    sub.motors.set_forward(channel_forward->norm_input());
    sub.motors.set_lateral(channel_lateral->norm_input());
}
