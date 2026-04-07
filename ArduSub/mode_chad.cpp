#include "Sub.h"

#define GUIDED_ATTITUDE_TIMEOUT_MS  1000    // guided mode's attitude controller times out after 1 second with no new updates


static Vector3p posvel_pos_target_cm;
static Vector3f posvel_vel_target_cms;

struct {
    uint32_t update_time_ms;
    float roll_cd;
    float pitch_cd;
    float yaw_cd;
    float climb_rate_cms;
} static guided_angle_state = {0, 0.0f, 0.0f, 0.0f, 0.0f};


bool ModeChad::init(bool ignore_checks){
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Activation du mode (giga)chad 🗿");

    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    sub.set_neutral_controls();
    angle_control_start();

    return true;
}


////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//////////////////////              EXPERIMENTAL            ////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////    ANGLE CONTROL   ////////////////////////////////

// initialise guided mode's angle controller
void ModeChad::angle_control_start()
{
    // initialise targets
    guided_angle_state.update_time_ms = AP_HAL::millis();
    guided_angle_state.roll_cd = ahrs.roll_sensor;
    guided_angle_state.pitch_cd = ahrs.pitch_sensor;
    guided_angle_state.yaw_cd = ahrs.yaw_sensor;
    guided_angle_state.climb_rate_cms = 0.0f;

    // // pilot always controls yaw
    // sub.yaw_rate_only = false;
    // set_auto_yaw_mode(AUTO_YAW_HOLD);
}

void ModeChad::angle_control_run()
{
    // if motors not enabled set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(0.0f,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        // initialise velocity controller
        position_control->init_z_controller();
        return;
    }

    // constrain desired lean angles
    float roll_in = guided_angle_state.roll_cd;
    float pitch_in = guided_angle_state.pitch_cd;
    float total_in = norm(roll_in, pitch_in);
    float angle_max = MIN(attitude_control->get_althold_lean_angle_max_cd(), sub.aparm.angle_max);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // wrap yaw request
    float yaw_in = wrap_180_cd(guided_angle_state.yaw_cd);

    // // constrain climb rate
    // float climb_rate_cms = constrain_float(guided_angle_state.climb_rate_cms, -sub.wp_nav.get_default_speed_down(), sub.wp_nav.get_default_speed_up());

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = AP_HAL::millis();
    if (tnow - guided_angle_state.update_time_ms > GUIDED_ATTITUDE_TIMEOUT_MS) {
        roll_in = 0.0f;
        pitch_in = 0.0f;
        // climb_rate_cms = 0.0f;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true);

    // no call to position controller since it's managed by the servo unit
}





// set_auto_yaw_mode - sets the yaw mode for auto
void ModeChad::set_auto_yaw_mode(autopilot_yaw_mode yaw_mode)
{
    // return immediately if no change
    if (sub.auto_yaw_mode == yaw_mode) {
        return;
    }
    sub.auto_yaw_mode = yaw_mode;

    // perform initialisation
    switch (sub.auto_yaw_mode) {
    
    case AUTO_YAW_HOLD:
        // pilot controls the heading
        break;

    case AUTO_YAW_LOOK_AT_NEXT_WP:
        // wpnav will initialise heading when wpnav's set_destination method is called
        break;

    case AUTO_YAW_ROI:
        // point towards a location held in yaw_look_at_WP
        sub.yaw_look_at_WP_bearing = ahrs.yaw_sensor;
        break;

    case AUTO_YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading
        // caller should set the yaw_look_at_heading
        break;

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        sub.yaw_look_ahead_bearing = ahrs.yaw_sensor;
        break;

    case AUTO_YAW_RESETTOARMEDYAW:
        // initial_armed_bearing will be set during arming so no init required
        break;
    
    case AUTO_YAW_RATE:
        // set target yaw rate to yaw_look_at_heading_slew
        break;
    }
}

////////////////////////////////     POS CONTROL    ////////////////////////////////

// // initialise guided mode's position controller
// void ModeChad::pos_control_start()
// {
//     // initialise waypoint controller
//     sub.wp_nav.wp_and_spline_init();

//     // initialise wpnav to stopping point at current altitude
//     // To-Do: set to current location if disarmed?
//     // To-Do: set to stopping point altitude?
//     Vector3f stopping_point;
//     sub.wp_nav.get_wp_stopping_point(stopping_point);

//     // no need to check return status because terrain data is not used
//     sub.wp_nav.set_wp_destination(stopping_point, false);
// }

// // pos_control_run - runs the guided position controller
// // called from guided_run
// void ModeChad::pos_control_run()
// {
//     // // if motors not enabled set throttle to zero and exit immediately
//     // if (!motors.armed()) {
//     //     motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
//     //     // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
//     //     attitude_control->set_throttle_out(0,true,g.throttle_filt);
//     //     attitude_control->relax_attitude_controllers();
//     //     sub.wp_nav.wp_and_spline_init();
//     //     return;
//     // }

//     // // process pilot's yaw input
//     // float target_yaw_rate = 0;
//     // if (!sub.failsafe.pilot_input) {
//     //     // get pilot's desired yaw rate
//     //     target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
//     //     if (!is_zero(target_yaw_rate)) {
//     //         set_auto_yaw_mode(AUTO_YAW_HOLD);
//     //     } else{
//     //         if (sub.yaw_rate_only){
//     //             set_auto_yaw_mode(AUTO_YAW_RATE);
//     //         } else{
//     //             set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
//     //         }
//     //     }
//     // }

//     // set motors to full range
//     motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

//     // run waypoint controller
//     sub.failsafe_terrain_set_status(sub.wp_nav.update_wpnav());

//     float lateral_out, forward_out;
//     sub.translate_wpnav_rp(lateral_out, forward_out);

//     // Send to forward/lateral outputs
//     motors.set_lateral(lateral_out);
//     motors.set_forward(forward_out);

//     // WP_Nav has set the vertical position control targets
//     // run the vertical position controller and set output throttle
//     position_control->update_z_controller();

//     // // call attitude controller
//     // if (sub.auto_yaw_mode == AUTO_YAW_HOLD) {
//     //     // roll & pitch & yaw rate from pilot
//     //     attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
//     // } else if (sub.auto_yaw_mode == AUTO_YAW_LOOK_AT_HEADING) {
//     //     // roll, pitch from pilot, yaw & yaw_rate from auto_control
//     //     target_yaw_rate = sub.yaw_look_at_heading_slew * 100.0;
//     //     attitude_control->input_euler_angle_roll_pitch_slew_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), target_yaw_rate);
//     // } else if (sub.auto_yaw_mode == AUTO_YAW_RATE) {
//     //     // roll, pitch from pilot, yaw_rate from auto_control
//     //     target_yaw_rate = sub.yaw_look_at_heading_slew * 100.0;
//     //     attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
//     // } else {
//     //     // roll, pitch from pilot, yaw heading from auto_heading()
//     //     attitude_control->input_euler_angle_roll_pitch_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), true);
//     // }
// }

// // set_destination - sets guided mode's target destination
// // Returns true if the fence is enabled and guided waypoint is within the fence
// // else return false if the waypoint is outside the fence
// bool ModeChad::set_destination(const Vector3f& destination)
// {
// #if AP_FENCE_ENABLED
//     // reject destination if outside the fence
//     const Location dest_loc(destination, Location::AltFrame::ABOVE_ORIGIN);
//     if (!sub.fence.check_destination_within_fence(dest_loc)) {
//         LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
//         // failure is propagated to GCS with NAK
//         return false;
//     }
// #endif

//     // ensure we are in position control mode
//     if (sub.guided_mode != Guided_WP) {
//         pos_control_start();
//     }

//     // no need to check return status because terrain data is not used
//     sub.wp_nav.set_wp_destination(destination, false);

// #if HAL_LOGGING_ENABLED
//     // log target
//     sub.Log_Write_GuidedTarget(sub.guided_mode, destination, Vector3f());
// #endif

//     return true;
// }




////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//////////////////////          END OF EXPERIMENTAL         ////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////


void ModeChad::PID_servo(Vector3<float> target, int dt, Vector3<float>& U){
    PIDx.set_kP(g.Px);
    PIDx.set_kI(g.Ix);
    PIDx.set_kD(g.Dx);

    PIDy.set_kP(g.Py);
    PIDy.set_kI(g.Iy);
    PIDy.set_kD(g.Dy);

    PIDz.set_kP(g.Pz);
    PIDz.set_kI(g.Iz);
    PIDz.set_kD(g.Dz);

    U[0] = PIDx.update_all(0, target[0], dt*1e-3);
    U[1] = PIDy.update_all(0, target[1], dt*1e-3);
    U[2] = PIDz.update_all(0, target[2], dt*1e-3);
}


// manual_run - runs the manual (passthrough) controller
// should be called at 100hz or more
void ModeChad::run(){

    // if not armed set throttle to zero and exit immediately
    if (!sub.motors.armed()) {
        sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        return;
    }

    
    
    angle_control_run();



    // Assumption : the vfov is oriented downwards.
    
    float vfov = AP_CHAD_ANGLE; // = - camera_backend.vertical_fov()
                                // orienté vers le haut
                                // -40° (-0.69813) ou 0°

    Quaternion cancel_cam_orientation; cancel_cam_orientation.from_euler(vfov, 0.f, 0.f);

    Vector3<float> target = {0.f, 0.f, 0.f};
    float& dx (target[0]), dy (target[1]), dz (target[2]);
    int dt;

    Vector3<float> U;
    float& Ux (U[0]), Uy (U[1]), Uz (U[2]);
    
    // Réception du capteur. Contrôle ssi nouvelle update reçue
    if (sub.chad.transmit(dx, dy, dz, dt) && dt<1000){

        // màj 
        last_instruction_date = AP_HAL::millis();

        // Changement de référentiel
        target = (cancel_cam_orientation) * target;
        


        // Asservissement du système
        
        PID_servo(target, dt, U);



        // Preprocess
        Uy = Uy/2 + 0.5; //From -1 <-> 1 to 0 <-> 1

        Ux = constrain_float(Ux, -1.f, 1.f);
        Uy = constrain_float(Uy, 0.f, 1.f);
        Uz = constrain_float(Uz, -1.f, 1.f);
        

        motors.set_lateral(Ux);
        motors.set_throttle(-Uy); // l'axe y est orienté à l'opposé de la poussée. 
        motors.set_forward(Uz);
    };
}

