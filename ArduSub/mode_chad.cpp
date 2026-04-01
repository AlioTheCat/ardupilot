#include "Sub.h"

#define GUIDED_ATTITUDE_TIMEOUT_MS  1000    // guided mode's attitude controller times out after 1 second with no new updates


static Vector3p posvel_pos_target_cm;
static Vector3f posvel_vel_target_cms;
static uint32_t update_time_ms;

struct {
    uint32_t update_time_ms;
    float roll_cd;
    float pitch_cd;
    float yaw_cd;
    float climb_rate_cms;
} static guided_angle_state = {0,0.0f, 0.0f, 0.0f, 0.0f};


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

    // pilot always controls yaw
    sub.yaw_rate_only = false;
    set_auto_yaw_mode(AUTO_YAW_HOLD);
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

    // constrain climb rate
    float climb_rate_cms = constrain_float(guided_angle_state.climb_rate_cms, -sub.wp_nav.get_default_speed_down(), sub.wp_nav.get_default_speed_up());

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = AP_HAL::millis();
    if (tnow - guided_angle_state.update_time_ms > GUIDED_ATTITUDE_TIMEOUT_MS) {
        roll_in = 0.0f;
        pitch_in = 0.0f;
        climb_rate_cms = 0.0f;
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
    
    float vfov = 0.f; // = - camera_backend.vertical_fov()
                      // orienté vers le haut

    Quaternion cancel_cam_orientation; cancel_cam_orientation.from_euler(vfov, 0.f, 0.f);

    Vector3<float> target = {0.f, 0.f, 0.f};
    float& dx = target[0];
    float& dy = target[1];
    float& dz = target[2];
    int dt;

    Vector3<float> U;
    float& Ux = U[0];
    float& Uy = U[1];
    float& Uz = U[2];
    
    // Réception du capteur
    sub.chad.transmit(dx, dy, dz, dt);
    // Changement de référentiel
    target = (cancel_cam_orientation) * target;

    dt = std::min(1000, dt); // Pas plus d'une seconde.
    
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
}

