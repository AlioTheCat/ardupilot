#include "Sub.h"

#include <iostream>

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
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Activation du mode CHAD");

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

float ModeChad::remap_angle_diff(float val){
    if (val>18000){
        return (36000 - val);
    }
    else {
        return val;
    }
}

bool ModeChad::pos_servo_authorized(){
    float roll_diff = remap_angle_diff( guided_angle_state.roll_cd - ahrs.roll_sensor );
    float pitch_diff = remap_angle_diff( guided_angle_state.pitch_cd - ahrs.pitch_sensor );
    float yaw_diff = remap_angle_diff( guided_angle_state.yaw_cd - ahrs.yaw_sensor );
    std::cout << "roll diff : " << roll_diff << ", pitch_diff : " << pitch_diff << ", yaw_diff : " << yaw_diff << std::endl; 
    
    return (abs(roll_diff) < g2.roll_ctrl_threshold 
            && 
            abs(pitch_diff) < g2.pitch_ctrl_threshold
            &&
            abs(yaw_diff) < g2.yaw_ctrl_threshold);
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

/* Computes force to apply from measurement */
void ModeChad::PID_servo(Vector3<float> measure, int dt, Vector3<float>& F){
    // PID parameters in straight camera coordinate system
    PIDx.set_kP(g.Px);
    PIDx.set_kI(g.Ix);
    PIDx.set_kD(g.Dx);

    PIDy.set_kP(g.Py);
    PIDy.set_kI(g.Iy);
    PIDy.set_kD(g.Dy);

    PIDz.set_kP(g.Pz);
    PIDz.set_kI(g.Iz);
    PIDz.set_kD(g.Dz);

    F[0] = - PIDx.update_all(0, measure[0], dt*1e-3); // horizontal axis goes from right (-1) to left (1)
    F[1] = PIDy.update_all(0, measure[1], dt*1e-3);
    F[2] = PIDz.update_all(0, measure[2], dt*1e-3);
}

bool ModeChad::timeout() { return AP_HAL::millis()-sub.chad.get_last_update_time() > 1000; }


// manual_run - runs the manual (passthrough) controller
// should be called at 5hz or more
void ModeChad::run(){

    // check if connection with CPU still holds, otherwise disarm and alert the user
    if (timeout() && sub.motors.armed()){
            ///// CORRECT DISARM METHOD //////
            sub.arming.disarm(AP_Arming::Method::CHADFAILSAFE, true);
            //////////////////////////////////
            sub.motors.output();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "No instruction received for 1 sec => Motors disarmed.");
            return; 
    }

    // if not armed set throttle to zero and exit immediately
    if (!sub.motors.armed()) {
        sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    if (g2.angle_ctrl_active == 1) angle_control_run();

    // Assumption : the vfov is oriented downwards.
        
    float vfov = AP_CHAD_ANGLE; // = - camera_backend.vertical_fov()
                                // orienté vers le haut
                                // -40° (-0.69813) ou 0°

    Quaternion cancel_cam_orientation; cancel_cam_orientation.from_euler(vfov, 0.f, 0.f);

    Vector3<float> measure = {0.f, 0.f, 0.f};
    float& dx (measure[0]); float& dy (measure[1]) ; float& dz (measure[2]); // measurement data
    int dt=0;
    float status = 0.0;

    // std::cout << "CHAD : Coucou j'entre dans la fonction run";

    Vector3<float> F = {0.f, 0.f, 0.f};
    float& F_lateral (F[0]); float& F_throttle (F[1]); float& F_forward (F[2]); // force to apply to the ROV to control translation

    bool transmition(sub.chad.transmit(dx, dy, dz, dt, status));
    // std :: cout << " dx : " << dx << ", dy : " << dy << ", dz : " << dz << ", dt : " << dt << ", transmit : " << transmition << std::endl;
    
    if (status==1.0) // Reference frame reset requested
    {
        // Resets the target attitude
        guided_angle_state.roll_cd = ahrs.roll_sensor;
        guided_angle_state.pitch_cd = ahrs.pitch_sensor;
        guided_angle_state.yaw_cd = ahrs.yaw_sensor;

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD : Target attitude reset (CPU : reference frame reset)");
    }

    // Changement de base prenant en compte les angles mesurés par l'IMU

    Quaternion adapt_coordinate_system; adapt_coordinate_system.from_euler(
                                                                            -remap_angle_diff(guided_angle_state.pitch_cd - ahrs.pitch_sensor),
                                                                            -remap_angle_diff(guided_angle_state.yaw_cd - ahrs.yaw_sensor),
                                                                            remap_angle_diff(guided_angle_state.roll_cd - ahrs.roll_sensor)
                                                                           );

    if (pos_servo_authorized()){
        
        

        // Réception du capteur. Contrôle ssi nouvelle update reçue
        if (transmition){

            // Changement de référentiel 
            Vector3<float> measure_camera_straight = (cancel_cam_orientation) * measure;

            // Adaptation aux données de l'IMU
            Vector3<float> measure_rotated_with_IMU = (adapt_coordinate_system) * measure_camera_straight;

            // Asservissement de la position
            PID_servo(measure_rotated_with_IMU, dt, F);

            // Scaling des instructions
            F_throttle = F_throttle/2 + 0.5; //From -1 <-> 1 to 0 <-> 1

            F_lateral = constrain_float(F_lateral, -1.f, 1.f);
            F_throttle = constrain_float(F_throttle, 0.f, 1.f);
            F_forward = constrain_float(F_forward, -1.f, 1.f);

            // Transmission des instructions moteur
            
            motors.set_forward(F_forward); // set_forward = vers l'avant (entre -1 et 1)
            motors.set_lateral(F_lateral); // set_lateral = vers la gauche (entre -1 et 1)
            motors.set_throttle(F_throttle); // set_throttle = vers le haut (entre 0 et 1)
        }
    } else {
        motors.set_forward(0);
        motors.set_lateral(0);
        motors.set_throttle(0.5);
    }
}

