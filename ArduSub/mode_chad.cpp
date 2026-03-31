#include "Sub.h"

bool ModeChad::init(bool ignore_checks){
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Activation du mode (giga)chad 🗿");

    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    sub.set_neutral_controls();
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

    // Assumption : the vfov is oriented downwards.
    
    float vfov = 0.0; // = - camera_backend.vertical_fov()
                      // orienté vers le haut

    Quaternion cancel_cam_orientation; cancel_cam_orientation.from_euler(vfov, 0.0, 0.0);

    float dx, dy, dz;
    sub.chad.transmit(dx, dy, dz);

    Vector3<float> displacement = {dx, dy, dz};
    displacement = (cancel_cam_orientation) * displacement;

    
    // PIDx.kP(g.Px);
    // PIDx.kI(g.Ix);
    // PIDx.kD(g.Dx);

    // PIDy.kP(g.Py);
    // PIDy.kI(g.Iy);
    // PIDy.kD(g.Dy);

    // PIDz.kP(g.Pz);
    // PIDz.kI(g.Iz);
    // PIDz.kD(g.Dz);

    // Ux = PIDx.update_all(0, V_robot.x, dt*1e-3);
    // Uy = PIDy.update_all(0, V_robot.y, dt*1e-3);
    // Uz = PIDz.update_all(0, V_robot.z, dt*1e-3);

    // // Preprocess
    // Uy = Uy/2 + 0.5; //From -1 <-> 1 to 0 <-> 1

    // Ux = std::min(1.0, std::max(-1.0, Ux));
    // Uy = std::min(1.0, std::max(0.0, Uy));
    // Uz = std::min(1.0, std::max(-1.0, Uz));
    

    motors.set_lateral(dx);
    motors.set_throttle(-dy); 
    motors.set_forward(dz);
}