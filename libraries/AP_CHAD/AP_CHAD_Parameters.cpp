#include <AP_CHAD/AP_CHAD_Parameters.h>

#include <AP_Param/AP_Param.h>

const AP_Param::GroupInfo AP_CHAD_Parameters::var_info[] = {

    // WARNING : The first argument of AP_GROUPINFO macro should be 10 characters at max. Otherwise the firmware will crash

    // @Param: CHAD_Ix
    // @DisplayName: Integ x
    // @Description: Integral gain for x axis
    // @Values: float
    // @User: Standard
    AP_GROUPINFO("Ix", 1,  AP_CHAD_Parameters,    Ix,   0),

    // @Param: CHAD_Px
    // @DisplayName: Prop x
    // @Description: Proportional gain for x axis
    // @Values: float
    // @User: Standard
    AP_GROUPINFO("Px", 2,  AP_CHAD_Parameters,    Px,   0),

    // @Param: CHAD_Dx
    // @DisplayName: Deriv x
    // @Description: Derivative gain for x axis
    // @Values: float
    // @User: Standard
    AP_GROUPINFO("Dx", 3,  AP_CHAD_Parameters,    Dx,   0),



    // @Param: CHAD_Iy
    // @DisplayName: Integ y
    // @Description: Integral gain for y axis
    // @Values: float
    // @User: Standard
    AP_GROUPINFO("Iy", 4,  AP_CHAD_Parameters,    Iy,   0),

    // @Param: CHAD_Py
    // @DisplayName: Prop y
    // @Description: Proportional gain for y axis
    // @Values: float
    // @User: Standard
    AP_GROUPINFO("Py", 5,  AP_CHAD_Parameters,    Py,   0),
    
    // @Param: CHAD_Dy
    // @DisplayName: Deriv y
    // @Description: Derivative gain for y axis
    // @Values: float
    // @User: Standard
    AP_GROUPINFO("Dy", 6,  AP_CHAD_Parameters,    Dy,   0),



    // @Param: CHAD_Iz
    // @DisplayName: Integ z
    // @Description: Integral gain for z axis
    // @Values: float
    // @User: Standard
    AP_GROUPINFO("Iz", 7,  AP_CHAD_Parameters,    Iz,   0),

    // @Param: CHAD_Pz
    // @DisplayName: Prop z
    // @Description: Proportional gain for z axis
    // @Values: float
    // @User: Standard
    AP_GROUPINFO("Pz", 8,  AP_CHAD_Parameters,    Pz,   0),

    // @Param: CHAD_Dz
    // @DisplayName: Deriv z
    // @Description: Derivative gain for z axis
    // @Values: float
    // @User: Standard
    AP_GROUPINFO("Dz", 9,  AP_CHAD_Parameters,    Dz,   0),

    // @Param: CHAD_roll_ctrl_threshold
    // @DisplayName: roll ctrl threshold
    // @Description: the roll threshold (in absolute value) over which translation should not be handled
    // @Values: float
    // @User: Standard
    AP_GROUPINFO("r_ctrl_thr", 10, AP_CHAD_Parameters, roll_ctrl_threshold, 300),

    // @Param: CHAD_pitch_ctrl_threshold
    // @DisplayName: pitch ctrl threshold
    // @Description: the pitch threshold (in absolute value) over which translation should not be handled
    // @Values: float
    // @User: Standard
    AP_GROUPINFO("p_ctrl_thr", 11, AP_CHAD_Parameters, pitch_ctrl_threshold, 300),

    // @Param: CHAD_yaw_ctrl_threshold
    // @DisplayName: yaw ctrl threshold
    // @Description: the yaw threshold (in absolute value) over which translation should not be handled
    // @Values: float
    // @User: Standard
    AP_GROUPINFO("y_ctrl_thr", 12, AP_CHAD_Parameters, yaw_ctrl_threshold, 300),

    // @Param: CHAD_attitude_ctrl_enabled
    // @DisplayName: attitude ctrl enabled
    // @Description: Whether are not the ROV should try to control its attitude while in CHAD mode. (For DEBUG purposes). Set to 1 to enable and 0 to disable
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ctrl_att", 13, AP_CHAD_Parameters, attitude_ctrl_enabled, 1),

    AP_GROUPEND
};

AP_CHAD_Parameters::AP_CHAD_Parameters() {
    AP_Param::setup_object_defaults(this, var_info);
}