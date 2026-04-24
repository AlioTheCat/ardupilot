#ifndef DEFINE_AP_CHAD_PARAMETERS
#define DEFINE_AP_CHAD_PARAMETERS

#include <AP_Math/AP_Math.h> 


class AP_CHAD_Parameters {

public: 
    AP_CHAD_Parameters();

    AP_Float Px, Py, Pz;
    AP_Float Ix, Iy, Iz;
    AP_Float Dx, Dy, Dz;

    AP_Float roll_ctrl_threshold, pitch_ctrl_threshold, yaw_ctrl_threshold;
    AP_Int8 attitude_ctrl_enabled;

    // parameter var info table
    static const struct AP_Param::GroupInfo var_info[];

private:

};

#endif