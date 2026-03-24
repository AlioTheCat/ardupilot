#include "AP_CHAD_config.h"
#include <GCS_MAVLink/GCS.h>

#if AP_CHAD_ENABLED

#include "AP_CHAD.h"

void AP_CHAD::init()
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor initialized");
}

void AP_CHAD::read(){
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor read");
}

#endif