#include "AP_CHAD_config.h"
#include <GCS_MAVLink/GCS.h>

#if AP_CHAD_ENABLED

#include "AP_CHAD.h"

AP_CHAD::AP_CHAD() {
    socket = new SocketAPM(true); // true = UDP/datagram
}

AP_CHAD::~AP_CHAD() {
    delete socket;
}

void AP_CHAD::init()
{
    if (!socket) {
        // handle allocation issue
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CHAD sensor : socket allocation issue");
        return;
    }

    if (!socket -> bind("0.0.0.0", 1106)) {
        // handle bind failure
        delete socket;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CHAD sensor : socket bind failed");
        return;
    }

    last_update = AP_HAL::millis();

    socket -> set_blocking(false); //continuous operation

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor initialized");
}

void AP_CHAD::read()
{
    ssize_t len = socket -> recv(buffer, sizeof(buffer), 0);
    if (len > 0) {
        if (len == sizeof(buffer)) {

            memcpy(&Sx, buffer  , 4);
            memcpy(&Sy, buffer+4, 4);
            memcpy(&Sz, buffer+8, 4);

            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor : received : %f, %f, %f", Sx, Sy, Sz);

            update_time_dates();
        }
        else 
        {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor : data incomplete");   
        }
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor : waiting");
    }
}

void AP_CHAD::transmit(float& dx, float& dy, float& dz, int& dt) {
    dx = Sx;
    dy = Sy;
    dz = Sz;
    dt = last_delta_time;
}

#endif