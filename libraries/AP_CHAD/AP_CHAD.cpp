#include "AP_CHAD_config.h"
#include <GCS_MAVLink/GCS.h>

#include <iostream>

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
    // std::cout << "CHAD sensor : J'entre dans la fonction read" << std::endl;
    ssize_t len = socket -> recv(buffer, sizeof(buffer), 0);
    // std::cout << "socket length : " << len << ", buffer size : " << sizeof(buffer) << std::endl;
    if (len > 0) {
        if (len == sizeof(buffer)) {

            memcpy(&dx,     buffer  , 4);
            memcpy(&dy,     buffer+4, 4);
            memcpy(&dz,     buffer+8, 4);
            memcpy(&status, buffer+12, 4);

            //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor : received : %f, %f, %f", dx, dy, dz);

            // std::cout << "CHAD sensor : received : " << dx << ", " << dy << ", " << dz << std::endl; 

            update_time_dates();
            new_update=true;
        }
        else 
        {
            //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor : data incomplete");   
        }
    } else {
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor : waiting");
    }
}

bool AP_CHAD::transmit(float& _dx, float& _dy, float& _dz, int& _dt, float& _status) {
    // Returns false if this update is not new
    if (new_update){
        new_update = false;
        _dx = dx;
        _dy = dy;
        _dz = dz;
        _dt = last_delta_time;
        _status = status;
        return true;
    }
    return false;
}

uint32_t AP_CHAD::get_last_update_time() { return last_update; }

#endif