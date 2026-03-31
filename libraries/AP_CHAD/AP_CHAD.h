#pragma once

#include "AP_CHAD_config.h"

#if AP_CHAD_ENABLED

#include <AP_HAL/utility/Socket.h>
#include <ctime>
#include <vector>

class AP_CHAD{

public:
    AP_CHAD();
    ~AP_CHAD();

    void init();
    void read();
    void transmit(float& dx, float& dy, float& dz);

    // Time in ms since last update.
    uint32_t time_since_last_update(){return AP_HAL::millis() - last_update;}

private:
    SocketAPM *socket;
    char buffer[12];

    float Sx, Sy, Sz; // Speed to apply
    bool new_update;
    std::clock_t last_update;
};

#endif