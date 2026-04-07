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
    bool transmit(float& dx, float& dy, float& dz, int& dt);

    // Time in ms since last update.
    void update_time_dates(){
        uint32_t tmp = AP_HAL::millis();
        last_delta_time = tmp - last_update;
        last_update = tmp;
    }

private:
    SocketAPM *socket;
    char buffer[12];

    float Sx, Sy, Sz; // Speed to apply
    bool new_update;
    uint32_t last_delta_time = 0;
    uint32_t last_update;
};

#endif