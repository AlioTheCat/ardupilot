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
    bool transmit(float& _dx, float& _dy, float& _dz, int& _dt);

    // Time in ms since last update.
    void update_time_dates(){
        uint32_t tmp = AP_HAL::millis();
        last_delta_time = tmp - last_update;
        last_update = tmp;
    }

    uint32_t get_last_update_time();

private:
    SocketAPM *socket;
    char buffer[12];

    float dx, dy, dz; // CHAD Processing Unit measurements
    bool new_update;
    uint32_t last_delta_time = 0;
    uint32_t last_update;
};

#endif