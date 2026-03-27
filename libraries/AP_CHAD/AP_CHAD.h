#pragma once

#include "AP_CHAD_config.h"

#if AP_CHAD_ENABLED

#include <AP_HAL/utility/Socket.h>

class AP_CHAD{

public:
    void init();
    void read();

private:
    // SocketAPM *socket;
    // char buffer[12];
    // uint32_t station_ip;
    // uint32_t station_port;

    //char* part1 = &buffer[0];
    //char* part2 = &buffer[4];
    //char* part3 = &buffer[8];
    //float Sx, Sy, Sz;
};

#endif