#pragma once

#include "AP_CHAD_config.h"

#if AP_CHAD_ENABLED

#include <AP_HAL/utility/Socket.h>

class AP_CHAD{

public:
    AP_CHAD();
    ~AP_CHAD();

    void init();
    void read();

private:
    SocketAPM *socket;
    char buffer[12];

    float Sx, Sy, Sz; // Speed to apply
};

#endif