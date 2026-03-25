#pragma once

#include "AP_CHAD_config.h"

#include <netinet/in.h> // defines sockaddr_in struct
#include <sys/socket.h> // defines socklen_t class

#if AP_CHAD_ENABLED



class AP_CHAD{

public:
    void init();
    void read();

private:

    // network configuration : 
    // socket adress configuration : 
    struct sockaddr_in station_addr; // IPv4 + port of GCS
    struct sockaddr_in local_addr; // IPv4 + port of ROV

    socklen_t station_addr_len; // memory usage of the station_addr socket
    socklen_t local_addr_len; // memory usage of the local_addr socket

    int socket_fd; // -1 : error, >= 0 : file descriptor (success)

    uint8_t buffer[15]; // a 15 bytes unsigned int buffer


    // sensor attributes : 
    float dx, dy, dz; // translation to apply to the ROV (in ROV reference frame)
    float rx, ry, rz; // rotations to apply to the ROV (in ROV reference frame)

};

#endif