#include "AP_CHAD_config.h"
#include <GCS_MAVLink/GCS.h>

#if AP_CHAD_ENABLED

#include "AP_CHAD.h"

#include <netinet/in.h> // defines socket(), SOCK_DGRAM, PF_INET
#include <sys/socket.h> // defines AF_INET, sockaddr_in, recvfrom()
#include <arpa/inet.h> // defines inet_pton(), htons()

#include <sstream>
#include <iostream>

void AP_CHAD::init()
{   
    socket_fd = socket(PF_INET, SOCK_DGRAM, 0); // Protocol Format : INET (IPv4), protocol : SOCK_DGRM (UDP), 0 (default protocol)

    // clear structure
    station_addr = {};
    station_addr_len = sizeof(station_addr);
    local_addr = {};

    // set local address configuration
    local_addr.sin_family = AF_INET; // Adress Family : INET (IPv4)
    int ip_set = inet_pton(AF_INET, "0.0.0.0", &(local_addr.sin_addr)); // set IP address (0.0.0.0 = listen to all interfaces)
                                                           // Returns 1 == success, 0 == invalid address, -1 == error
    if (ip_set > 0) {
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor : ip set");
        std::cout << "CHAD sensor : ip set \n";
    } else {
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor error : ip not set");
        std::cout << "CHAD sensor : ip not set \n";
    }
    local_addr.sin_port = htons(1106); // set listening port to 1106
    local_addr_len = sizeof(local_addr);

    // initialize connection
    int binded = bind(socket_fd, reinterpret_cast<sockaddr*>(&local_addr), local_addr_len);

    if (binded == 0) {
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor initialized : bound");
    std::cout << "CHAD sensor initialization : bound \n";
    } else {
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor not initialized, bind error");
        std::cout << "CHAD sensor initialization : bind error \n";
    }
}

void AP_CHAD::read()
{
    std::cout << "CHAD sensor : trying to read \n";
    int nbytes = recvfrom(socket_fd, buffer, sizeof(buffer), MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&station_addr), &station_addr_len); // fills buffer with incoming data
        // if > 0 : number of bytes received, if == 0 : connection closed, if < 0 : error / no data
    
    if (nbytes < 0) 
    { 
        std::cout << "CHAD sensor : error / no data" << nbytes << " \n";
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor : error / no data");
        return; 
    }
    if (nbytes == 0)
    {
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor : connection closed");
        std::cout << "CHAD sensor : connection closed \n";
        return;
    } 
    else 
    {   
        std::ostringstream flux;
        flux << "CHAD sensor : received ";
        flux << nbytes;
        flux << "    bytes. Contains : ";
        flux << buffer;
        flux << std::endl;
        const char *msg = flux.str().c_str();
        std::cout << msg << std::endl;
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s", msg);
    }


}

#endif