#include "AP_CHAD_config.h"
#include <GCS_MAVLink/GCS.h>

#if AP_CHAD_ENABLED

#include "AP_CHAD.h"

#include <sstream>
#include <iostream>
using namespace std;



void AP_CHAD::init()
{
    // *socket = new SocketAPM(true); // true = UDP/datagram

    // if (!socket) {
    //     // handle allocation issue
    //     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor : socket allocation issue");
    //     return;
    // }

    //  if (!socket -> bind("0.0.0.0", 1106)) {
    //      // handle bind failure
    //      delete socket;
    //      GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor : socket bind failed");
    //      return;
    //  }

    // socket -> set_blocking(false); //continuous operation
}

void AP_CHAD::read()
{
    // ssize_t len = socket -> recv(buffer, sizeof(buffer), 100); // timeout : 100 ms
    // if (len > 0) {
    //     if (len == sizeof(buffer)) {
            // all data received, process

            // Sx = *reinterpret_cast<const float *>(part1);
            // Sy = *reinterpret_cast<const float *>(part2);
            // Sz = *reinterpret_cast<const float *>(part3);
            // char cPart1 [4] = *reinterpret_cast<>;
            // Sx = *reinterpret_cast<const float *>(&cPart1[0]);

            // sstream flux;
            // cout << "CHAD sensor : received ";
            // cout << *reinterpret_cast<const float *>(part1) << ", " << *reinterpret_cast<const float *>(part2) << ", " << *reinterpret_cast<const float *>(part3) << endl;
            // msg = sstream.str().c_str();

            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, ("%s", msg));
    //      }
    //      else 
    //      {
    //          GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor : data incomplete");   
    //      }
    //  } else {
    //      GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHAD sensor : waiting");
    //  }
}

#endif