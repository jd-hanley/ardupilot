#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

class AP_Centeye_Nano_Backend
{
    public:
    // Constructor
    AP_Centeye_Nano_Backend();

    // Method to obtain the correct the UART driver
    // Note that we will not be using any detect functionality
    // Simply call hal.serial(serial_instance) with the known serial instance
    void init_serial(uint8_t serial_instance);

    // Method to read all data in to the sensor
    // Proposed structure: read_all will take in a pointer to a sensor struct. It will then call read() with the command corresponding to
    // the data orientations we desire. For example, it may call read() with the command corresponding to h optical flow along with the 
    // pointer delineating the start of that buffer in the sensor struct. Then it will call again with the v vertical flow command, etc
    void read_all();

    private:

    // Our actual instance of the UART driver
    AP_HAL::UARTDriver *uart = nullptr;

    // Wrapper function that takes in a byte corresponding to a command and sends the byte stream to the sensor
    void write(uint8_t cmd);

    // Wrapper function that takes in a buffer and stores the bytes coming in
    // Function should take in the buffer and the command. It should then call write with the command and potentially store the returned data
    // directly in the buffer. An auxiliary data structure may be needed to temporarily store the data
    void read(uint8_t *buffer);

};