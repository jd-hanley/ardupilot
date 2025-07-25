#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

class AP_Centeye_Nano_Backend
{
    public:
    // Constructor
    AP_Centeye_Nano_Backend();

    // Method to obtain the correct the UART driver
    void init_serial(uint8_t serial_instance);

    // Method to read all data in to the sensor
    // No parameters at the moment, but will take in a pointer to a sensor struct
    void read_all();

    private:

    // Our actual instance of the UART driver
    AP_HAL::UARTDriver *uart = nullptr;

    // Wrapper function that takes in a byte corresponding to a command and sends the byte stream to the sensor
    void write(uint8_t cmd);

    // Wrapper function that takes in a buffer and stores the bytes coming in
    void read(uint8_t *buffer);

};