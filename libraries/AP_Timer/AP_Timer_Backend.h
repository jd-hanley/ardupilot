#pragma once

#include "AP_Timer.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/I2CDevice.h>

class AP_Timer_Backend
{
public:
    // Constructor takes in the address and sets up the smart pointer
    AP_Timer_Backend(uint8_t address);

    // Init function sets up the callback
    void init();

    // Helper functions for broadcasting the system time
    void broadcast_millis();
    void broadcast_micros();

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    bool write_bytes(uint8_t* bytes, uint8_t length);
    uint8_t address;


};