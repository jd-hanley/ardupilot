#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#define NUM_CAMERAS 2

extern const AP_HAL::HAL& hal;

class AP_Timer_Backend;

class AP_Timer
{
    /**
     * AP_Timer class is a convenient wrapper for timesyncing devices
     */
    public:
    
    // Constructor
    AP_Timer();

    // Do not allow copies 
    CLASS_NO_COPY(AP_Timer);

    // Initialize the timer object
    void init(uint8_t initial_address);
    
    // Helper functions for broadcasting the system time
    void broadcast_millis(AP_HAL::I2CDevice* dev);
    void broadcast_micros(AP_HAL::I2CDevice* dev);

    private:

    // Write bytes helper function
    bool write_bytes(AP_HAL::I2CDevice* dev, uint8_t* bytes, uint8_t length);
    void callbackFunction();

    // Smart pointers to I2C Device Managers
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev_1;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev_2;
};