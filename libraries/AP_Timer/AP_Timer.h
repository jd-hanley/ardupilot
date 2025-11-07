#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#define NUM_SENSORS 2

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

    private:
    AP_Timer_Backend *drivers[NUM_SENSORS];
};