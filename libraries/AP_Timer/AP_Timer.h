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
    void start_broadcast(uint32_t period_us = 16700);

    private:
    AP_Timer_Backend *drivers[NUM_CAMERAS];
    void tick();
    static void tick_proc();
    uint32_t period_ms_ = 17;
    uint8_t num = 0;
    static AP_Timer* instance_;  
};