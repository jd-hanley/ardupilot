#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

extern const AP_HAL::HAL& hal;

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
    void init(uint8_t sensor_address);

    // Broadcast system time
    void broadcast_millis();
    bool broadcast_micros();

    private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    bool write_bytes(uint8_t* bytes, uint8_t length);
    uint8_t address = 0;

};