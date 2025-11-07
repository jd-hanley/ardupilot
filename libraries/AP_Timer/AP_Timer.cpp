#include "AP_Timer.h"
#include "AP_Timer_Backend.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <stdio.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/functor.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

AP_Timer* AP_Timer::instance_ = nullptr;

/**
 * AP_Timer constructor
 */
AP_Timer::AP_Timer()
{
    // As of now, the constructor does nothing
}

/**
 * Init sets up the I2C Device Manager for the specified address
 * Ensure all slave devices that need to receive the system time are addressed the same
 */
void AP_Timer::init(uint8_t num_sensors)
{

    // New init function to include additional backend structure
    // Loop through the number of sensors and create the backend object for each
    // Add the created backend object to the array of backend objects
    instance_ = this;
    uint8_t initial_address = 0x12;
    uint32_t base_us = 16700;           // ~60 Hz
    for (uint8_t i = 0; i < num_sensors; i++)
    {
        AP_Timer_Backend* temp = NEW_NOTHROW AP_Timer_Backend(initial_address++);
        drivers[i] = temp;
    }
}

void AP_Timer::start_broadcast(uint32_t period_us)
{
    period_us_ = period_us;
    // Schedule first run
    hal.scheduler->register_delay_callback(&AP_Timer::tick_proc, period_us_);
}

void AP_Timer::tick_proc()
{
    if (instance_)
    {
        instance_->tick();
        hal.scheduler->register_delay_callback(&AP_Timer::tick_proc, instance_->period_us_);
    }
}

void AP_Timer::tick()
{
    for (uint8_t i = 0; i < NUM_CAMERAS; i++)
    {
        drivers[i]->broadcast_millis();
    }
}
