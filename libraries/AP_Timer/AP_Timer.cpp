#include "AP_Timer.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <stdio.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/functor.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

#define NUM_BYTES 4

/**
 * AP_Timer constructor
 */
AP_Timer::AP_Timer()
{
    // As of now, the constructor does nothing
}

/**
 * Init sets up the I2C Device Managers for the specified address
 * Ensure all slave devices that need to receive the system time are addressed the same
 */
void AP_Timer::init(uint8_t initial_address)
{

    // New init function to include additional backend structure
    // Loop through the number of sensors and create the backend object for each
    // Add the created backend object to the array of backend objects
    // uint32_t base_us = 16700;           // ~60 Hz

    _dev = hal.i2c_mgr->get_device(0, initial_address, 400000);


    // Register the periodic callback on one of the I2C Device Managers
    _dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_Timer::broadcast_millis, void));
}

void AP_Timer::broadcast_millis()
{
    // Grab the current system time
    uint32_t sysTime = AP_HAL::millis();
    hal.console->printf("%d\n", sysTime);
    // Break the system time into bytes to be transferred in big endian order
    uint8_t bytes[NUM_BYTES];
    bool has_sem = _dev->get_semaphore()->take(20);
    if (has_sem)
    {
        for (uint8_t i = 0; i < NUM_BYTES; i++)
        {
            // Shift right an appropriate amount and mask
            bytes[i] = (sysTime >> ((NUM_BYTES - i - 1) * 8)) & 0xFF;
        }
        if (!write_bytes(bytes, NUM_BYTES))
        {
            // Writing bytes failed for some reason, needs to be handled here.
            hal.console->printf("Failed to broadcast system time\n");
            // return false;
        }
        else
        {
            hal.console->printf("Successfully broadcast system time\n");
            // return true;
        }
        _dev->get_semaphore()->give();
    }
    else
    {
        // Failed to obtain semaphore
        hal.console->printf("Failed to obtain semaphore\n");
        // return false;
    }
}

void AP_Timer::broadcast_micros()
{
    // Grab the current system time
    uint32_t sysTime = AP_HAL::micros();
    // Break the system time into bytes to be transferred in big endian order
    uint8_t bytes[NUM_BYTES];
    bool has_sem = _dev->get_semaphore()->take(20);
    if (has_sem)
    {
        for (uint8_t i = 0; i < NUM_BYTES; i++)
        {
            // Shift right an appropriate amount and mask
            bytes[i] = (sysTime >> ((NUM_BYTES - i - 1) * 8)) & 0xFF;
        }
        if (!write_bytes(bytes, NUM_BYTES))
        {
            // Writing bytes failed for some reason, needs to be handled here.
            hal.console->printf("Failed to broadcast system time\n");
            // return false;
        }
        else
        {
            hal.console->printf("Successfully broadcast system time\n");
            // return true;
        }
        _dev->get_semaphore()->give();
    }   
    else
    {
        // Failed to get semaphore
        hal.console->printf("Failed to obtain semaphore\n");
        // return false;
    }
}

bool AP_Timer::write_bytes(uint8_t* bytes, uint8_t length)
{
    return _dev->transfer(bytes, length, NULL, 0);
}
