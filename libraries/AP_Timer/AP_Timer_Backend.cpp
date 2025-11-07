#include "AP_Timer_Backend.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>


#define NUM_BYTES 4

// Constructor implementation
AP_Timer_Backend::AP_Timer_Backend(uint8_t _address)
{
    _dev = hal.i2c_mgr->get_device(0, _address);
    address = _address;
}

// Init implementation
void AP_Timer_Backend::init()
{
    // hal.scheduler->delay_microseconds(phase_us);
    // _dev->register_periodic_callback(base_us, FUNCTOR_BIND_MEMBER(&AP_Timer_Backend::broadcast_millis, void));
}

/* Convenient wrapper function for the I2C Device Manager's transfer function */
bool AP_Timer_Backend::write_bytes(uint8_t* bytes, uint8_t length)
{
    return _dev->transfer(bytes, length, NULL, 0);
}

/* Implementation of System Time Broadcast Functions */
void AP_Timer_Backend::broadcast_millis()
{
    // Grab the current system time
    uint32_t sysTime = AP_HAL::millis();
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
            hal.console->printf("Successfully broadcast system time to device %u\n", address);
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

void AP_Timer_Backend::broadcast_micros()
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