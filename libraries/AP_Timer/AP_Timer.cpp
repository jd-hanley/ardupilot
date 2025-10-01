#include "AP_Timer.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

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
 * Init sets up the I2C Device Manager for the specified address
 * Ensure all slave devices that need to receive the system time are addressed the same
 */
void AP_Timer::init(uint8_t sensor_address)
{
    _dev = hal.i2c_mgr->get_device(0, sensor_address);
}

/**
 * Convenient wrapper function for the I2C Device Manager's transfer function
 */
bool AP_Timer::write_bytes(uint8_t* bytes, uint8_t length)
{
    return _dev->transfer(bytes, length, NULL, 0);
}

/**
 * Broadcast functions simply send out the current system time via I2C
 */
bool AP_Timer::broadcast_millis()
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
            return false;
        }
        else
        {
            hal.console->printf("Successfully broadcast system time\n");
            return true;
        }
    }
    else
    {
        // Failed to obtain semaphore
        hal.console->printf("Failed to obtain semaphore\n");
    }
}

bool AP_Timer::broadcast_micros()
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
            return false;
        }
        else
        {
            hal.console->printf("Successfully broadcast system time\n");
            return true;
        }
    }   
    else
    {
        // Failed to get semaphore
        hal.console->printf("Failed to obtain semaphore\n");
    }
}

