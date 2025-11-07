#include "AP_Timer.h"
#include "AP_Timer_Backend.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <stdio.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/functor.h>

extern const AP_HAL::HAL& hal;

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
    uint8_t initial_address = 0x12;
    for (uint8_t i = 0; i < num_sensors; i++)
    {
        AP_Timer_Backend* temp = NEW_NOTHROW AP_Timer_Backend(initial_address++);
        drivers[i] = temp;
        drivers[i]->init();
    }

}

