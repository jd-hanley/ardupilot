#include "AP_Centeye_Nano_Backend.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

// Constructor implementation
AP_Centeye_Nano_Backend::AP_Centeye_Nano_Backend(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, AP_Centeye_Nano* singleton) :
    _front_end(singleton),
    _dev(std::move(dev)) {}

// Write byte implementation: essentially copied from strain driver
bool AP_Centeye_Nano_Backend::write_byte(uint8_t byte)
{
    uint8_t msg = byte;
    return _dev->transfer(&msg, sizeof(msg), NULL, 0);
}

bool AP_Centeye_Nano_Backend::init()
{
    // Call timer at 60 hz
    _dev->register_periodic_callback(16670, FUNCTOR_BIND_MEMBER(&AP_Centeye_Nano_Backend::timer, void));

    // If there is any additional calibration we want to do, it should go here
    return true;
}

void AP_Centeye_Nano_Backend::timer()
{
    // This is the method that will be called at a regular interval
    // Within this function we need to write out the byte stream to request the data we want
    // We need to read in the data that is returned and store it in the front end's sensor data structure. 

    // Here is how I am imagining this working: 
}

