#include "AP_Centeye_Nano.h"

#include <utility>
#include <stdio.h>

#include "AP_Centeye_Nano_Backend.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/I2CDevice.h>

#include <AP_Arming/AP_Arming.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Vehicle/AP_Vehicle.h>

extern const AP_HAL::HAL& hal;

// Singleton instance
AP_Centeye_Nano *AP_Centeye_Nano::_singleton;

// Constructor implementation
AP_Centeye_Nano::AP_Centeye_Nano()
{
    _singleton = this;
}

void AP_Centeye_Nano::init()
{
    // A couple things to do here:
    // Obtain a smart pointer to I2C Device Manager for each of our sensors
    // Set the sensor status for each of our sensors
    // Set the I2C ID for each of our sensors
    // Create a new backend object for each of the sensors and add to the array of drivers
    // Call init for each of the backend objects

    // TODO:
    //      - Loop through and set all sensor data members to default values
    //      - Obtain a OwnPtr to I2CDevice object
    //      - Use the OwnPtr and the current entry in the sensor array to dynamically allocate and construct a new backend object
    //      - Call init on the new backend object 
    for (uint8_t i = 0; i < OFLOW_MAX_INSTANCES; i++)
    {
        // Address devices as 0x10, 0x11, 0x12, 0x13
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_temp = hal.i2c_mgr->get_device(0, 0x10 + i);
        AP_Centeye_Nano_Backend* backend_temp = NEW_NOTHROW AP_Centeye_Nano_Backend(std::move(dev_temp), _singleton, i);
        drivers[i] = backend_temp;
        if (drivers[i]->init())
        {
            // We have successfully created and initialized our backend... could increment a sensor counter here
        }
    }
}

bool AP_Centeye_Nano::update_from_backend()
{
    // For each backend object, call the copy function
    for (uint8_t i = 0; i < OFLOW_MAX_INSTANCES; i++)
    {
        drivers[i]->copy_to_front_end();
    }
    return true;
}

int32_t AP_Centeye_Nano::get_odo_x()
{
    update_from_backend();
    return sensors[0].odom_x;
}