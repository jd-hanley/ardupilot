#include "AP_Centeye_Nano.h"

#include <utility>
#include <stdio.h>

#include "AP_Strain_Backend.h"
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

}