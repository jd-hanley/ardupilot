#include "AP_Strain.h"

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

// singleton instance
AP_Strain *AP_Strain::_singleton;

/*
  AP_Strain constructor
 */
AP_Strain::AP_Strain()
{
    _singleton = this;
}

// initialise the strain object, loading backend drivers
void AP_Strain::init(void)
{   
    if (_num_sensors != 0) {
        // don't re-init if we've found some sensors already
        return;
    }

    // TODO:
    //      - Loop through and set all sensor data members to default values
    //      - Obtain a OwnPtr to I2CDevice object
    //      - Use the OwnPtr and the current entry in the sensor array to dynamically allocate and construct a new backend object
    //      - Call init on the new backend object 
    for (uint8_t i = 0; i < STRAIN_MAX_INSTANCES; i++)
    {
        sensors[i].status = Status::NotConnected;
        sensors[i].I2C_id = 0x9 + 7 * i; // 0x09, 0x10
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_temp = hal.i2c_mgr->get_device(0, sensors[i].I2C_id);
        AP_Strain_Backend* backend_temp = NEW_NOTHROW AP_Strain_Backend(sensors[i], std::move(dev_temp), _singleton);
        drivers[i] = backend_temp;
        
        if (drivers[i]->init())
        {
            _num_sensors++;
        }
        // hal.scheduler->delay(100);
    }

    num_cal = 0;
    
    init_done = true;
    // AP_HAL::panic("AP_Strain::init() not implemented");
}

int32_t* AP_Strain::get_data(uint8_t instance)
{

    return sensors[instance].data;

}

float AP_Strain::get_avg_data()
{
    float sum = 0.0;
    uint8_t i;
    uint8_t j;
    int32_t* strain_data;
    for (i = 0; i < STRAIN_MAX_INSTANCES; i++)
    {
        strain_data = sensors[i].data;
        for(j = 0; j < STRAIN_SENSORS; j++)
        {
            sum += strain_data[j];
        }
    }
    return sum/(STRAIN_SENSORS * STRAIN_MAX_INSTANCES);
}

float AP_Strain::get_scaled_weighted_avg_data()
{
    // TODO: Implement scaled weighted average data
    return 0;
}

float AP_Strain::get_scaled_avg_data()
{
    return get_avg_data() / SENSOR_SCALE_FACTOR;
}

uint8_t AP_Strain::get_num_sensors()
{
    return _num_sensors;
}

AP_Strain::Status AP_Strain::get_status(uint8_t instance)
{
    return sensors[instance].status;
}

uint32_t AP_Strain::get_last_update(uint8_t instance)
{
    return sensors[instance].last_update_ms;
}

bool AP_Strain::calibrate_all()
{
    for (uint8_t i = 0; i < STRAIN_MAX_INSTANCES; i++)
    {
        if (!drivers[i]->calibrate())
        {
            return false;
        }
    }
    num_cal++;
    return true;
}

bool AP_Strain::reset_all()
{
    for (uint8_t i = 0; i < STRAIN_MAX_INSTANCES; i++)
    {
        if(!drivers[i]->reset())
        {
            return false;
        }
    }
    return true;
}

bool AP_Strain::get_status_all()
{
    // Iterate through all sensors and return false if the status of any sensor is NotConnected or Bad
    for (uint8_t i = 0; i < STRAIN_MAX_INSTANCES; i++)
    {
        if (sensors[i].status == Status::NoData || sensors[i].status == Status::NotConnected)
        {
            return false;
        }
    }
    return true;
}



namespace AP {

    AP_Strain &strain()
    {
        return *AP_Strain::get_singleton();
    }
    
    };

