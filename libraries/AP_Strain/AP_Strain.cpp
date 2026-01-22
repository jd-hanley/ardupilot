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
        sensors[i].status = Status::Bad;
        sensors[i].I2C_id = 0x9 + 7 * i; // 0x09, 0x10
        sensors[i].last_status_check_ms = 0;
        sensors[i].update_count = 0;
        sensors[i].avg_refresh_rate_hz = 0.0f;

        // Initialize previous data to zero
        for (uint8_t j = 0; j < STRAIN_SENSORS; j++) {
            sensors[i].prev_data[j] = 0;
        }

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

float AP_Strain::get_roll_accel_strain()
{
    // ensure accel_strain is updated from latest sensor readings
    update_strain_accel();
    // return the updated struct (copies two floats)
    return accel_strain.roll_accel;
}

float AP_Strain::get_pitch_accel_strain()
{
    // ensure accel_strain is updated from latest sensor readings
    update_strain_accel();
    // return the updated struct (copies two floats)
    return accel_strain.pitch_accel;
}

void AP_Strain::get_arm_averages(float* destination)
{
    destination[0] = (float)(sensors[0].data[0] ); // back arm bending
    destination[1] = (float)(sensors[0].data[4] ); // left arm bending
    destination[2] = (float)(sensors[1].data[0] ); // front arm bending
    destination[3] = (float)(sensors[1].data[4] ); // right arm bending     
}

float AP_Strain::apply_strain_weights(const float *weights)
{
    // currently setup for TDA V1 round sensor arms with 3 nodes per arm but records 4
    // data from sensor[3] & sensor[7] is garbage and not used in calculations

    int32_t* strain_data1 = sensors[0].data;
    int32_t* strain_data2 = sensors[1].data;

    // Combine strain data from both sensors into a single array
    // 0,1,2 from sensor 0 is the back arm 
    // 4,5,6 from sensor 0 is the left arm 
    // 0,1,2 from sensor 1 is the front arm 
    // 4,5,6 from sensor 1 is the right arm 
    int32_t total_strain_data[12] = {strain_data1[0],
                                     strain_data1[1],
                                     strain_data1[2],
                                     strain_data1[4],
                                     strain_data1[5],
                                     strain_data1[6],
                                     strain_data2[0],
                                     strain_data2[1],
                                     strain_data2[2],
                                     strain_data2[4],
                                     strain_data2[5],
                                     strain_data2[6]
    };

    float angular_accel = 0.0f;
                                
    for (uint8_t i = 0; i < 12; i++)
    {   
        float strain_data = (float) total_strain_data[i] / strain_scale;

        angular_accel += weights[i] * strain_data;
    }

    return angular_accel;
}

void AP_Strain::update_strain_accel()
{
    accel_strain.roll_accel = apply_strain_weights(strain_accel_weights_roll) + strain_accel_intercept_roll;
    accel_strain.pitch_accel = apply_strain_weights(strain_accel_weights_pitch) + strain_accel_intercept_pitch;
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
    // Iterate through all sensors and return false if the status of any sensor is Bad
    for (uint8_t i = 0; i < STRAIN_MAX_INSTANCES; i++)
    {
        if (sensors[i].status == Status::Bad)
        {
            return false;
        }
    }
    return true;
}

void AP_Strain::update()
{
    const uint32_t now = AP_HAL::millis();
    const uint32_t status_check_interval_ms = 100;  // Check every 100ms (10Hz)
    const uint32_t min_refresh_interval_ms = 1000 / STRAIN_MIN_REFRESH_RATE_HZ;  // ~16.67ms for 60Hz

    for (uint8_t i = 0; i < STRAIN_MAX_INSTANCES; i++)
    {

        // Check if it's time for a status check (every 100ms)
        if ((now - sensors[i].last_status_check_ms) < status_check_interval_ms) {
            continue;
        }

        // Calculate average refresh rate based on update count since last check
        // Interval is 100ms = 0.1s, so Hz = count / 0.1 = count * 10
        sensors[i].avg_refresh_rate_hz = sensors[i].update_count * 10.0f;
        sensors[i].update_count = 0;  // Reset counter for next interval

        sensors[i].last_status_check_ms = now;

        // Check 1: Refresh rate
        // Calculate the time since the last update
        uint32_t time_since_update = now - sensors[i].last_update_ms;

        // If refresh rate is below minimum (time between updates > max allowed interval)
        if (time_since_update > min_refresh_interval_ms) {
            sensors[i].status = Status::Bad;
            continue;
        }

        // Check 2: Data staleness
        // Check if any data value has changed since the last status check
        bool data_changed = false;
        for (uint8_t j = 0; j < sensors[i].num_data; j++) {
            if (sensors[i].data[j] != sensors[i].prev_data[j]) 
            {
                data_changed = true;
                break;
            }
        }

        // Update previous data for next check
        for (uint8_t j = 0; j < sensors[i].num_data; j++) 
        {
            sensors[i].prev_data[j] = sensors[i].data[j];
        }

        // If data hasn't changed for more than one cycle, mark as bad
        if (!data_changed) 
        {
            sensors[i].status = Status::Bad;
            continue;
        }

        // If all checks passed and status was Bad, restore to Good
        if (sensors[i].status == Status::Bad) 
        {
            sensors[i].status = Status::Good;
        }
    }
}

// void AP_Strain::send_telemetry()
// {
//     gcs().send_message(MSG_STRAIN_DATA);
//     // uint32_t current_time = AP_HAL::millis();
//     // if ((current_time - old_time) > 100)
//     // {
//     //     gcs().send_message(MSG_STRAIN_DATA);
//     //     old_time = current_time;
//     // }
// }


namespace AP {

    AP_Strain &strain()
    {
        return *AP_Strain::get_singleton();
    }
    
    };

