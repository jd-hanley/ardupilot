#include "AP_Strain_Backend.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>

#define TIMEOUT 100

extern const AP_HAL::HAL& hal;

// Constructor - 
AP_Strain_Backend::AP_Strain_Backend(AP_Strain::sensor &_strain_arm, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, AP_Strain* singleton) : 
    _frontEnd(singleton),
    _sensor(_strain_arm),
    _dev(std::move(dev)) {}

// private

bool AP_Strain_Backend::write_byte(uint8_t write_byte)
{
    uint8_t msg = write_byte;
    return _dev->transfer(&msg, sizeof(msg), NULL, 0);
}


bool AP_Strain_Backend::init()
{
    DEV_PRINTF("I2C starting\n");

    // Call timer() at 100Hz
    _dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_Strain_Backend::timer, void));


    if (!calibrate()) {
        return false;
    }

    return true;
}

void AP_Strain_Backend::timer(void)
{
    // Boolean read will be used throughout the function
    bool read = false;

    // Store the old data
    // Joe - Eventually we might want to check data from all strain gauges before arming
    // For now, we will assume either all sensors are working or none of them are working
    int32_t last_data = _sensor.data[0];
    int32_t last_time = _sensor.last_update_ms;

    // Get the semaphore
    bool has_sem = _dev->get_semaphore()->take(50);
    if (has_sem)
    {
        // If we have the semaphore, call get_reading and store the return value in read
        read = get_reading();
        _dev->get_semaphore()->give();
    }
    // Key note here: do not worry about the case where we fail to get semaphore... setting read to false by default will handle this case
    // If read returned true, it means we successfully read in data (possibly bad data, but that will be handled later)
    // Set the status
    if (read)
    {
        _sensor.status = AP_Strain::Status::Good;
    }
    // If read returned false, either our writing poll to sensor or reading data from sensor failed (or potentially we did not get the semaphore)
    else
    {
        _sensor.status = AP_Strain::Status::NotConnected;
    }

    // If the sensor data did not change, we need to call update_last_change_ms with the reset argument as false to extend the last change time
    if (_sensor.data[0] == last_data)
    {
        update_last_change_ms(false , last_time);
    }
    // Otherwise, reset the last change time to zero by calling update_last_change_ms with the reset argument as true
    else
    {
        update_last_change_ms(true , last_time);
    }

    // Final check: if last change time is greater than timeout, we need to reset the arm and calibrate all sensors
    if (_sensor.last_change_ms > TIMEOUT)
    {
        // Reset method will send 'R' to this particular arm ONLY
        reset();
        // Calling front end method will calibrate ALL strain arms
        _frontEnd->calibrate_all();
        _sensor.status = AP_Strain::Status::NoData;    
    }
}

void AP_Strain_Backend::update_last_change_ms(bool reset, int32_t last_time)
{
    // If the boolean argument is true, reset the last change time to 0
    if (reset)
        _sensor.last_change_ms = 0;
    // Otherwise, increment last change time by the amount of time that has passed
    else
        _sensor.last_change_ms += AP_HAL::millis() - last_time;
}

// get_reading: return true if we successfully read in data from the sensor, false otherwise
// Note that the semaphore has already been obtained by the caller
bool AP_Strain_Backend::get_reading()
{
    // Create the buffer to store the data
    uint8_t buffer[_sensor.num_data*4];

    // *** not nessary to write "P" to the sensor currently ***
    // Write "P" to sensor
    // if (!write_byte(0x50)) 
    // {
    //     // Writing to sensor failed
    //     // Return false... dealt with in timer()
    //     return false;
    // }

    // Read bytes into the buffer
    if (!_dev->read(buffer, sizeof(buffer)))
    {
        // Reading from sensor failed
        // Return false... dealt with in timer()
        return false;
    }

    // Buffer now stores all 40 bytes
    // Iterate the bytes 4 at a time and shift to form 32 bit signed integers
    for (uint8_t i = 0; i < _sensor.num_data; i++)
    {
        // For each sensor, combine the four bytes to form a 32 bit signed integer (assuming little endian)
        _sensor.data[i] = (int32_t) buffer[i*4]  |
                 ((int32_t) buffer[i*4+1] << 8)  | 
                 ((int32_t) buffer[i*4+2] << 16) | 
                 ((int32_t) buffer[i*4+3] << 24);
    }
    correct_missing_sensor(); // fixes missing value on sensor 10
    _sensor.last_update_ms = AP_HAL::millis();

    return true;
}

// set status and update valid count
void AP_Strain_Backend::set_status(AP_Strain::sensor &_strain_arg, AP_Strain::Status status)
{
    _strain_arg.status = status;
}

// true if sensor is returning data
bool AP_Strain_Backend::has_data() const {
    return ((_sensor.status != AP_Strain::Status::NotConnected) &&
            (_sensor.status != AP_Strain::Status::NoData));
}

bool AP_Strain_Backend::calibrate()
{
    // Objective: send the byte 'Z' to the sensor
    bool has_sem = _dev->get_semaphore()->take(20);
    if (has_sem)
    {
        if (!write_byte(0x5A)) 
        {
            // Writing to sensor failed
            _dev->get_semaphore()->give(); 
            return false;
        }
        else
        {
            // Writing to sensor successful
            _dev->get_semaphore()->give(); 
            return true;
        }   
    }
    else
    {
        // Failed to get semaphore
        return false;
    }
}

bool AP_Strain_Backend::reset()
{
    // Objective send the byte 'R' to the sensor
    bool has_sem = _dev->get_semaphore()->take(50);
    if (has_sem)
    {
        if (!write_byte(0x59))
        {
            // Writing to sensor failed
            _dev->get_semaphore()->give();  
            return false;
        }      
        else
        {
            // Writing to sensor successful
            _dev->get_semaphore()->give();
            return true;
        }
    }
    else
    {
        // Error getting semaphore
        return false;
    }

}


void AP_Strain_Backend::correct_missing_sensor()
{   
    // hacky fix for missing sensor 10
    if (_sensor.data[10] == 0 && !_sensor.data[11] == 0)
    {
        _sensor.data[10] = _sensor.data[9];
        _sensor.data[4] = _sensor.data[3];
        _sensor.data[5] = _sensor.data[3];
    }
}

