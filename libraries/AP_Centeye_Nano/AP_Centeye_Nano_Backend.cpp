#include "AP_Centeye_Nano_Backend.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

// Constructor implementation
AP_Centeye_Nano_Backend::AP_Centeye_Nano_Backend(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, AP_Centeye_Nano* singleton, uint8_t id) :
    _front_end(singleton),
    _dev(std::move(dev)),
    sensor_id(id) {}

// Write byte implementation: essentially copied from strain driver
bool AP_Centeye_Nano_Backend::write_byte(uint8_t byte)
{
    uint8_t msg = byte;
    return _dev->transfer(&msg, sizeof(msg), NULL, 0);
}

bool AP_Centeye_Nano_Backend::write_bytes(uint8_t* bytes, uint8_t length)
{
    return _dev->transfer(bytes, length, NULL, 0);
}

bool AP_Centeye_Nano_Backend::init()
{
    // hal.console->printf("Inside the backend init function\n");
    // Call timer at 60 hz
    if (_dev->register_periodic_callback(16700, FUNCTOR_BIND_MEMBER(&AP_Centeye_Nano_Backend::timer, void)))
    {
        // hal.console->printf("Successfully registered function\n");
    }
    else
    {
        // hal.console->printf("Failed to register function\n");
    }

    // If there is any additional calibration we want to do, it should go here
    return true;
}

void AP_Centeye_Nano_Backend::timer()
{
    // This is the function that will be called at some set frequency (60 hz)
    // This function should obtain the semaphore, write the correct bytes to the sensor to request data, and update the unsafe data structure, then release the semaphore
    // Note that this method will need to be revisited in the future to add robust status checks
    // For now, here is the bare bones implementation

    // uint32_t t0 = AP_HAL::micros();
    // hal.console->printf("Inside timer\n");
    get_data();
    // uint32_t dur = AP_HAL::micros() - t0;
    // hal.console->printf("Duration: %ld\n", dur);
}

bool AP_Centeye_Nano_Backend::get_data()
{  
    uint32_t t0 = AP_HAL::micros(); 
    if (!read_odom())
    {
        // Error handling goes here for failure to read odometry
        // hal.console->printf("Failed to read odometry data\n");
    }
    else
    {
        // hal.console->printf("Successfully read odometry data\n");
    }
    uint32_t dur = AP_HAL::micros() - t0;
    hal.console->printf("Duration: %ld\n", dur);
    t0 = AP_HAL::micros(); 
    if (!read_objdet_h())
    {
        // Error handling goes here for failure to read objdet
    }
    else
    {
        // hal.console->printf("Successfully read objdet_h data\n");
    }
    dur = AP_HAL::micros() - t0;
    hal.console->printf("Duration: %ld\n", dur);
    // // t0 = AP_HAL::micros(); 
    // if (!read_objdet_v())
    // {
    //     // Error handling goes here for failure to read objdet
    // }
    // dur = AP_HAL::micros() - t0;
    // hal.console->printf("Duration: %ld\n", dur);
    return true;

}

bool AP_Centeye_Nano_Backend::read_odom()
{
    // Read odom details:
    // Send the following bytes to the sensor:
    //      ATT DS ONLY (255 or 0xFF)
    //      DS ID (11 or 0xB)
    // For odometry, we should receive 12 bytes containing x, y, and div odometries in 4-byte little endian format
    // So, declare a buffer of length 12 bytes and call read
    // Lastly, process the resulting data and store in the unsafe data structure
    // With little endian, the first byte in a given 4 byte sequence will actually be the last byte
    // So we bit shift the 4th byte left by 24 and | that with the 3rd byte bit shifted left by 16, etc
    // Here is the implementation:

    // Obtain the semaphore
    bool has_sem = _dev->get_semaphore()->take(20);
    if (has_sem)
    {
        uint8_t command[] = {dtt_ds_only, odo_ds_id};
        if (!write_bytes(command, 2))
        {
            // Error handling goes here...
            // hal.console->printf("Write failed\n");
        }
        uint8_t buffer[ODOM_BYTES];
        int32_t old_odom_x = unsafe_data.odom_x;
        int32_t old_odom_y = unsafe_data.odom_y;
        int32_t old_odom_div = unsafe_data.odom_div;
        uint32_t current_time = AP_HAL::millis();
        // Read into the buffer
        if (!_dev->read(buffer, 12))
        {
            // Error handling goes here... 
            // hal.console->printf("Read failed\n");
        }

        // With the data now in the buffer, we can bit shift into the proper form
        unsafe_data.odom_x = (uint32_t) buffer[3] << 24 | (uint32_t) buffer[2] << 16 | (uint32_t) buffer[1] << 8 | (uint32_t) buffer[0];
        unsafe_data.odom_y = (uint32_t) buffer[7] << 24 | (uint32_t) buffer[6] << 16 | (uint32_t) buffer[5] << 8 | (uint32_t) buffer[4];
        unsafe_data.odom_div = (uint32_t) buffer[11] << 24 | (uint32_t) buffer[10] << 16 | (uint32_t) buffer[9] << 8 | (uint32_t) buffer[8];
        // Calculate the time step in seconds
        // hal.console->printf("%ld\n",unsafe_data.odom_x);
        // float dt = ((float) current_time - (float) unsafe_data.meas_time) / 1000.0;
        unsafe_data.meas_time = current_time;
        unsafe_data.flow_x = (((float) unsafe_data.odom_x - (float) old_odom_x)) / 1000;
        unsafe_data.flow_y = (((float) unsafe_data.odom_y - (float) old_odom_y)) / 1000;
        unsafe_data.flow_div = (((float) unsafe_data.odom_div - (float) old_odom_div)) / 1000;
        _dev->get_semaphore()->give();
    }
    else
    {
        // Error handling for failure to obtain semaphore goes here
        hal.console->printf("Failed to get semaphore in odometry reading\n");
    } 
    return true;
}

bool AP_Centeye_Nano_Backend::read_objdet_h()
{
    // Read objdet details
    // Send the following bytes to the sensor:
    //      ATT DS ONLY
    //      DS ID (12, then 13 OR 0xC then 0xD)
    // We expect to receive 64 total bytes, representing a 4x4 matrix of 4-byte integers in little endian format
    // Given the layout of 2D arrays in memory, we can do everything simply using pointer arithmetic
    // Here is the implementation

    // get the semaphore
    bool has_sem = _dev->get_semaphore()->take(20);

    if (has_sem)
    {
        uint8_t command[] = {dtt_ds_only, objdet_h_ds_id};
        if (!write_bytes(command, 2))
        {
            // Error handling goes here...
        }
        uint8_t buffer[OBJDET_BYTES];
        // Read into the buffer
        if (!_dev->read(buffer, 64))
        {
            // Error handling goes here... 
        }

        int32_t* ptr = unsafe_data.objdet_h;
        for (uint8_t i = 0; i < 64; i += 4)
        {
            *ptr = (uint32_t) buffer[i + 3] << 24 | (uint32_t) buffer[i + 2] << 16 | (uint32_t) buffer[i + 1] << 8 | (uint32_t) buffer[i];
            ptr++;
        }
        _dev->get_semaphore()->give();

    }
    else
    {
        hal.console->printf("Failed to get semaphore in objdet reading\n");
    }
    return true;
}

bool AP_Centeye_Nano_Backend::read_objdet_v()
{
    // Read objdet details
    // Send the following bytes to the sensor:
    //      ATT DS ONLY
    //      DS ID (12, then 13 OR 0xC then 0xD)
    // We expect to receive 64 total bytes, representing a 4x4 matrix of 4-byte integers in little endian format
    // Given the layout of 2D arrays in memory, we can do everything simply using pointer arithmetic
    // Here is the implementation

    // get the semaphore
    bool has_sem = _dev->get_semaphore()->take(20);

    if (has_sem)
    {
        uint8_t command[] = {dtt_ds_only, objdet_v_ds_id};
        if (!write_bytes(command, 2))
        {
            // Error handling goes here...
        }
        uint8_t buffer[OBJDET_BYTES];
        // Read into the buffer
        if (!_dev->read(buffer, 64))
        {
            // Error handling goes here... 
        }

        int32_t* ptr = unsafe_data.objdet_v;
        for (uint8_t i = 0; i < 64; i += 4)
        {
            *ptr = (uint32_t) buffer[i + 3] << 24 | (uint32_t) buffer[i + 2] << 16 | (uint32_t) buffer[i + 1] << 8 | (uint32_t) buffer[i];
            ptr++;
        }
        _dev->get_semaphore()->give();
    }
    return true;
}


bool AP_Centeye_Nano_Backend::copy_to_front_end()
{
    // Copy details:
    // Obtain the semaphore, copy all data to the front end structure
    bool has_sem = _dev->get_semaphore()->take(20);
    if (has_sem)
    {
        // Assume we have the semaphore and lets copy over all of the data
        _front_end->sensors[sensor_id].odom_x = unsafe_data.odom_x;
        _front_end->sensors[sensor_id].odom_y = unsafe_data.odom_y;
        _front_end->sensors[sensor_id].odom_div = unsafe_data.odom_div;
        _front_end->sensors[sensor_id].flow_x = unsafe_data.flow_x;
        _front_end->sensors[sensor_id].flow_y = unsafe_data.flow_y;
        _front_end->sensors[sensor_id].flow_div = unsafe_data.flow_div;

        for (uint8_t i = 0; i < 16; i++)
        {
            _front_end->sensors[sensor_id].objdet_h[i] = unsafe_data.objdet_h[i];
            // _front_end->sensors[sensor_id].objdet_v[i] = unsafe_data.objdet_v[i];
        }
        _dev->get_semaphore()->give();
    }
    else
    {
        // Error handling for failure to obtain semaphore goes here
    }
    return true;
}