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
    // Call timer at 60 hz
    _dev->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&AP_Centeye_Nano_Backend::timer, void));

    // If there is any additional calibration we want to do, it should go here
    return true;
}

void AP_Centeye_Nano_Backend::timer()
{
    // This is the function that will be called at some set frequency (60 hz)
    // This function should obtain the semaphore, write the correct bytes to the sensor to request data, and update the unsafe data structure, then release the semaphore
    // Note that this method will need to be revisited in the future to add robust status checks
    // For now, here is the bare bones implementation

    // Obtain the semaphore
    // bool has_sem = _dev->get_semaphore()->take(50);
    // if (has_sem)
    // {
    //     // Read data into the unsafe backend data structure
    //     // Maintain a boolean variable for optional later use
    //     // bool read = get_data();
    //     get_data();
    //     // Give up the semaphore
    //     _dev->get_semaphore()->give();
    // }
    // get_data();
    // static uint32_t last_start = 0;
    // uint32_t start = AP_HAL::millis();
    // uint32_t since_last = start - last_start;
    // last_start = start;

    // if (_in_timer) { hal.console->printf("[centeye] reentry\n"); return; }
    // _in_timer = true;

    // uint32_t t0 = AP_HAL::millis();
    get_data();
    // uint32_t dur = AP_HAL::millis() - t0;
    // hal.console->printf("Duration: %lu\n", dur);

    // if (dur > 16) hal.console->printf("[centeye] loop=%lums, gap=%lums\n", dur, since_last);

    // _in_timer = false;
}

bool AP_Centeye_Nano_Backend::get_data()
{

    // Try and only read one data set 
    if (counter == 0)
    {
        if (!read_odom(counter))
        {
            // Error handling goes here
        }
    }
    else if (counter == 1)
    {
        if (!read_odom(counter))
        {
            // Error handling goes here
        }
    }
    else if (counter == 2)
    {
        if (!read_objdet_h(counter))
        {
            // Error handling goes here
        }
    }
    else if (counter == 3)
    {
        if (!read_objdet_h(counter))
        {
            // Error handling goes here
        }
    }
    counter = (counter + 1) % 4;
    // if (!read_odom())
    // {
    //     // Error handling goes here for failure to read odometry
    // }
    // hal.scheduler->delay_microseconds(200); 
    // if (!read_id())
    // {
    //     // Error handling goes here for failure to read id
    // }
    // if (!read_objdet_h())
    // {
    //     // Error handling goes here for failure to read objdet
    // }
    // if (!read_objdet_v())
    // {
    //     // Error handling goes here for failure to read objdet
    // }
    return true;

}

bool AP_Centeye_Nano_Backend::read_odom(uint8_t cmd)
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

    // get the semaphore
    bool has_sem = _dev->get_semaphore()->take(50);
    if (has_sem)
    {
        if (cmd == 0)
        {
            uint8_t command[] = {dtt_ds_only, odo_ds_id};
            if (!write_bytes(command, 2))
            {
                // Error handling goes here...
                // hal.console->printf("Write failed\n");
            }
            _dev->get_semaphore()->give();
        }
        else
        {
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
            float dt = ((float) current_time - (float) unsafe_data.meas_time) / 1000.0;
            // unsafe_data.meas_time = current_time;
            unsafe_data.flow_x = ((float) unsafe_data.odom_x - (float) old_odom_x) / dt;
            unsafe_data.flow_y = ((float) unsafe_data.odom_y - (float) old_odom_y) / dt;
            unsafe_data.flow_div = ((float) unsafe_data.odom_div - (float) old_odom_div) / dt;
            _dev->get_semaphore()->give();

        }
    }
    else
    {
        // error handling for no semaphore goes here
    }



    // if (has_sem)
    // {
    //     uint8_t buffer[ODOM_BYTES];

    //     int32_t old_odom_x = unsafe_data.odom_x;
    //     int32_t old_odom_y = unsafe_data.odom_y;
    //     int32_t old_odom_div = unsafe_data.odom_div;
    //     uint32_t current_time = AP_HAL::millis();

    //     uint8_t command[] = {dtt_ds_only, odo_ds_id};
    //     if (!write_bytes(command, 2))
    //     {
    //         // Error handling goes here...
    //         // hal.console->printf("Write failed\n");
    //     }

    //     // Read into the buffer
    //     if (!_dev->read(buffer, 12))
    //     {
    //         // Error handling goes here... 
    //         // hal.console->printf("Read failed\n");
    //     }

    //     // With the data now in the buffer, we can bit shift into the proper form
    //     unsafe_data.odom_x = (uint32_t) buffer[3] << 24 | (uint32_t) buffer[2] << 16 | (uint32_t) buffer[1] << 8 | (uint32_t) buffer[0];
    //     unsafe_data.odom_y = (uint32_t) buffer[7] << 24 | (uint32_t) buffer[6] << 16 | (uint32_t) buffer[5] << 8 | (uint32_t) buffer[4];
    //     unsafe_data.odom_div = (uint32_t) buffer[11] << 24 | (uint32_t) buffer[10] << 16 | (uint32_t) buffer[9] << 8 | (uint32_t) buffer[8];
    //     // Calculate the time step in seconds
    //     float dt = ((float) current_time - (float) unsafe_data.meas_time) / 1000.0;
    //     unsafe_data.meas_time = current_time;
    //     unsafe_data.flow_x = ((float) unsafe_data.odom_x - (float) old_odom_x) / dt;
    //     unsafe_data.flow_y = ((float) unsafe_data.odom_y - (float) old_odom_y) / dt;
    //     unsafe_data.flow_div = ((float) unsafe_data.odom_div - (float) old_odom_div) / dt;

    //     _dev->get_semaphore()->give();
    // }   
    return true;
}

// bool AP_Centeye_Nano_Backend::read_id()
// {
//     bool has_sem = _dev->get_semaphore()->take(50);
//     if (has_sem)
//     {
//         uint8_t buffer[ID_BYTES];

//         uint8_t command[] = {dtt_ds_only, (uint8_t) 0};
//         if (!write_bytes(command, 2))
//         {
//             // Error handling goes here...
//             hal.console->printf("Write failed\n");
//         }

//         // Read into the buffer
//         if (!_dev->read(buffer, 4))
//         {
//             // Error handling goes here... 
//             hal.console->printf("Read failed\n");
//         }

//         // With the data now in the buffer, we can bit shift into the proper form
//         unsafe_data.id[0] = buffer[0];
//         unsafe_data.id[1] = buffer[1];
//         unsafe_data.id[2] = buffer[2];
//         unsafe_data.id[2] = buffer[2];
//         _dev->get_semaphore()->give();
//     }   
//     return true;

// }

bool AP_Centeye_Nano_Backend::read_objdet_h(uint8_t cmd)
{
    // Read objdet details
    // Send the following bytes to the sensor:
    //      ATT DS ONLY
    //      DS ID (12, then 13 OR 0xC then 0xD)
    // We expect to receive 64 total bytes, representing a 4x4 matrix of 4-byte integers in little endian format
    // Given the layout of 2D arrays in memory, we can do everything simply using pointer arithmetic
    // Here is the implementation

    // get the semaphore
    bool has_sem = _dev->get_semaphore()->take(50);

    if (has_sem)
    {
        if (cmd == 2)
        {
            uint8_t command_h[] = {dtt_ds_only, objdet_h_ds_id};
            if (!write_bytes(command_h, 2))
            {
                // Error handling goes here...
            }
            _dev->get_semaphore()->give();
        }
        else
        {
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
    }

    // if (has_sem)
    // {
    //     uint8_t buffer[OBJDET_BYTES];

    //     uint8_t command_h[] = {dtt_ds_only, objdet_h_ds_id};
    //     if (!write_bytes(command_h, 2))
    //     {
    //         // Error handling goes here...
    //     }
    //     // Read into the buffer
    //     if (!_dev->read(buffer, 64))
    //     {
    //         // Error handling goes here... 
    //     }

    //     int32_t* ptr = unsafe_data.objdet_h;
    //     for (uint8_t i = 0; i < 64; i += 4)
    //     {
    //         *ptr = (uint32_t) buffer[i + 3] << 24 | (uint32_t) buffer[i + 2] << 16 | (uint32_t) buffer[i + 1] << 8 | (uint32_t) buffer[i];
    //         ptr++;
    //     }
    //     _dev->get_semaphore()->give();
        // Assume the buffer is now populated with our horizontal data.
        // // Let's get the pointer to the start of the 4x4 horizontal data matrix
        // int32_t* ptr = unsafe_data.objdet_h[0];
        // // We know that the 4x4 matrix simply takes up 64 sequential bytes in memory, so we will rely on this pointer for navigating that memory
        // for (uint8_t i = 0; i < OBJDET_BYTES; i += 4)
        // {
        //     *ptr = buffer[i+3] << 24 | buffer[i+2] << 16 | buffer[i+1] << 8 | buffer[i];
        //     ptr++;
        // }

        // // Let's reset the command to request the vertical pixel data
        // uint8_t command_v[] = {dtt_ds_only, objdet_v_ds_id};
        // if (!write_bytes(command_v, 2))
        // {
        //     // Error handling goes here... 
        // }
        // if (!_dev->read(buffer, 64))
        // {
        //     // Error handling goes here... 
        // }
        // // Let's reset our pointer
        // ptr = unsafe_data.objdet_v[0];
        // for (uint8_t i = 0; i < 64; i += 4)
        // {
        //     *ptr = buffer[i+3] << 24 | buffer[i+2] << 16 | buffer[i+1] << 8 | buffer[i];
        //     ptr++;
        // }
    // }

    return true;

}

// bool AP_Centeye_Nano_Backend::read_objdet_v()
// {
//     // Read objdet details
//     // Send the following bytes to the sensor:
//     //      ATT DS ONLY
//     //      DS ID (12, then 13 OR 0xC then 0xD)
//     // We expect to receive 64 total bytes, representing a 4x4 matrix of 4-byte integers in little endian format
//     // Given the layout of 2D arrays in memory, we can do everything simply using pointer arithmetic
//     // Here is the implementation

//     // get the semaphore
//     bool has_sem = _dev->get_semaphore()->take(50);

//     if (has_sem)
//     {
//         uint8_t buffer[OBJDET_BYTES];

//         uint8_t command[] = {dtt_ds_only, objdet_v_ds_id};
//         if (!write_bytes(command, 2))
//         {
//             // Error handling goes here...
//         }
//         // Read into the buffer
//         if (!_dev->read(buffer, 64))
//         {
//             // Error handling goes here... 
//         }

//         int32_t* ptr = unsafe_data.objdet_v;
//         for (uint8_t i = 0; i < 64; i += 4)
//         {
//             *ptr = (uint32_t) buffer[i + 3] << 24 | (uint32_t) buffer[i + 2] << 16 | (uint32_t) buffer[i + 1] << 8 | (uint32_t) buffer[i];
//             ptr++;
//         }
//         // Assume the buffer is now populated with our horizontal data.
//         // // Let's get the pointer to the start of the 4x4 horizontal data matrix
//         // int32_t* ptr = unsafe_data.objdet_h[0];
//         // // We know that the 4x4 matrix simply takes up 64 sequential bytes in memory, so we will rely on this pointer for navigating that memory
//         // for (uint8_t i = 0; i < OBJDET_BYTES; i += 4)
//         // {
//         //     *ptr = buffer[i+3] << 24 | buffer[i+2] << 16 | buffer[i+1] << 8 | buffer[i];
//         //     ptr++;
//         // }

//         // // Let's reset the command to request the vertical pixel data
//         // uint8_t command_v[] = {dtt_ds_only, objdet_v_ds_id};
//         // if (!write_bytes(command_v, 2))
//         // {
//         //     // Error handling goes here... 
//         // }
//         // if (!_dev->read(buffer, 64))
//         // {
//         //     // Error handling goes here... 
//         // }
//         // // Let's reset our pointer
//         // ptr = unsafe_data.objdet_v[0];
//         // for (uint8_t i = 0; i < 64; i += 4)
//         // {
//         //     *ptr = buffer[i+3] << 24 | buffer[i+2] << 16 | buffer[i+1] << 8 | buffer[i];
//         //     ptr++;
//         // }
//     }

//     return true;

// }

bool AP_Centeye_Nano_Backend::copy_to_front_end()
{
    // Copy details:
    // Obtain the semaphore, copy all data to the front end structure
    // bool has_sem = _dev->get_semaphore()->take(50);
    // if (has_sem)
    // {
        // Assume we have the semaphore and lets copy over all of the data
        _front_end->sensors[sensor_id].odom_x = unsafe_data.odom_x;
        _front_end->sensors[sensor_id].odom_y = unsafe_data.odom_y;
        _front_end->sensors[sensor_id].odom_div = unsafe_data.odom_div;
        _front_end->sensors[sensor_id].flow_x = unsafe_data.flow_x;
        _front_end->sensors[sensor_id].flow_y = unsafe_data.flow_y;
        _front_end->sensors[sensor_id].flow_div = unsafe_data.flow_div;

        // for (uint8_t i = 0; i < 4; i++)
        // {
        //     _front_end->sensors[0].id[i] = unsafe_data.id[i];
        // }

        // for (uint8_t i = 0; i < 16; i++)
        // {
        //     _front_end->sensors[sensor_id].objdet_h[i] = unsafe_data.objdet_h[i];
        //     _front_end->sensors[sensor_id].objdet_v[i] = unsafe_data.objdet_v[i];
        // }
        // _dev->get_semaphore()->give();
    // }

    // // Lets just keep things simple and use pointer arithmetic, not worrying about the dimensionality of the matrices
    // int32_t* dest_ptr = _front_end->sensors[sensor_id].objdet_h[0];
    // int32_t* src_ptr = unsafe_data.objdet_h[0];

    // for (uint8_t i = 0; i < 16; i++)
    // {
    //     *dest_ptr = *src_ptr;
    //     dest_ptr++;
    //     src_ptr++;
    // }
    // dest_ptr = _front_end->sensors[sensor_id].objdet_v[0];
    // src_ptr = unsafe_data.objdet_v[0];
    // for (uint8_t i = 0; i < 16; i++)
    // {
    //     *dest_ptr = *src_ptr;
    //     dest_ptr++;
    //     src_ptr++;
    // };
    return true;
}