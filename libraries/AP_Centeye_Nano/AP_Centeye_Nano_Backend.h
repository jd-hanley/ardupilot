
#pragma once
#include "AP_Centeye_Nano.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/I2CDevice.h>

#define ODOM_BYTES 12
#define OBJDET_BYTES 64
#define COMMAND_LENGTH 2
#define ID_BYTES 4

class AP_Centeye_Nano_Backend
{
    // *** New approach using I2C ***
    // Goal 140 bytes (two 4x4 matrices + odometry values per sensor) at 60 Hz
    // Individual backend for each sensor

    public:
        // Constructor
        AP_Centeye_Nano_Backend(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, AP_Centeye_Nano* singleton, uint8_t id);

        // Init method
        // Should call register periodic callback at 60 Hz
        bool init();
        // Method for copying data to the safe front end data structure
        bool copy_to_front_end();

    private:
        // Reference to the shared front end object
        AP_Centeye_Nano* _front_end;
        // Smart pointer to I2CDevice
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

        // Convenient wrapper for I2C Device Manager's transfer function
        bool write_byte(uint8_t write_byte);
        // Derived wrapper for I2C Device Manager's transfer function
        bool write_bytes(uint8_t* bytes, uint8_t length);
        // Primary function set to run at 60 hz... must receive data from the sensor and update the front end
        void timer();
        // Read data from the sensor
        bool get_data();
        bool read_odom();
        bool read_objdet_h();
        // bool read_objdet_v();
        // bool read_id();

        // Some useful data members
        uint8_t sensor_id;
        uint8_t dtt_ds_only = 0xFF;
        uint8_t odo_ds_id = 11;
        uint8_t objdet_h_ds_id = 13;
        uint8_t objdet_v_ds_id = 12;
        uint8_t counter = 1;
        bool _in_timer = false;
        AP_Centeye_Nano::sensor unsafe_data;
};