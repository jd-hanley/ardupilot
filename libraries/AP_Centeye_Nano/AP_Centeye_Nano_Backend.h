
#pragma once
#include "AP_Centeye_Nano.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/I2CDevice.h>

class AP_Centeye_Nano_Backend
{
    // *** New approach using I2C ***
    // Goal 140 bytes (two 4x4 matrices + odometry values per sensor) at 60 Hz
    // Individual backend for each sensor

    public:
        // Constructor
        AP_Centeye_Nano_Backend(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, AP_Centeye_Nano* singleton);

        // Init method
        // Should call register periodic callback at 60 Hz
        bool init();

    private:
        // Reference to the shared front end object
        AP_Centeye_Nano* _front_end;
        // Smart pointer to I2CDevice
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

        // Convenient wrapper for I2C Device Manager's transfer function
        bool write_byte(uint8_t write_byte);
        // Primary function set to run at 60 hz... must receive data from the sensor and update the front end
        void timer();
        // Read data from the sensor
        bool read_odom();
        bool read_objdet();

        // Some useful data members
        int8_t dev_addr_r;
        int8_t dev_addr_w;
        int8_t dtt_ds_only = 0xFF;
        int8_t odo_ds_id = 11;
        int8_t objdet_h_ds_id = 13;
        int8_t objdet_z_ds_id = 12;
        AP_Centeye_Nano::sensor unsafe_data;
};