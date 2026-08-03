#pragma once
#include "AP_Strain.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>

#ifdef USE_STRAIN_RATE_SENSOR

#include <AP_CANManager/AP_CANSensor.h>

// The Teensy strain-rate sensor broadcasts its 4 channels in a single CAN
// frame (STRAIN_RATE_CAN_ID) rather than being polled. It is wired to the
// same physical CAN bus as the InertialSense IMU, so this backend registers
// via MultiCAN under AP_CAN::Protocol::InertialSenseCAN, sharing that
// interface's receive thread rather than owning its own driver slot.
class AP_Strain_Backend
{
public:
    AP_Strain_Backend(AP_Strain::sensor &_strain_arm, AP_Strain* singleton);

    bool init() { return true; }

    // true if sensor is returning data
    bool has_data() const;

    bool reset(void);
    bool calibrate(void);

private:

    // Reference to the shared front end data structure
    AP_Strain* _frontEnd;
    // Reference to the corresponding entry in the front end's sensor array
    AP_Strain::sensor &_sensor;

    // Callback registered with the shared MultiCAN dispatcher. Returns true
    // if the frame was our strain-rate broadcast, so the dispatcher knows
    // not to offer it to any other listener sharing this bus (e.g. the IMU).
    bool handle_frame(AP_HAL::CANFrame &frame);

    MultiCAN *_multican;

    static void set_status(AP_Strain::sensor &_strain_arg, AP_Strain::Status status);
    void set_status(AP_Strain::Status status) { set_status(_sensor, status); }

    // Helper function for updating last change time
    void update_last_change_ms(bool reset, int32_t last_time);
};

#else // !USE_STRAIN_RATE_SENSOR

#include <AP_HAL/Semaphores.h>
#include <AP_HAL/I2CDevice.h>

class AP_Strain_Backend
{
public:
    AP_Strain_Backend(AP_Strain::sensor &_strain_arm, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, AP_Strain* singleton);
    bool init();
    float data() const { return 1.0f; }

    // true if sensor is returning data
    bool has_data() const;

    bool reset(void);
    bool calibrate(void);



private:

    // Reference to the shared front end data structure
    AP_Strain* _frontEnd;
    // Reference to the corresponding entry in the front end's sensor array
    AP_Strain::sensor &_sensor;
    // Smart pointer to I2CDevice
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    // I believe these are both obsolete functions
    static void set_status(AP_Strain::sensor &_strain_arg, AP_Strain::Status status);
    void set_status(AP_Strain::Status status) { set_status(_sensor, status); }

    // Convenient wrapper for I2CDevice's transfer method
    bool write_byte(uint8_t write_byte);
    // Primary function set to run at 100 hz... must receive data from the sensor and update the front end
    void timer();

    // Flag set from main thread; actual I2C write executed inside timer()
    volatile bool _cal_pending;
    // Read data from the sensor
    bool get_reading();
    // Helper function for updating last change time
    void update_last_change_ms(bool reset,int32_t last_time);

    void correct_missing_sensor();

};

#endif // USE_STRAIN_RATE_SENSOR
