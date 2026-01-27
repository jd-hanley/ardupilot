/*
  AP_InertialSenseCAN - CAN driver for Inertial Sense IMX5 IMU
  Parses gyro angular rate and computes angular acceleration
 */

#pragma once

#include "AP_InertialSenseCAN_config.h"

#if AP_INERTIALSENSECAN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANSensor.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <Filter/DerivativeFilter.h>

class AP_AngularAccel;

class AP_InertialSenseCAN_Driver : public CANSensor
{
public:
    AP_InertialSenseCAN_Driver();

    // Called periodically from main loop
    void update();

private:
    // Required CANSensor override - handles incoming CAN frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

    // Parse gyro message for specific axis (0=X, 1=Y, 2=Z)
    void parse_gyro_message(const AP_HAL::CANFrame &frame, uint8_t axis);

    // Compute angular acceleration from accumulated gyro data
    void compute_angular_accel();

    // Inertial Sense CAN Message IDs
    // These are configurable on the IMX5, default values shown
    // CID_DUAL messages contain raw gyro/accel data
    static const uint16_t CID_DUAL_PX = 0x0D;  // Gyro X + Accel X
    static const uint16_t CID_DUAL_QY = 0x0E;  // Gyro Y + Accel Y
    static const uint16_t CID_DUAL_RZ = 0x0F;  // Gyro Z + Accel Z

    // State tracking
    struct {
        Vector3f gyro_rate;           // Current gyro reading (rad/s)
        uint32_t gyro_timestamp_us;   // Timestamp of current reading
        uint8_t axes_received;        // Bitmask of received axes (0x07 = all)
        bool initialized;             // First valid reading received
    } _state;

    // Derivative filters for each axis (Holoborodko smooth differentiator)
    DerivativeFilterFloat_Size5 _deriv_filter_x;
    DerivativeFilterFloat_Size5 _deriv_filter_y;
    DerivativeFilterFloat_Size5 _deriv_filter_z;

    // Semaphore for thread safety
    HAL_Semaphore _sem;
};

class AP_InertialSenseCAN
{
public:
    AP_InertialSenseCAN();

    CLASS_NO_COPY(AP_InertialSenseCAN);

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();

    static AP_InertialSenseCAN *get_singleton() { return _singleton; }

private:
    static AP_InertialSenseCAN *_singleton;

    // Parameters
    AP_Int8 _enabled;

    AP_InertialSenseCAN_Driver *_driver;
};

namespace AP {
    AP_InertialSenseCAN *inertialsensecan();
};

#endif // AP_INERTIALSENSECAN_ENABLED
