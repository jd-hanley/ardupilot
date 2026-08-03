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

class AP_InertialSenseCAN_Driver
{
public:
    AP_InertialSenseCAN_Driver();

    // Called periodically from main loop
    void update();

private:
    // Callback registered with the shared MultiCAN dispatcher, since this bus
    // may carry other CAN sensors (e.g. AP_Strain) alongside the IMU.
    // Returns true if the frame was one of ours, so the dispatcher knows not
    // to offer it to any other listener sharing this bus.
    bool handle_frame(AP_HAL::CANFrame &frame);

    // Handles frames for this driver's own CAN protocol slot; may be shared
    // with other sensors wired to the same physical bus.
    MultiCAN *_multican;

    // Parse gyro message for specific axis (0=X, 1=Y, 2=Z)
    void parse_gyro_message(const AP_HAL::CANFrame &frame, uint8_t axis);

    // Compute angular acceleration from accumulated gyro data
    void compute_angular_accel();

    // Inertial Sense CAN Message IDs
    // Configured to match IMX5 RUG3 settings: P=2, Q=3 (R ignored)
    static const uint16_t CID_DUAL_PX = 0x02;  // Gyro P (X axis)
    static const uint16_t CID_DUAL_QY = 0x03;  // Gyro Q (Y axis)
    // State tracking
    struct {
        Vector3f gyro_rate;           // Current gyro reading (rad/s)
        uint32_t gyro_timestamp_us;   // Timestamp of current reading
        uint32_t first_axis_timestamp_us; // Timestamp of first axis in current set
        uint8_t axes_received;        // Bitmask of received axes (0x07 = all)
        bool initialized;             // First valid reading received
    } _state;

    // Maximum time allowed between first and last axis in a set (microseconds)
    static constexpr uint32_t MAX_AXIS_SPREAD_US = 2000;  // 2ms max spread

    // Maximum valid gyro rate (rad/s) - reject values beyond this
    static constexpr float MAX_GYRO_RATE = 35.0f;  // ~2000 deg/s

    // Derivative filters for P and Q axes (Holoborodko smooth differentiator)
    DerivativeFilterFloat_Size5 _deriv_filter_x;
    DerivativeFilterFloat_Size5 _deriv_filter_y;

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
