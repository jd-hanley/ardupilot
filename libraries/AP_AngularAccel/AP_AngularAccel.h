/*
  AP_AngularAccel - Angular acceleration storage library
  Provides thread-safe access to angular rate and acceleration data
 */

#pragma once

#include "AP_AngularAccel_config.h"

#if AP_ANGULARACCEL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

class AP_AngularAccel
{
public:
    AP_AngularAccel();

    CLASS_NO_COPY(AP_AngularAccel);

    // Singleton access
    static AP_AngularAccel *get_singleton(void) { return _singleton; }

    // Data structure for angular rate and acceleration
    struct AngularData {
        Vector3f angular_rate;        // rad/s from IMU
        Vector3f angular_accel;       // rad/s^2 computed derivative
        uint32_t timestamp_us;        // microsecond timestamp
        bool valid;                   // data validity flag
    };

    // Set data from CAN driver (thread-safe)
    void set_data(const Vector3f &rate, const Vector3f &accel, uint32_t timestamp_us);

    // Get latest data (thread-safe) - returns false if no valid data
    bool get_data(AngularData &data) const;

    // Individual accessors
    Vector3f get_angular_rate() const;
    Vector3f get_angular_accel() const;
    uint32_t get_last_update_us() const;

    // Health check - returns true if data is recent
    bool healthy() const;

private:
    static AP_AngularAccel *_singleton;

    AngularData _data;
    mutable HAL_Semaphore _sem;

    static constexpr uint32_t TIMEOUT_US = 50000;  // 50ms timeout
};

namespace AP {
    AP_AngularAccel *angularaccel();
};

#endif // AP_ANGULARACCEL_ENABLED
