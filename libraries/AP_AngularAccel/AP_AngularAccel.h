/*
  AP_AngularAccel - Angular acceleration storage library
  Provides thread-safe access to angular rate and acceleration data
 */

#pragma once

#include "AP_AngularAccel_config.h"

#if AP_ANGULARACCEL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter.h>

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

    // Get average update frequency (Hz)
    float get_update_freq_hz() const;

    // Low-pass filter control for angular acceleration
    void set_accel_filter_enabled(bool enabled) { _filter_enabled = enabled; }
    bool get_accel_filter_enabled() const { return _filter_enabled; }

    // Set filter cutoff frequency (Hz)
    void set_accel_filter_cutoff(float cutoff_hz);
    float get_accel_filter_cutoff() const { return _filter_cutoff_hz; }

private:
    static AP_AngularAccel *_singleton;

    AngularData _data;
    mutable HAL_Semaphore _sem;

    // Frequency tracking
    uint32_t _prev_timestamp_us{0};      // previous update timestamp
    float _update_freq_hz{0.0f};         // filtered update frequency
    static constexpr float FREQ_FILTER_ALPHA = 0.1f;  // low-pass filter coefficient

    // Low-pass filter for angular acceleration
    bool _filter_enabled{true};                       // filter enable flag (enabled by default)
    float _filter_cutoff_hz{50.0f};                   // filter cutoff frequency (Hz)
    LowPassFilterFloat _accel_filter_x;               // X-axis (roll) filter
    LowPassFilterFloat _accel_filter_y;               // Y-axis (pitch) filter
    LowPassFilterFloat _accel_filter_z;               // Z-axis (yaw) filter

    static constexpr uint32_t TIMEOUT_US = 50000;  // 50ms timeout
};

namespace AP {
    AP_AngularAccel *angularaccel();
};

#endif // AP_ANGULARACCEL_ENABLED
