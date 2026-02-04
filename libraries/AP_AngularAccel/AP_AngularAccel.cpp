/*
  AP_AngularAccel - Angular acceleration storage library
 */

#include "AP_AngularAccel.h"

#if AP_ANGULARACCEL_ENABLED

AP_AngularAccel *AP_AngularAccel::_singleton;

AP_AngularAccel::AP_AngularAccel()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_AngularAccel must be singleton");
    }
    _singleton = this;

    _data.valid = false;
    _data.timestamp_us = 0;

    // Initialize low-pass filters with default cutoff frequency
    _accel_filter_x.set_cutoff_frequency(_filter_cutoff_hz);
    _accel_filter_y.set_cutoff_frequency(_filter_cutoff_hz);
    _accel_filter_z.set_cutoff_frequency(_filter_cutoff_hz);
}

// Set data from CAN driver (thread-safe)
void AP_AngularAccel::set_data(const Vector3f &rate, const Vector3f &accel, uint32_t timestamp_us)
{
    WITH_SEMAPHORE(_sem);
    _data.angular_rate = rate;
    _data.timestamp_us = timestamp_us;
    _data.valid = true;

    // Calculate dt for filter and frequency tracking
    float dt_s = 0.0f;
    if (_prev_timestamp_us != 0 && timestamp_us > _prev_timestamp_us) {
        uint32_t dt_us = timestamp_us - _prev_timestamp_us;
        if (dt_us > 0 && dt_us < 1000000) {  // sanity check: dt between 1us and 1s
            dt_s = dt_us * 1e-6f;
            // Calculate update frequency using low-pass filter
            float instant_freq = 1.0f / dt_s;
            _update_freq_hz = _update_freq_hz * (1.0f - FREQ_FILTER_ALPHA) + instant_freq * FREQ_FILTER_ALPHA;
        }
    }
    _prev_timestamp_us = timestamp_us;

    // Apply low-pass filter to angular acceleration if enabled and we have valid dt
    if (_filter_enabled && dt_s > 0.0f) {
        _data.angular_accel.x = _accel_filter_x.apply(accel.x, dt_s);
        _data.angular_accel.y = _accel_filter_y.apply(accel.y, dt_s);
        _data.angular_accel.z = _accel_filter_z.apply(accel.z, dt_s);
    } else {
        _data.angular_accel = accel;
    }
}

// Get latest data (thread-safe) - returns false if no valid data
bool AP_AngularAccel::get_data(AngularData &data) const
{
    WITH_SEMAPHORE(_sem);
    if (!_data.valid) {
        return false;
    }
    data = _data;
    return true;
}

// Get angular rate
Vector3f AP_AngularAccel::get_angular_rate() const
{
    WITH_SEMAPHORE(_sem);
    return _data.angular_rate;
}

// Get angular acceleration
Vector3f AP_AngularAccel::get_angular_accel() const
{
    WITH_SEMAPHORE(_sem);
    return _data.angular_accel;
}

// Get timestamp of last update
uint32_t AP_AngularAccel::get_last_update_us() const
{
    WITH_SEMAPHORE(_sem);
    return _data.timestamp_us;
}

// Health check - returns true if data is recent
bool AP_AngularAccel::healthy() const
{
    WITH_SEMAPHORE(_sem);
    if (!_data.valid) {
        return false;
    }
    uint32_t now_us = AP_HAL::micros();
    return (now_us - _data.timestamp_us) < TIMEOUT_US;
}

// Get average update frequency
float AP_AngularAccel::get_update_freq_hz() const
{
    WITH_SEMAPHORE(_sem);
    return _update_freq_hz;
}

// Set filter cutoff frequency
void AP_AngularAccel::set_accel_filter_cutoff(float cutoff_hz)
{
    WITH_SEMAPHORE(_sem);
    _filter_cutoff_hz = cutoff_hz;
    _accel_filter_x.set_cutoff_frequency(cutoff_hz);
    _accel_filter_y.set_cutoff_frequency(cutoff_hz);
    _accel_filter_z.set_cutoff_frequency(cutoff_hz);
}

namespace AP {

AP_AngularAccel *angularaccel()
{
    return AP_AngularAccel::get_singleton();
}

}

#endif // AP_ANGULARACCEL_ENABLED
