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
}

// Set data from CAN driver (thread-safe)
void AP_AngularAccel::set_data(const Vector3f &rate, const Vector3f &accel, uint32_t timestamp_us)
{
    WITH_SEMAPHORE(_sem);
    _data.angular_rate = rate;
    _data.angular_accel = accel;
    _data.timestamp_us = timestamp_us;
    _data.valid = true;
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

namespace AP {

AP_AngularAccel *angularaccel()
{
    return AP_AngularAccel::get_singleton();
}

}

#endif // AP_ANGULARACCEL_ENABLED
