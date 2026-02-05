/*
  AP_InertialSenseCAN - CAN driver for Inertial Sense IMX5 IMU
 */

#include "AP_InertialSenseCAN.h"

#if AP_INERTIALSENSECAN_ENABLED

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_AngularAccel/AP_AngularAccel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_INERTIALSENSECAN_DEBUG 0

// Parameter definitions
const AP_Param::GroupInfo AP_InertialSenseCAN::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Inertial Sense CAN Enable
    // @Description: Enable Inertial Sense IMX5 CAN driver for angular acceleration
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_InertialSenseCAN, _enabled, 1, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

AP_InertialSenseCAN::AP_InertialSenseCAN()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_InertialSenseCAN must be singleton");
    }
#endif
    _singleton = this;
}

void AP_InertialSenseCAN::init()
{
    if (_driver != nullptr) {
        // only allow one instance
        return;
    }

    if (!_enabled) {
        return;
    }

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::InertialSenseCAN) {
            _driver = NEW_NOTHROW AP_InertialSenseCAN_Driver();
            if (_driver != nullptr) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "InertialSenseCAN: driver started");
            }
            return;
        }
    }
}

void AP_InertialSenseCAN::update()
{
    if (_driver == nullptr) {
        return;
    }
    _driver->update();
}

// Driver implementation
AP_InertialSenseCAN_Driver::AP_InertialSenseCAN_Driver() : CANSensor("ISenseCAN")
{
    register_driver(AP_CAN::Protocol::InertialSenseCAN);

    // Initialize state
    _state.axes_received = 0;
    _state.initialized = false;
}

// Handle incoming CAN frames - called from CAN receive thread
void AP_InertialSenseCAN_Driver::handle_frame(AP_HAL::CANFrame &frame)
{
    // Get message ID (support both standard and extended frames)
    uint32_t msg_id;
    if (frame.isExtended()) {
        msg_id = frame.id & AP_HAL::CANFrame::MaskExtID;
    } else {
        msg_id = frame.id & AP_HAL::CANFrame::MaskStdID;
    }

    // Validate frame length - CID_DUAL messages are 8 bytes (float gyro + float accel)
    if (frame.dlc < 8) {
        return;
    }

#if AP_INERTIALSENSECAN_DEBUG
    static uint32_t last_debug_ms = 0;
    static uint32_t frames_p = 0, frames_q = 0, frames_r = 0, frames_other = 0;

    // Count frames per ID
    if (msg_id == CID_DUAL_PX) frames_p++;
    else if (msg_id == CID_DUAL_QY) frames_q++;
    else if (msg_id == CID_DUAL_RZ) frames_r++;
    else frames_other++;

    uint32_t now_ms = AP_HAL::millis();
    uint32_t elapsed_ms = now_ms - last_debug_ms;
    if (elapsed_ms > 1000) {
        // Scale to per-second rate based on actual elapsed time
        uint32_t hz_p = (frames_p * 1000) / elapsed_ms;
        uint32_t hz_q = (frames_q * 1000) / elapsed_ms;
        uint32_t hz_r = (frames_r * 1000) / elapsed_ms;
        uint32_t hz_other = (frames_other * 1000) / elapsed_ms;

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ISense Hz: P=%u Q=%u R=%u other=%u",
                      (unsigned)hz_p, (unsigned)hz_q, (unsigned)hz_r, (unsigned)hz_other);
        frames_p = frames_q = frames_r = frames_other = 0;
        last_debug_ms = now_ms;
    }
#endif

    // Parse based on message ID - P=0x02, Q=0x03 (R ignored)
    switch (msg_id) {
        case CID_DUAL_PX:
            parse_gyro_message(frame, 0);  // P (X axis)
            break;
        case CID_DUAL_QY:
            parse_gyro_message(frame, 1);  // Q (Y axis)
            break;
        default:
            break;
    }
}

// Parse gyro data from CAN frame
// CID_DUAL format: bytes 0-1 = gyro (int16, scaled by 1000, rad/s), bytes 2-3 = accel (int16, scaled by 100, m/s^2)
void AP_InertialSenseCAN_Driver::parse_gyro_message(const AP_HAL::CANFrame &frame, uint8_t axis)
{
    if (axis > 2) {
        return;
    }

    // Extract gyro value (first 2 bytes, int16 little-endian, scaled by 1000)
    int16_t gyro_raw;
    memcpy(&gyro_raw, frame.data, sizeof(int16_t));
    float gyro_value = (float)gyro_raw / 1000.0f;  // Convert to rad/s

    // Validate gyro value is within reasonable bounds
    if (fabsf(gyro_value) > MAX_GYRO_RATE) {
        // Reject bad reading - likely corrupted CAN frame
        return;
    }

    uint32_t now_us = AP_HAL::micros();

    WITH_SEMAPHORE(_sem);

    // Check if we need to reset due to timeout (stale partial data)
    if (_state.axes_received != 0) {
        uint32_t elapsed_us = now_us - _state.first_axis_timestamp_us;
        if (elapsed_us > MAX_AXIS_SPREAD_US) {
            // Too much time since first axis - discard stale data and start fresh
            _state.axes_received = 0;
        }
    }

    // If this is the first axis in a new set, record the timestamp
    if (_state.axes_received == 0) {
        _state.first_axis_timestamp_us = now_us;
    }

    // Store gyro value in appropriate axis (P and Q only, R ignored)
    switch (axis) {
        case 0:
            _state.gyro_rate.x = gyro_value;
            break;
        case 1:
            _state.gyro_rate.y = -gyro_value;
            break;
        default:
            return;  // Ignore other axes
    }

    _state.axes_received |= (1 << axis);
    _state.gyro_timestamp_us = now_us;

    // When P and Q axes received, compute angular acceleration
    if (_state.axes_received == 0x03) {
        _state.axes_received = 0;
        _state.initialized = true;
        compute_angular_accel();
    }
}

// Compute angular acceleration using derivative filters (P and Q only)
void AP_InertialSenseCAN_Driver::compute_angular_accel()
{
    uint32_t now_us = _state.gyro_timestamp_us;

    // Update derivative filters with current gyro rate (X and Y only)
    _deriv_filter_x.update(_state.gyro_rate.x, now_us);
    _deriv_filter_y.update(_state.gyro_rate.y, now_us);

    // Get smoothed derivatives (rad/s^2) - Z is unused
    Vector3f angular_accel;
    angular_accel.x = _deriv_filter_x.slope();
    angular_accel.y = _deriv_filter_y.slope();
    angular_accel.z = 0.0f;

    // Ensure Z gyro rate is zero since we don't use R
    _state.gyro_rate.z = 0.0f;

    // Push to AP_AngularAccel singleton
    AP_AngularAccel *aa = AP::angularaccel();
    if (aa != nullptr) {
        aa->set_data(_state.gyro_rate, angular_accel, now_us);
    }
}

// Called from main loop - currently unused but available for future use
void AP_InertialSenseCAN_Driver::update()
{
    // Nothing needed here - all processing happens in handle_frame
    // which is called from the CAN receive thread
}

// Singleton instance
AP_InertialSenseCAN *AP_InertialSenseCAN::_singleton;

namespace AP {

AP_InertialSenseCAN *inertialsensecan()
{
    return AP_InertialSenseCAN::get_singleton();
}

}

#endif // AP_INERTIALSENSECAN_ENABLED
