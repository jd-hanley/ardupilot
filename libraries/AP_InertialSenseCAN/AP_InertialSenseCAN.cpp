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
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_InertialSenseCAN, _enabled, 0, AP_PARAM_FLAG_ENABLE),

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

    // Validate frame length - Inertial Sense CID_DUAL messages are 8 bytes
    // Format: 4 bytes gyro (float) + 4 bytes accel (float)
    if (frame.dlc < 8) {
        return;
    }

#if AP_INERTIALSENSECAN_DEBUG
    static uint32_t last_debug_ms = 0;
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_debug_ms > 1000) {
        last_debug_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "ISense CAN: id=0x%X dlc=%d", (unsigned)msg_id, (int)frame.dlc);
    }
#endif

    // Parse based on message ID
    switch (msg_id) {
        case CID_DUAL_PX:
            parse_gyro_message(frame, 0);  // X axis
            break;
        case CID_DUAL_QY:
            parse_gyro_message(frame, 1);  // Y axis
            break;
        case CID_DUAL_RZ:
            parse_gyro_message(frame, 2);  // Z axis
            break;
        default:
            // Unknown message ID - could add more message types here
            break;
    }
}

// Parse gyro data from CAN frame
// CID_DUAL format: bytes 0-3 = gyro (float, rad/s), bytes 4-7 = accel (float, m/s^2)
void AP_InertialSenseCAN_Driver::parse_gyro_message(const AP_HAL::CANFrame &frame, uint8_t axis)
{
    if (axis > 2) {
        return;
    }

    // Extract gyro value (first 4 bytes, IEEE 754 float, little-endian)
    float gyro_value;
    memcpy(&gyro_value, frame.data, sizeof(float));

    // Check for NaN or infinity
    if (!isfinite(gyro_value)) {
        return;
    }

    uint32_t now_us = AP_HAL::micros();

    WITH_SEMAPHORE(_sem);

    // Store gyro value in appropriate axis
    switch (axis) {
        case 0:
            _state.gyro_rate.x = gyro_value;
            break;
        case 1:
            _state.gyro_rate.y = gyro_value;
            break;
        case 2:
            _state.gyro_rate.z = gyro_value;
            break;
    }

    _state.axes_received |= (1 << axis);
    _state.gyro_timestamp_us = now_us;

    // When all 3 axes received, compute angular acceleration
    if (_state.axes_received == 0x07) {
        _state.axes_received = 0;
        _state.initialized = true;
        compute_angular_accel();
    }
}

// Compute angular acceleration using derivative filters
void AP_InertialSenseCAN_Driver::compute_angular_accel()
{
    uint32_t now_us = _state.gyro_timestamp_us;

    // Update derivative filters with current gyro rate
    _deriv_filter_x.update(_state.gyro_rate.x, now_us);
    _deriv_filter_y.update(_state.gyro_rate.y, now_us);
    _deriv_filter_z.update(_state.gyro_rate.z, now_us);

    // Get smoothed derivatives (rad/s^2)
    Vector3f angular_accel;
    angular_accel.x = _deriv_filter_x.slope();
    angular_accel.y = _deriv_filter_y.slope();
    angular_accel.z = _deriv_filter_z.slope();

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
