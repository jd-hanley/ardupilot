#include "AP_Strain_Backend.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

#ifdef USE_STRAIN_RATE_SENSOR

AP_Strain_Backend::AP_Strain_Backend(AP_Strain::sensor &_strain_arm, AP_Strain* singleton) :
    _frontEnd(singleton),
    _sensor(_strain_arm)
{
    // Shares the InertialSenseCAN driver slot/interface rather than owning
    // its own, since both sensors are wired to the same physical CAN bus.
    _multican = NEW_NOTHROW MultiCAN{FUNCTOR_BIND_MEMBER(&AP_Strain_Backend::handle_frame, bool, AP_HAL::CANFrame &), AP_CAN::Protocol::InertialSenseCAN, "Strain"};
    if (_multican == nullptr) {
        AP_BoardConfig::allocation_error("Failed to create strain multican");
    }
}

// handler for incoming frames; the Teensy broadcasts one 8-byte frame
// (4 x int16, little-endian) on STRAIN_RATE_CAN_ID
bool AP_Strain_Backend::handle_frame(AP_HAL::CANFrame &frame)
{
    if (frame.isExtended() || (frame.id & AP_HAL::CANFrame::MaskStdID) != _sensor.I2C_id || frame.dlc < 8) {
        return false;
    }

    // Store the old data so we can detect whether this update changed anything
    int16_t last_data = _sensor.data[0];
    int32_t last_time = _sensor.last_update_ms;

    memcpy(&_sensor.data[0], frame.data, 8);

    _sensor.status = AP_Strain::Status::Good;
    _sensor.freq_hz = _sensor.avg_refresh_rate_hz;
    _sensor.last_update_ms = AP_HAL::millis();

    // If the sensor data did not change, extend the last change time
    if (memcmp(&_sensor.data[0], &last_data, sizeof(last_data)) == 0) {
        update_last_change_ms(false, last_time);
    } else {
        update_last_change_ms(true, last_time);
        // Only count updates where new data was actually received
        _sensor.update_count++;
    }
    return true;
}

void AP_Strain_Backend::update_last_change_ms(bool reset, int32_t last_time)
{
    // If the boolean argument is true, reset the last change time to 0
    if (reset)
        _sensor.last_change_ms = 0;
    // Otherwise, increment last change time by the amount of time that has passed
    else
        _sensor.last_change_ms += AP_HAL::millis() - last_time;
}

// set status and update valid count
void AP_Strain_Backend::set_status(AP_Strain::sensor &_strain_arg, AP_Strain::Status status)
{
    _strain_arg.status = status;
}

// true if sensor is returning data
bool AP_Strain_Backend::has_data() const {
    return (_sensor.status == AP_Strain::Status::Good);
}

// The Teensy strain-rate sensor firmware only ever transmits on the CAN
// bus (raw ADC readings, no calibration procedure), so there is nothing
// to send it here. Kept as no-ops so callers (e.g. calibrate_all()) don't
// need to know the backend type.
bool AP_Strain_Backend::calibrate()
{
    return true;
}

bool AP_Strain_Backend::reset()
{
    return true;
}

#else // !USE_STRAIN_RATE_SENSOR

// Constructor -
AP_Strain_Backend::AP_Strain_Backend(AP_Strain::sensor &_strain_arm, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, AP_Strain* singleton) :
    _frontEnd(singleton),
    _sensor(_strain_arm),
    _dev(std::move(dev)),
    _cal_pending(false) {}

// private

bool AP_Strain_Backend::write_byte(uint8_t write_byte)
{
    uint8_t msg = write_byte;
    return _dev->transfer(&msg, sizeof(msg), NULL, 0);
}


bool AP_Strain_Backend::init()
{

    DEV_PRINTF("I2C starting\n");

    // Call timer() at 100Hz
    _dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_Strain_Backend::timer, void));

    if (!calibrate()) {
        return false;
    }

    return true;
}

void AP_Strain_Backend::timer(void)
{
    // Boolean read will be used throughout the function
    bool read = false;

    // Store the old data
    // Joe - Eventually we might want to check data from all strain gauges before arming
    // For now, we will assume either all sensors are working or none of them are working
    float last_data = _sensor.data[0];
    int32_t last_time = _sensor.last_update_ms;

    // Get the semaphore
    bool has_sem = _dev->get_semaphore()->take(50);
    if (has_sem)
    {
        // Send calibration byte if requested by main thread (must be inside semaphore)
        if (_cal_pending) {
            _cal_pending = false;
            write_byte(0x5A);
        }
        // If we have the semaphore, call get_reading and store the return value in read
        read = get_reading();
        _dev->get_semaphore()->give();
    }
    // Key note here: do not worry about the case where we fail to get semaphore... setting read to false by default will handle this case
    // If read returned true, it means we successfully read in data (possibly bad data, but that will be handled later)
    // Set the status
    if (read)
    {
        _sensor.status = AP_Strain::Status::Good;
    }
    // If read returned false, either our writing poll to sensor or reading data from sensor failed (or potentially we did not get the semaphore)
    else
    {
        _sensor.status = AP_Strain::Status::Bad;
    }

    // If the sensor data did not change, we need to call update_last_change_ms with the reset argument as false to extend the last change time
    if (memcmp(&_sensor.data[0], &last_data, sizeof(last_data)) == 0)
    {
        update_last_change_ms(false , last_time);
    }
    // Otherwise, reset the last change time to zero by calling update_last_change_ms with the reset argument as true
    else
    {
        update_last_change_ms(true , last_time);
        // Only count updates where new data was actually received
        _sensor.update_count++;
    }

}

void AP_Strain_Backend::update_last_change_ms(bool reset, int32_t last_time)
{
    // If the boolean argument is true, reset the last change time to 0
    if (reset)
        _sensor.last_change_ms = 0;
    // Otherwise, increment last change time by the amount of time that has passed
    else
        _sensor.last_change_ms += AP_HAL::millis() - last_time;
}

// get_reading: return true if we successfully read in data from the sensor, false otherwise
// Note that the semaphore has already been obtained by the caller.
bool AP_Strain_Backend::get_reading()
{
    // Protocol: send 'P' to trigger data capture, then read 36 floats in 3 chunks of 12.
    // For each chunk: write the chunk index (0/1/2), then read 48 bytes (12 floats).
    if (!write_byte('P')) {
        return false;
    }

    const uint8_t chunk_size  = 12;
    const uint8_t num_chunks  = _sensor.num_data / chunk_size; // 36/12 = 3
    const uint8_t chunk_bytes = chunk_size * sizeof(float);    // 48 bytes

    for (uint8_t chunk = 0; chunk < num_chunks; chunk++) {
        if (!write_byte(chunk)) {
            return false;
        }
        uint8_t buf[chunk_bytes];
        if (!_dev->read(buf, chunk_bytes)) {
            return false;
        }
        memcpy(&_sensor.data[chunk * chunk_size], buf, chunk_bytes);
    }

    _sensor.last_update_ms = AP_HAL::millis();
    return true;
}

// set status and update valid count
void AP_Strain_Backend::set_status(AP_Strain::sensor &_strain_arg, AP_Strain::Status status)
{
    _strain_arg.status = status;
}

// true if sensor is returning data
bool AP_Strain_Backend::has_data() const {
    return (_sensor.status == AP_Strain::Status::Good);
}

bool AP_Strain_Backend::calibrate()
{
    // Set flag so timer() sends the calibration byte from the device thread
    _cal_pending = true;
    return true;
}

bool AP_Strain_Backend::reset()
{
    // Objective send the byte 'R' to the sensor
    bool has_sem = _dev->get_semaphore()->take(50);
    if (has_sem)
    {
        if (!write_byte('R'))
        {
            // Writing to sensor failed
            _dev->get_semaphore()->give();
            return false;
        }
        else
        {
            // Writing to sensor successful
            _dev->get_semaphore()->give();
            return true;
        }
    }
    else
    {
        // Error getting semaphore
        return false;
    }

}


void AP_Strain_Backend::correct_missing_sensor()
{
    // hacky fix for missing sensor 10
    // if (_sensor.data[10] == 0 && !_sensor.data[11] == 0)
    // {
    //     _sensor.data[10] = _sensor.data[9];
    //     _sensor.data[4] = _sensor.data[3];
    //     _sensor.data[5] = _sensor.data[3];
    // }
    return;
}

#endif // USE_STRAIN_RATE_SENSOR
