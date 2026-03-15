#pragma once


#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <Filter/DerivativeFilter.h>
#include <Filter/LowPassFilter.h>
#include <AP_MSP/msp.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <GCS_MAVLink/ap_message.h>



#define STRAIN_MAX_INSTANCES 2
#define STRAIN_SENSORS 8
#define NUM_ARMS 4
#define BUS_NUMBER 0
// timeouts for health reporting
#define STRAIN_TIMEOUT_MS                 500     // timeout in ms since last successful read
#define STRAIN_DATA_CHANGE_TIMEOUT_MS    2000     // timeout in ms since first strain gauge reading changed
#define STRAIN_MIN_REFRESH_RATE_HZ        60      // minimum refresh rate in Hz for sensor health
#define STRAIN_ACCEL_FILTER_CUTOFF_HZ     20.0f   // low-pass filter cutoff for strain angular acceleration
#define STRAIN_IMU_DERIV_FILTER_CUTOFF_HZ 30.0f   // low-pass filter cutoff for IMU gyro derivative
#define STRAIN_CF_CROSSOVER_HZ             2.0f   // complementary filter crossover frequency (Hz)

class AP_Strain_Backend;

class AP_Strain
{
    friend class AP_Strain_Backend;

    public:
    // constructor
    AP_Strain();

    // Do not allow copies 
    CLASS_NO_COPY(AP_Strain);

    // get singleton
    static AP_Strain *get_singleton(void) 
    {
        return _singleton;
    }

    // initialise the strain object, loading backend drivers
    void init(void);
    
    enum class Status {
        Good           = 0,
        Bad            = 1
    };

    
    int32_t* get_data(uint8_t instance);
    float get_roll_accel_strain();
    float get_pitch_accel_strain();
    float get_roll_accel_imu();
    float get_pitch_accel_imu();
    float get_roll_accel_cf();
    float get_pitch_accel_cf();
    void get_arm_averages(float* destination);
    uint8_t get_num_sensors();
    AP_Strain::Status get_status(uint8_t instance);
    uint32_t get_last_update(uint8_t instance);
    uint8_t get_ID(uint8_t instance) { return sensors[instance].I2C_id; }
    float get_avg_refresh_rate(uint8_t instance) { return sensors[instance].avg_refresh_rate_hz; }
    
    bool reset_all();
    bool calibrate_all();
    bool get_status_all();

    // update status checks (should be called at 10Hz)
    void update();

    uint16_t get_num_calibrations() const { return num_cal; }

    ////////////////////////////////////////////////////////////////////////////////////////////// 
    private:

    float apply_strain_weights(const float *weights);
    void update_strain_accel();
    void update_imu_accel(float raw_strain_roll, float raw_strain_pitch, float dt_s);

    // singleton
    static AP_Strain *_singleton;

    // how many drivers do we have?
    AP_Strain_Backend *drivers[STRAIN_MAX_INSTANCES];

    // how many sensors do we have?
    uint8_t _num_sensors = 0;
    uint8_t _primary = 0;
    uint32_t old_time = 0;
    bool init_done = false;

    // pre multiply strain values by this weight 
    float strain_scale = 6000.0f; 

    // weights and intercepts for calculating angular acceleration from strain gauges
    const float strain_accel_weights_roll[12] = {
        -1.068080f,
        -1.683783f,
        0.8467935f,
        -0.922849f,
        0.4239221f,
        0.8202695f,
        0.7667506f,
        -1.668138f,
        -2.331724f,
        0.3536477f,
        0.4822857f,
        1.243949f
    };

    const float strain_accel_intercept_roll = -0.27805f;
    
    const float strain_accel_weights_pitch[12] = {
        -0.698936f,
         0.560709f,
        -0.828038f,
        -0.200762f,
        -1.29055f,
        -1.19833f,
        -0.121431f,
        0.398409f,
        0.768617f,
        1.10044f,
        -1.55722f,
        -1.16315f
    };
    const float strain_accel_intercept_pitch = 0.51966f;

    uint16_t num_cal = 0; // number of calibrations done

    struct sensor
    {
        uint32_t last_update_ms;        // last update time in ms
        uint32_t last_change_ms;        // last update time in ms that included a change in reading from previous readings
        uint32_t last_status_check_ms;  // last status check time in ms
        uint16_t update_count;          // number of updates since last status check
        float avg_refresh_rate_hz;      // average refresh rate in Hz
        static const uint8_t num_data = STRAIN_SENSORS;
        int32_t data[num_data];         // 8 strain gauge measurements
        int32_t prev_data[num_data];    // previous data for staleness check
        enum AP_Strain::Status status;
        uint8_t I2C_id;

    } sensors[STRAIN_MAX_INSTANCES];

    struct angular_accel_strain {
        float roll_accel;        // strain-derived roll angular accel (LPF'd), rad/s^2
        float pitch_accel;       // strain-derived pitch angular accel (LPF'd), rad/s^2
        float imu_roll_accel;    // IMU gyro derivative roll angular accel (LPF'd), rad/s^2
        float imu_pitch_accel;   // IMU gyro derivative pitch angular accel (LPF'd), rad/s^2
        float cf_roll_accel;     // complementary filter roll: IMU + strain, rad/s^2
        float cf_pitch_accel;    // complementary filter pitch: IMU + strain, rad/s^2
    } accel_strain;

    // LPF for strain-derived angular acceleration
    LowPassFilterFloat _strain_filter_roll;
    LowPassFilterFloat _strain_filter_pitch;
    uint32_t _last_filter_update_ms{0};

    // LPF for IMU gyro derivative
    LowPassFilterFloat _imu_deriv_filter_roll;
    LowPassFilterFloat _imu_deriv_filter_pitch;

    // LPFs for complementary filter (crossover frequency)
    LowPassFilterFloat _cf_lpf_imu_roll;
    LowPassFilterFloat _cf_lpf_imu_pitch;
    LowPassFilterFloat _cf_lpf_strain_roll;
    LowPassFilterFloat _cf_lpf_strain_pitch;

    // Previous gyro sample for finite-difference derivative
    Vector3f _prev_gyro;
    bool _imu_accel_initialized{false};

};

namespace AP {
    AP_Strain &strain();
};
