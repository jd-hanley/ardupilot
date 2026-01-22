#pragma once


#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <Filter/DerivativeFilter.h>
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
    float strain_scale = 60000.0f; 

    // old weights ( these worked but im trying new, maybe better, values )
    // const float strain_accel_weights_roll[12] = {
    //     -10.0207f, 6.5910f, -14.2687f,      // arm 1
    //     -17.2856f, 0.0f, 17.1085f,          // arm 2
    //     4.6589f, 7.9093f, -4.0103f,         // arm 3
    //     13.0439f, -17.4240f, 1.2953f        // arm 4
    // };
    // const float strain_accel_weights_pitch[12] = {
    //     17.4908f, -4.13330f, -2.23474f, 
    //     -0.255422f, 0.0f, 2.21664f, 
    //     -19.0035f, 17.9168f, 14.3791f, 
    //     -7.49279f, -15.3344f, -21.0789f
    // };

    // weights and intercepts for calculating angular acceleration from strain gauges
    const float strain_accel_weights_roll[12] = {
        -6.36451f, 7.71152f, -16.3007f, 
        -20.8643f, 9.96837f, 23.3253f,
        7.62146f, 11.8745f, -16.7582f,
        25.5925f, -12.2790f, -2.44038f
    };
    const float strain_accel_intercept_roll = 0.3964f;
    
    const float strain_accel_weights_pitch[12] = {
        22.8082f, 11.2027f, -0.823509f,
        3.00961f, 18.3754f, -7.51376f,
        -18.4098f, 4.68598f, 14.1807f,
        -4.97128f, -6.03368f, -14.6994f
    };
    const float strain_accel_intercept_pitch = 0.1534f;

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

    struct angular_accel_strain{
        float roll_accel;
        float pitch_accel;
    } accel_strain;

};

namespace AP {
    AP_Strain &strain();
};
