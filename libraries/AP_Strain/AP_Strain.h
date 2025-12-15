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
#define BUS_NUMBER = 0
#define SENSOR_SCALE_FACTOR 55
// timeouts for health reporting
#define STRAIN_TIMEOUT_MS                 500     // timeout in ms since last successful read
#define STRAIN_DATA_CHANGE_TIMEOUT_MS    2000     // timeout in ms since first strain gauge reading changed 

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
        NotConnected   = 0,
        NoData         = 1,
        Good           = 2,
    };

    
    int32_t* get_data(uint8_t instance);
    float get_roll_accel_strain();
    void get_arm_averages(float* destination);
    float get_pitch_accel_strain();
    uint8_t get_num_sensors();
    AP_Strain::Status get_status(uint8_t instance);
    uint32_t get_last_update(uint8_t instance);
    uint8_t get_ID(uint8_t instance) { return sensors[instance].I2C_id; }
    
    bool reset_all();
    bool calibrate_all();
    bool get_status_all();

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

    // weights and intercepts for calculating angular acceleration from strain gauges
    const float strain_accel_weights_roll[12] = {
        -10.0207f, 6.5910f, -14.2687f,      // arm 1
        -17.2856f, 0.0f, 17.1085f,          // arm 2
        4.6589f, 7.9093f, -4.0103f,         // arm 3
        13.0439f, -17.4240f, 1.2953f        // arm 4
    };
    const float strain_accel_intercept_roll = -0.3564f;
    
    const float strain_accel_weights_pitch[12] = {
        17.4908f, -4.13330f, -2.23474f, 
        -0.255422f, 0.0f, 2.21664f, 
        -19.0035f, 17.9168f, 14.3791f, 
        -7.49279f, -15.3344f, -21.0789f
    };
    const float strain_accel_intercept_pitch = -2.7509f;


    uint16_t num_cal = 0; // number of calibrations done

    struct sensor
    {
        uint32_t last_update_ms;        // last update time in ms
        uint32_t last_change_ms;        // last update time in ms that included a change in reading from previous readings
        static const uint8_t num_data = STRAIN_SENSORS;
        int32_t data[num_data];         // 8 strain gauge measurements
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
