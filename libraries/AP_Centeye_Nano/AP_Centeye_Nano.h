#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <Filter/DerivativeFilter.h>
#include <AP_MSP/msp.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

#define OFLOW_MAX_INSTANCES 1
#define MATRIX_SIZE 4
#define BUS_NUMBER 0
#define OBJDET_LENGTH 16

class AP_Centeye_Nano_Backend;

class AP_Centeye_Nano
{

    friend class AP_Centeye_Nano_Backend;
    public:

    // Constructor
    AP_Centeye_Nano();

    // Do not allow copies
    CLASS_NO_COPY(AP_Centeye_Nano);

    // get singleton - returns a pointer to the current instance of the class
    static AP_Centeye_Nano *get_singleton(void) 
    {
        return _singleton;
    }

    // Initialize the object, loading the backend drivers
    void init();

    enum class Status 
    {
        NotConnected   = 0,
        NoData         = 1,
        Good           = 2,
    };

    // Other methods go here: Calibration, Reset, Status Check, etc. TBD what is necessary for this sensor

    // Data accessor methods go here... need to think about how we want to access the data
    // Can return a pointer to the sensor data structure, copy data into an auxiliary structure, etc
    int32_t get_odo_x();


    private:

    // singleton
    static AP_Centeye_Nano* _singleton;

    // Array of backend drivers 
    AP_Centeye_Nano_Backend* drivers[OFLOW_MAX_INSTANCES];

    bool init_done = false;


    struct sensor
    {
        // int32_t odom_x;
        // int32_t odom_y;
        // int32_t odom_div;
        int32_t objdet_h[OBJDET_LENGTH];
        // int32_t objdet_v[MATRIX_SIZE][MATRIX_SIZE];
    };  
    
    sensor sensors[OFLOW_MAX_INSTANCES];

    // Update method
    bool update_from_backend();

    // Array of sensor statuses to track the health of each sensor
    Status sensor_status[OFLOW_MAX_INSTANCES];;

    // Number of sensors
    uint8_t num_sensors;

};