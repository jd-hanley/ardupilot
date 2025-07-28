#pragma once

#define MAX_INSTANCES 4

class AP_Centeye_Nano_Backend;

class AP_Centeye_Nano
{
    public:

    // Constructor
    AP_Centeye_Nano();

    // Do not allow copies

    // Possibly implement get_singleton()

    // Initialize the object, loading the backend drivers
    void init();

    enum class Status 
    {
        NotConnected   = 0,
        NoData         = 1,
        Good           = 2,
    };

    // Data accessor methods go here

    // Other methods go here: Calibration, Reset, Status Check, etc. TBD what is necessary for this sensor


    private:

    // singleton
    static AP_Centeye_Nano* singleton;

    // Array of backend drivers 
    AP_Centeye_Nano_Backend* drivers[MAX_INSTANCES];


    struct sensor
    {
        // Structure of the sensor struct TBD until format of data and desired data is finalized

    };  
    
    sensor sensors[MAX_INSTANCES];



};