/*
 *  RangeFinder test code
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Strain/AP_Strain_Backend.h>


const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

void setup();
void loop();

// extern const AP_HAL::HAL& hal;
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_SerialManager serial_manager;
static RangeFinder sonar;

static AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_temp = nullptr;
static AP_Strain strain;

int32_t count = 0;
void setup()
{

    // strain
    hal.scheduler->delay(5000);
    hal.console->printf("Strain test\n");
    strain.init();
    strain.calibrate_all();

}

void loop()
{  

    int32_t* data_arm_0 = strain.get_data(0);
    uint32_t last_update_arm_0 = strain.get_last_update(0);

    int32_t* data_arm_1 = strain.get_data(1);
    uint32_t last_update_arm_1 = strain.get_last_update(1);

    uint8_t num_sensors = strain.get_num_sensors();

    hal.console->printf("start----------------------------\n");
    hal.console->printf("backend count: %u\n", num_sensors);
    hal.console->printf("arm0 ID: %u\n", strain.get_ID(0));
    hal.console->printf("arm1 IDs: %u\n", strain.get_ID(1));

    hal.console->printf("arm1----------------------------\n");
    hal.console->printf("time: %ld\n", last_update_arm_0);
    for (uint8_t i = 0; i < 6; i++)
    {
        hal.console->printf("Strain gauge %d: %ld\n", i+1, data_arm_0[i]);

    }
    hal.console->printf("\n");

    hal.console->printf("arm2----------------------------\n");
    for (uint8_t i = 6; i < 12; i++)
    {
        hal.console->printf("Strain gauge %d: %ld\n", i+1, data_arm_0[i]);

    }
    hal.console->printf("\n");


    hal.console->printf("arm3----------------------------\n");
    hal.console->printf("time: %ld\n", last_update_arm_1);
    for (uint8_t i = 0; i < 6; i++)
    {
        hal.console->printf("Strain gauge %d: %ld\n", i+1, data_arm_1[i]);

    }
    hal.console->printf("\n");

    hal.console->printf("arm4----------------------------\n");
    for (uint8_t i = 6; i < 12; i++)
    {
        hal.console->printf("Strain gauge %d: %ld\n", i+1, data_arm_1[i]);

    }
    hal.console->printf("\n");
    hal.console->printf("-------------------- end \n");

    hal.scheduler->delay(300);
  
    
}
AP_HAL_MAIN();
