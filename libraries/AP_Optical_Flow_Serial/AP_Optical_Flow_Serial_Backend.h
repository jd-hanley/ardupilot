#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

namespace AP_Optical_Flow_Serial
{
    enum class Status
    {
        NotConnected = 0,
        NoData,
        OutOfRange,
        Good
    };

    struct AP_Optical_Flow_Serial_State
    {
        float measurement;
        uint32_t last_reading_ms;
        Status status;
        uint8_t signal_quality_pct;
    };
}

class AP_Optical_Flow_Backend_Serial
{

    public:
    // Add constructor
    AP_Optical_Flow_Backend_Serial();

    // Add init_serial
    // Note that we are going to get our UARTDriver object directly from hal.serial()

    protected:
    
    // Set initial baudrate
    // Set rx_bufsize
    // Set tx_bufsize

    // Create pointer to UART object
    AP_HAL::UARTDriver *uart = nullptr;

    // Add update function
    // Add get_reading function

}