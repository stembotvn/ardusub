#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>
#include <AP_Param/AP_Param.h>



class AP_Conductivity {
public:
    AP_Conductivity();

    bool init(void);
    bool healthy(void) { return _healthy; };

    static const struct AP_Param::GroupInfo var_info[];

    void set_temperature(uint32_t temperature) { _temperature = temperature; }; // centidegrees

    float get_ec(void) { return raw_ec * 100.0f; };
    float get_tds(void) { return raw_tds * 100.0f; };
    float get_pss(void) { return raw_pss * 100.0f; };

private:
    void _update(void);

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    bool _healthy;
    uint8_t _device_type;
    uint8_t _firmware_version;

    uint32_t _temperature; // centidegrees, Write here to compensate, default value is 2500 (25C)

    uint32_t raw_ec; // microsiemens * 100
    uint32_t raw_tds;
    uint32_t raw_pss;

    AP_Float _k;
    AP_Int8 _compensate_temperature;
    AP_Int8 _debug;
};
