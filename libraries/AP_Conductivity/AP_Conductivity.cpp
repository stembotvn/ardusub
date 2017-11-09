#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Conductivity.h"

extern const AP_HAL::HAL &hal;

static const uint8_t ATLAS_DEVICE_ADDRESS               = 0x64;
static const uint8_t ATLAS_REG_DEVICE_TYPE              = 0x00;
static const uint8_t ATLAS_REG_FIRMWARE_VERSION         = 0x01;
static const uint8_t ATLAS_REG_ADDRESS_LOCK_UNLOCK      = 0x02;
static const uint8_t ATLAS_REG_ADDRESS                  = 0x03;
static const uint8_t ATLAS_REG_INTERRUPT_CONTROL        = 0x04;
static const uint8_t ATLAS_REG_LED_CONTROL              = 0x05;
static const uint8_t ATLAS_REG_ACTIVE_HYBERNATE         = 0x06;
static const uint8_t ATLAS_REG_NEW_READING              = 0x07;
static const uint8_t ATLAS_REG_PROBE_TYPE               = 0x09;
static const uint8_t ATLAS_REG_CALIBRATION              = 0x0A;
static const uint8_t ATLAS_REG_CALIBRATION_REQUEST      = 0x0E;
static const uint8_t ATLAS_REG_CALIBRATION_CONFIRMATION = 0x0F;
static const uint8_t ATLAS_REG_TEMPERATURE_COMPENSATION = 0x10;
static const uint8_t ATLAS_REG_TEMPERATURE_CONFIRMATION = 0x12;
static const uint8_t ATLAS_REG_EC_READING               = 0x18;
static const uint8_t ATLAS_REG_TDS_READING              = 0x1C;
static const uint8_t ATLAS_REG_PSS_READING              = 0x20;

// table of user settable parameters
const AP_Param::GroupInfo AP_Conductivity::var_info[] = {
    // @Param: K
    // @DisplayName: Sensor K
    // @Description: Conductivity sensor K factor
    // @Increment: 0.1
    // @User: Advanced
    // @RequiresReboot: true
    AP_GROUPINFO("K", 1, AP_Conductivity, _k, 1.0f),

    AP_GROUPINFO("T_COMP",2, AP_Conductivity, _compensate_temperature, false),

    AP_GROUPINFO("DEBUG", 3, AP_Conductivity, _debug, true),

    AP_GROUPEND
};

AP_Conductivity::AP_Conductivity() :
        _dev(nullptr),
        _temperature(2500)
{
    AP_Param::setup_object_defaults(this, var_info);

}

bool AP_Conductivity::init()
{
    hal.console->printf("\nChecking for Atlas Conductivity\nk is %.2f\n", _k.get());

    _dev = std::move(hal.i2c_mgr->get_device(1, ATLAS_DEVICE_ADDRESS));
    if (!_dev) {
        hal.console->printf("Atlas device is null!\n");
        return false;
    }

    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        AP_HAL::panic("PANIC: Conductivity: failed to take serial semaphore for init");
    }

    _dev->set_retries(10);

    uint16_t cmd;

    // Wake up the device
    cmd = ATLAS_REG_ACTIVE_HYBERNATE << 8 | 0x01;
    if (!_dev->transfer((uint8_t*)&cmd, 1, nullptr, 0)) {
        hal.console->printf("Conductivity: wake failed\n");
        _dev->get_semaphore()->give();
        return false;
    }


    // Read device type
    if (!_dev->transfer(&ATLAS_REG_DEVICE_TYPE, 1, &_device_type, 1)) {
        hal.console->printf("Conductivity: device type read failed\n");
        _dev->get_semaphore()->give();
        return false;
    }

    hal.console->printf("Conductivity: device type is %d", _device_type);

    // Read firmware version
    if (!_dev->transfer(&ATLAS_REG_FIRMWARE_VERSION, 1, &_firmware_version, 1)) {
        hal.console->printf("Conductivity: firmware version read failed\n");
        _dev->get_semaphore()->give();
        return false;
    }

    hal.console->printf("Conductivity: firmware version is %d\n", _firmware_version);

    // Disable interrupt control
    cmd = ATLAS_REG_INTERRUPT_CONTROL << 8 | 0x00;
    if (!_dev->transfer((uint8_t*)&cmd, 2, nullptr, 0)) {
        hal.console->printf("Conductivity: interrupt control failed\n");
        _dev->get_semaphore()->give();
        return false;
    }

    // Disable LED control
    cmd = ATLAS_REG_LED_CONTROL << 8 | 0x00;
    if (!_dev->transfer((uint8_t*)&cmd, 2, nullptr, 0)) {
        hal.console->printf("Conductivity: LED control failed\n");
        _dev->get_semaphore()->give();
        return false;
    }

    // Set K type of probe
    struct PACKED {
        uint8_t reg;
        uint16_t k;
    } command = { ATLAS_REG_PROBE_TYPE, (uint16_t)(_k * 100.0f) } ;
    if (!_dev->transfer((uint8_t*)&command, sizeof(command), nullptr, 0)) {
        hal.console->printf("Conductivity: probe type control failed\n");
        _dev->get_semaphore()->give();
        return false;
    }

    // lower retries for run
    _dev->set_retries(3);

    _dev->get_semaphore()->give();

    /* Request 1Hz update */
    if (!_dev->register_periodic_callback(1000 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&AP_Conductivity::_update, void))) {
        hal.console->printf("Conductivity: periodic callback registration failed");
    }
    return true;
}

void AP_Conductivity::_update()
{
    printf("Conductivity: Reading Atlas\n");

    _healthy = false;

    uint8_t new_reading;

    if (!_dev->transfer(&ATLAS_REG_NEW_READING, 1, &new_reading, 1)) {
        printf("Conductivity: new reading failed\n");
        _healthy = false;
        return;
    }

    // No new reading
    if (!new_reading) {
        printf("Conductivity: no new reading\n");
        return;
    }

    if (!_dev->transfer(&ATLAS_REG_EC_READING, 1, (uint8_t*)&raw_ec, 4)) {
        printf("Conductivity: EC read failed\n");
        return;
    }

    if (!_dev->transfer(&ATLAS_REG_TDS_READING, 1, (uint8_t*)&raw_tds, 4)) {
        printf("Conductivity: TDS read failed\n");
        return;
    }

    if (!_dev->transfer(&ATLAS_REG_PSS_READING, 1, (uint8_t*)&raw_pss, 4)) {
        printf("Conductivity: PSS read failed\n");
        return;
    }

    printf("Conductivity: ec %d\t tds %d\t pss %d\n", raw_ec, raw_tds, raw_pss);

    if (_compensate_temperature) {
        printf("Conductivity: compensating temperature %d\n", _temperature);
        struct PACKED {
            uint8_t reg;
            uint32_t temperature;
        } command = { ATLAS_REG_TEMPERATURE_COMPENSATION, _temperature } ;
        if (!_dev->transfer((uint8_t*)&command, sizeof(command), nullptr, 0)) {
            printf("Conductivity: temperature compensation failed\n");
            return;
        }
    }

    // Reset new reading flag
    uint16_t cmd = ATLAS_REG_NEW_READING << 8 | 0x00;
    if (!_dev->transfer((uint8_t*)&cmd, 2, nullptr, 0)) {
        printf("Conductivity: interrupt control failed\n");
        return;
    }

    _healthy = true;
}
