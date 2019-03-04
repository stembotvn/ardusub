
#include <AP_AHRS/AP_AHRS.h>
//#include <AP_Baro/AP_Baro.h>
//#include <stdio.h>

class AS_PositionControl
{

public:
    AS_PositionControl(AP_AHRS& ahrs_) : ahrs(ahrs_) {}

    void set_target_x(float target_x);
    void set_target_y(float target_y);
    void set_target_z(float target_z) {
        target_z_ = target_z;
    };
    void update()
    {
        output_z = 0;
        output_z += p * get_error_z();
    };

    float get_target_z() { return target_z_; }
    float get_output_x();
    float get_output_y();
    float get_output_z() { return output_z; }
    float get_error_x();
    float get_error_y();
    float get_error_z()
    {
        return target_z_ - ahrs.get_baro().get_altitude();
    }


private:
    float p = 1.0f, i, d;
    // meters!
    float target_z_, target_x, target_y, output_x, output_y, output_z;
    const AP_AHRS& ahrs;

};
