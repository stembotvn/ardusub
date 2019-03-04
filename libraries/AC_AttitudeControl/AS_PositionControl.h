
#include <AP_AHRS/AP_AHRS.h>
//#include <AP_Baro/AP_Baro.h>
//#include <stdio.h>

class AS_PositionControl
{

public:
    AS_PositionControl(AP_AHRS& ahrs_) : ahrs(ahrs_) {}

    void set_target_position(Vector3f target);
    void set_target_velocity(Vector3f target);
    void set_target_acceleration(Vector3f target);

    Vector3f get_target_position();
    Vector3f get_target_velocity();
    Vector3f get_target_acceleration();

    Vector3f get_error_position();
    Vector3f get_error_velocity();
    Vector3f get_error_acceleration();

    void update()
    {
        update_position_controller();
        update_velocity_controller();
        update_acceleration_controller();
    };

    Vector3f get_output_command();


private:
    AC_PID pid_position_x;
    AC_PID pid_velocity_x;
    AC_PID pid_acceleration_x;

    AC_PID pid_position_y;
    AC_PID pid_velocity_y;
    AC_PID pid_acceleration_y;

    AC_PID pid_position_z_;
    AC_PID pid_velocity_z;
    AC_PID pid_acceleration_z;

    // meters!
    Vector3f target_position;
    Vector3f target_velocity;
    Vector3f target_acceleration;
    Vector3f output_command;

    const AP_AHRS& ahrs;

    void update_position_controller()
    {
        target_velocity = 0;
        pid_position_z.set_input_filter_all(get_error_position().z);
        

        target_velocity += pid_position_.
    };
    void update_velocity_controller();
    void update_acceleration_controller();
};
