
#include <AP_AHRS/AP_AHRS.h>
//#include <AP_Baro/AP_Baro.h>
//#include <stdio.h>

class AS_PositionControl
{

public:
    AS_PositionControl(AP_AHRS& ahrs_) : ahrs(ahrs_) {}

    void set_target_position(Vector3f target) { target_position = target; }
    void set_target_velocity(Vector3f target) { target_velocity = target; }
    void set_target_acceleration(Vector3f target) { target_acceleration = target; }

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
    AC_PID pid_position_x_;
    AC_PID pid_velocity_x_;
    AC_PID pid_acceleration_x_;

    AC_PID pid_position_y_;
    AC_PID pid_velocity_y_;
    AC_PID pid_acceleration_y_;

    AC_PID pid_position_z_;
    AC_PID pid_velocity_z_;
    AC_PID pid_acceleration_z_;

    // meters!
    Vector3f target_position;
    Vector3f target_velocity;
    Vector3f target_acceleration;
    Vector3f output_command;

    const AP_AHRS& ahrs;

    void update_position_controller()
    {
        pid_position_z.set_input_filter_all(get_error_position().z);
        target_velocity.z = pid_position_z.get_pid();
    };
    void update_velocity_controller()
    {
        pid_velocity_z.set_input_filter_all(get_error_velocity().z);
        target_acceleration.z = pid_velocity_z.get_pid();
    };
    void update_acceleration_controller()
    {
        pid_acceleration_z.set_input_filter_all(get_error_acceleration().z);
        output_command.z = pid_acceleration_z.get_pid();
    }
};
