
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>

//#include <AP_Baro/AP_Baro.h>
#include <stdio.h>

class AS_PositionControl
{

public:
    AS_PositionControl(AP_AHRS& ahrs) : ahrs_(ahrs)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    void set_target_position(Vector3f target) { target_position = target; }
    void set_target_velocity(Vector3f target) { target_velocity = target; }
    void set_target_acceleration(Vector3f target) { target_acceleration = target; }

    Vector3f get_target_position() { return target_position; }
    Vector3f get_target_velocity() { return target_velocity; }
    Vector3f get_target_acceleration() { return target_acceleration; }

    Vector3f get_error_position() { return target_position - get_current_position(); }
    Vector3f get_error_velocity() { return target_velocity - get_current_velocity(); }
    Vector3f get_error_acceleration() { return target_acceleration - get_current_acceleration(); }

    Vector3f get_current_position() { return Vector3f { 0, 0, ahrs_.get_baro().get_altitude() }; }
    //return ahrs_.get_relative_position_NED_origin();
    Vector3f get_current_velocity() { Vector3f a; ahrs_.get_velocity_NED(a); return a;}
    Vector3f get_current_acceleration() { return Vector3f {0, 0, -(ahrs_.get_accel_ef_blended().z + GRAVITY_MSS)}; }

    void update()
    {
        //update_position_controller();
        update_velocity_controller();
        //update_acceleration_controller();
        //printf("target_acc: %f, current_acc: %f, acc_error: %f, target_speed: %f, current speed: %f, command.z: %f\n", target_acceleration.z, get_current_acceleration().z, get_error_acceleration().z , target_velocity.z, get_current_velocity().z, output_command.z);

    }

    Vector3f get_output_command() { return output_command; }

    // parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    AC_PID pid_position_x { 1.0f, 0, 0, 100.0f, 100.0f, 0.01f };
    AC_PID pid_velocity_x { 1.0f, 0, 0, 100.0f, 100.0f, 0.01f };
    AC_PID pid_acceleration_x { 1.0f, 0, 0, 100.0f, 100.0f, 0.01f };

    AC_PID pid_position_y { 1.0f, 0, 0, 100.0f, 100.0f, 0.01f };
    AC_PID pid_velocity_y { 1.0f, 0, 0, 100.0f, 100.0f, 0.01f };
    AC_PID pid_acceleration_y { 1.0f, 0, 0, 100.0f, 100.0f, 0.01f };

    AC_PID pid_position_z { 1.0f, 0.0f, 0.0f, 1.0f, 100.0f, 0.01f };
    AC_PID pid_velocity_z { 25.0f, 50.0f, 0.0f, 600.0f, 100.0f, 0.01f };
    AC_PID pid_acceleration_z { 1.2f, 0.0f, 0.0f, 1.0f, 100.0f, 0.01f };

    // meters!
    Vector3f target_position = { 0, 0, 0 };
    Vector3f target_velocity = { 0, 0, 0 };
    Vector3f target_acceleration = { 0, 0, 0 };
    Vector3f output_command = { 0, 0, 0 };

    const AP_AHRS& ahrs_;

    void update_position_controller()
    {
        pid_position_z.set_input_filter_all(get_error_position().z);
        //printf("error_position: %f\n", get_error_position().z);
        target_velocity.z = pid_position_z.get_pid();
    }

    void update_velocity_controller()
    {
        pid_velocity_z.set_input_filter_all(-get_error_velocity().z);
        target_acceleration.z = pid_velocity_z.get_pid();
        output_command.z = target_acceleration.z;
    }

    void update_acceleration_controller()
    {
        pid_acceleration_z.set_input_filter_all(get_error_acceleration().z);
        //output_command.z = pid_velocity_z.get_pid();

        output_command.x = constrain_float(output_command.x, -1.0f, 1.0f);
        output_command.y = constrain_float(output_command.y, -1.0f, 1.0f);
        output_command.z = constrain_float(output_command.z, -1.0f, 1.0f);
    }
};
