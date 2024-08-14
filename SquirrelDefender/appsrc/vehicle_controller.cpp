/********************************************************************************
 * @file    vehicle_controller.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Control the position, velocity, and acceleration of the drone by
 *          sending the following MAVLINK message to the drone.  Control the
 *          vector position, velocity, acceleration, and yaw/yaw rate.
 *
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "vehicle_controller.h"
#include "sim_flight_test_4_VelocityControl.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
DebugTerm VehStateInfo("");
PID height_ctrl;

bool takeoff_dbc;
uint16_t takeoff_dbc_cnt;
float z_adjust;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: VehicleController
 * Description: Constructor of the VehicleController class.
 ********************************************************************************/
VehicleController::VehicleController(void) {}

/********************************************************************************
 * Function: VehicleController
 * Description: Constructor of the VehicleController class.
 ********************************************************************************/
VehicleController::~VehicleController(void) {}

/********************************************************************************
 * Function: cmd_position_NED
 * Description: Move to an x,y,z coordinate in the NED frame.
 ********************************************************************************/
void VehicleController::cmd_position_NED(float position_target[3])
{
    VelocityController::cmd_position_NED(position_target);
}

/********************************************************************************
 * Function: cmd_velocity_NED
 * Description: Move in direction of vector vx,vy,vz in the NED frame.
 ********************************************************************************/
void VehicleController::cmd_velocity_NED(float velocity_target[3])
{
    VelocityController::cmd_velocity_NED(velocity_target);
}

/********************************************************************************
 * Function: cmd_velocity_xy_NED
 * Description: Move in xy plane given a vector vx,vy in the NED frame.
 ********************************************************************************/
void VehicleController::cmd_velocity_xy_NED(float velocity_target[3])
{
    VelocityController::cmd_velocity_xy_NED(velocity_target);
}

/********************************************************************************
 * Function: cmd_velocity_x_NED
 * Description: Move in direction of vector vx in the NED frame.
 ********************************************************************************/
void VehicleController::cmd_velocity_x_NED(float velocity_target)
{
    VelocityController::cmd_velocity_x_NED(velocity_target);
}

/********************************************************************************
 * Function: cmd_velocity_y_NED
 * Description: Move in direction of vector vy in the NED frame.
 ********************************************************************************/
void VehicleController::cmd_velocity_y_NED(float velocity_target)
{
    VelocityController::cmd_velocity_y_NED(velocity_target);
}

/********************************************************************************
 * Function: cmd_acceleration_NED
 * Description: Move in direction of vector ax,ay,az in the NED frame.
 ********************************************************************************/
void VehicleController::cmd_acceleration_NED(float acceleration_target[3])
{
    VelocityController::cmd_acceleration_NED(acceleration_target);
}

#ifdef USE_JETSON

/********************************************************************************
 * Function: follow_target
 * Description: Move in direction of vector ax,ay,az in the NED frame.
 ********************************************************************************/
void VehicleController::follow_mode(void)
{
    float target_velocity[3] = {0.0, 0.0, 0.0};

    Follow::follow_control_loop();

    target_velocity[0] = vx_adjust;
    target_velocity[1] = vy_adjust;
    target_velocity[2] = vz_adjust;

    if (target_too_close)
    {
        VehicleController::cmd_velocity_xy_NED(target_velocity);
    }
    else
    {
        VehicleController::cmd_velocity_NED(target_velocity);
    }
}

/********************************************************************************
 * Function: height_control
 * Description: Keep drone height at a set point.
 ********************************************************************************/
void VehicleController::height_control(void)
{
    float target_velocity[3] = {0.0, 0.0, 0.0};
    const float w1 = 1.0;
    const float Kp_z_height = 0.001;
    const float Ki_z_height = 0.0;
    const float Kd_z_height = 0.0;
    const uint16_t veh_height_target = (uint16_t)200;
    uint16_t veh_height_err = (uint16_t)0;
    bool veh_too_high = false;

    veh_too_high = (mav_veh_rngfdr_current_distance >= veh_height_target);

    DebugTerm rngfndr("/dev/pts/5");

    /*
    Need to account for max and min values of the rangefinder or gps.
    Max value of Terabee-60m I2C has max range of 6000cm, and min detectable of of 50cm.
    Need to filter out values above this or jumps to 6000cm or higher (occurs when sensor gets a bad
    reading due to the surface quality).
    Need to account for no resolution below 50cm.
    */

    if (mav_veh_rngfdr_current_distance > (uint16_t)0 && veh_too_high == true)
    {
        veh_height_err = mav_veh_rngfdr_current_distance - veh_height_target;
    }
    else if (mav_veh_rngfdr_current_distance > (uint16_t)0 && veh_too_high == false)
    {
        veh_height_err = veh_height_target - mav_veh_rngfdr_current_distance;
    }
    else
    {
        veh_height_err = (uint16_t)0;
    }

    /* remember in NED frame down is positive, and up is negative */
    if (veh_too_high)
    {
        z_adjust = height_ctrl.pid_controller_3d(Kp_z_height, Ki_z _height, Kd_z_height,
                                                 (float)veh_height_err, 0.0, 0.0,
                                                 w1, 0.0, 0.0, CONTROL_DIM::Z);
    }
    else
    {
        z_adjust = -height_ctrl.pid_controller_3d(Kp_z_height, Ki_z_height, Kd_z_height,
                                                  (float)veh_height_err, 0.0, 0.0,
                                                  w1, 0.0, 0.0, CONTROL_DIM::Z);
    }

    target_velocity[2] = z_adjust;

    rngfndr.cpp_cout(std::to_string(target_velocity[2]));

    VehicleController::cmd_velocity_NED(target_velocity);
}

#endif // USE_JETSON

/********************************************************************************
 * Function: vehicle_control_init
 * Description: Initial setup of vehicle controller.
 ********************************************************************************/
bool VehicleController::vehicle_control_init(void)
{
    takeoff_dbc = false;
    takeoff_dbc_cnt = 200;
    z_adjust = (float)0.0;

    return true;
}

/********************************************************************************
 * Function: vehicle_control_loop
 * Description: Vehicle controller main loop.  Handles all
 ********************************************************************************/
void VehicleController::vehicle_control_loop(void)
{
    DebugTerm rngfndr("/dev/pts/5");

    rngfndr.cpp_cout("Height rangefinder, Height gps: " + std::to_string(mav_veh_rngfdr_current_distance) + ", " + std::to_string(mav_veh_rel_alt));

    if (system_state == SYSTEM_STATE::INIT)
    {
        MavCmd::set_mode_GUIDED();
    }
    else if (system_state == SYSTEM_STATE::PRE_ARM_GOOD)
    {
        MavCmd::arm_vehicle();
    }
    else if (system_state == SYSTEM_STATE::STANDBY)
    {
        MavCmd::takeoff_GPS_long((float)3.5);
    }
    else if (system_state == SYSTEM_STATE::IN_FLIGHT_GOOD)
    {
#ifdef USE_JETSON

        if (takeoff_dbc_cnt > 0)
        {
            takeoff_dbc_cnt--;
        }
        else
        {
            takeoff_dbc_cnt = 0;
            takeoff_dbc = true;
        }

        if (takeoff_dbc)
        {
            height_control();
        }

#elif USE_WSL

        if (takeoff_dbc_cnt > 0)
        {
            takeoff_dbc_cnt--;
        }
        else
        {
            takeoff_dbc_cnt = 0;
            takeoff_dbc = true;
        }

        if (takeoff_dbc)
        {
            test_flight();
        }

#endif // USE_JETSON
    }
}

/********************************************************************************
 * Function: vehicle_control_shutdown
 * Description: Code needed to shutdown the vehicle controller.
 ********************************************************************************/
void VehicleController::vehicle_control_shutdown(void)
{
    MavCmd::set_mode_LAND();
}