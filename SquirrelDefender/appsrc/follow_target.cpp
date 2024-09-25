#ifdef ENABLE_CV

/********************************************************************************
 * @file    follow_target.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Follow the target and maintain a specified x, y, z offset.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "follow_target.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
DebugTerm FollowData("");
PID pid_forwd;
PID pid_rev;

bool target_too_close;
bool target_identified;
float x_actual;
float height_actual;
float y_actual;
float width_actual;
float x_centroid_err;
float target_height_err;
float y_centroid_err;
float target_left_side;
float target_right_side;
float target_left_err;
float target_right_err;
float target_height_err_rev;

float vx_adjust;
float vy_adjust;
float vz_adjust;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
float Kp_x;
float Ki_x;
float Kd_x;
float Kp_y;
float Ki_y;
float Kd_y;
float w1_x;
float w2_x;
float w3_x;
float w1_y;
float w2_y;
float w3_y;
float w1_z;
float w2_z;
float w3_z;
float Kp_x_rev;
float Ki_x_rev;
float Kd_x_rev;
float w1_x_rev;
float w2_x_rev;
float w3_x_rev;
float Kp_y_rev;
float Ki_y_rev;
float Kd_y_rev;
float w1_y_rev;
float w2_y_rev;
float w3_y_rev;
uint16_t vehicle_rel_height_err;
uint16_t vehicle_height_desired;
float x_desired;
float target_height_desired;
float y_desired;
float target_width_desired;

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: Follow
 * Description: Follow class constructor.
 ********************************************************************************/
Follow::Follow(void) {};

/********************************************************************************
 * Function: ~Follow
 * Description: Follow class destructor.
 ********************************************************************************/
Follow::~Follow(void) {};

/********************************************************************************
 * Function: get_control_params
 * Description: Read follow control parameters from a json or other file type.
 ********************************************************************************/
void Follow::get_control_params(void)
{
#ifdef DEBUG_BUILD

    Parameters veh_params("../params.json");
    // Accessing Vel_PID_x parameters
    Kp_x = veh_params.get_float_param("Vel_PID_x", "Kp");
    Ki_x = veh_params.get_float_param("Vel_PID_x", "Ki");
    Kd_x = veh_params.get_float_param("Vel_PID_x", "Kd");
    w1_x = veh_params.get_float_param("Vel_PID_x", "w1");
    w2_x = veh_params.get_float_param("Vel_PID_x", "w2");
    w3_x = veh_params.get_float_param("Vel_PID_x", "w3");

    // Accessing Vel_PID_y parameters
    Kp_y = veh_params.get_float_param("Vel_PID_y", "Kp");
    Ki_y = veh_params.get_float_param("Vel_PID_y", "Ki");
    Kd_y = veh_params.get_float_param("Vel_PID_y", "Kd");
    w1_y = veh_params.get_float_param("Vel_PID_y", "w1");
    w2_y = veh_params.get_float_param("Vel_PID_y", "w2");
    w3_y = veh_params.get_float_param("Vel_PID_y", "w3");

    // Accessing Vel_PID_x parameters for reverse movement
    Kp_x_rev = veh_params.get_float_param("Vel_PID_x_reverse", "Kp");
    Ki_x_rev = veh_params.get_float_param("Vel_PID_x_reverse", "Ki");
    Kd_x_rev = veh_params.get_float_param("Vel_PID_x_reverse", "Kd");
    w1_x_rev = veh_params.get_float_param("Vel_PID_x_reverse", "w1");
    w2_x_rev = veh_params.get_float_param("Vel_PID_x_reverse", "w2");
    w3_x_rev = veh_params.get_float_param("Vel_PID_x_reverse", "w3");

    // Accessing Vel_PID_y parameters for reverse movment
    Kp_y_rev = veh_params.get_float_param("Vel_PID_y_reverse", "Kp");
    Ki_y_rev = veh_params.get_float_param("Vel_PID_y_reverse", "Ki");
    Kd_y_rev = veh_params.get_float_param("Vel_PID_y_reverse", "Kd");
    w1_y_rev = veh_params.get_float_param("Vel_PID_y_reverse", "w1");
    w2_y_rev = veh_params.get_float_param("Vel_PID_y_reverse", "w2");
    w3_y_rev = veh_params.get_float_param("Vel_PID_y_reverse", "w3");

#else

    // Vel_PID_x parameters for forward movement
    Kp_x = 0.025;
    Ki_x = 0.0009;
    Kd_x = 0.0005;
    w1_x = 0.5;
    w2_x = 0.5;
    w3_x = 0.0;

    // Vel_PID_y parameters for forward movement
    Kp_y = 0.0001;
    Ki_y = 0.00009;
    Kd_y = 0.00005;
    w1_y = 1.0;
    w2_y = 0.0;
    w3_y = 0.0;

    // Vel_PID_x parameters for reverse movement
    Kp_x_rev = 0.01;
    Ki_x_rev = 0.00009;
    Kd_x_rev = 0.00005;
    w1_x_rev = 1.0;
    w2_x_rev = 0.0;
    w3_x_rev = 0.0;

    // Vel_PID_y parameters for reverse movement
    Kp_y_rev = 0.001;
    Ki_y_rev = 0.00009;
    Kd_y_rev = 0.00005;
    w1_y_rev = 1.0;
    w2_y_rev = 0.0;
    w3_y_rev = 0.0;

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: get_desired_target_size
 * Description: Read follow target parameters from a json or other file type.
 ********************************************************************************/
void Follow::get_desired_target_size(void)
{
#ifdef DEBUG_BUILD

    Parameters target_params("../params.json");

    x_desired = 0.0;//static_cast<float>(input_video_height) / 2.0;
    y_desired = 0.0;//static_cast<float>(input_video_width) / 2.0;
    target_height_desired = target_params.get_float_param("Target", "Desired_Height");
    target_width_desired = target_params.get_float_param("Target", "Desired_Width");

#else

    x_desired = 0.0;// static_cast<float>(input_video_height) / 2.0;
    y_desired = 0.0;//static_cast<float>(input_video_width) / 2.0;
    target_height_desired = (float)650;
    target_width_desired = (float)150;
    vehicle_height_desired = 150;

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: calc_target_size
 * Description: Calculate the parameters of a target.
 ********************************************************************************/
void Follow::calc_target_size(int n)
{
    x_actual = 0.0;// detections[n].Height() / 2.0 + detections[n].Top;
    height_actual = 0.0;//detections[n].Height();
    y_actual = 0.0;//detections[n].Width() / 2.0 + detections[n].Left;
    width_actual = 0.0;//detections[n].Width();
    target_left_side = 0.0;//detections[n].Left;
    target_right_side = 0.0;//detections[n].Right;
}

/********************************************************************************
 * Function: error_zero_protection
 * Description: Calculate the error between a desired output and the actual,
                return 0 if the actual is below a threshold.
 ********************************************************************************/
float Follow::error_zero_protection(float desired, float actual, float threshold)
{
    if (actual < threshold)
    {
        return 0;
    }
    else
    {
        return (desired - actual);
    }
}

/********************************************************************************
 * Function: calc_follow_error
 * Description: Calculate the error of a target's position based
 ********************************************************************************/
void Follow::calc_follow_error(void)
{
    float bounding_box_left_side = 320.0;
    float bounding_box_right_side = 960.0;

    target_left_err = error_zero_protection(bounding_box_left_side, target_left_side, 1.0);
    target_right_err = error_zero_protection(bounding_box_right_side, target_right_side, 1.0);
    x_centroid_err = error_zero_protection(x_desired, x_actual, 1.0);
    target_height_err = error_zero_protection(target_height_desired, height_actual, 1.0);
    y_centroid_err = -error_zero_protection(y_desired, y_actual, 1.0);

    if (mav_veh_rngfdr_current_distance >= mav_veh_rngfdr_max_distance && mav_veh_rngfdr_current_distance >= 50)
    {
        vehicle_rel_height_err = vehicle_height_desired - mav_veh_rngfdr_current_distance;
    }
}

/********************************************************************************
 * Function: dtrmn_target_ID
 * Description: Determine which detected object to follow.
 ********************************************************************************/
int Follow::dtrmn_target_ID(void)
{
    /*
    for (int n = 0; n < numDetections; n++)
    {
        if (detections[n].TrackID >= 0 && detections[n].ClassID == 1 && detections[n].Confidence > 0.5)
        {
            target_identified = true;
            return n;
        }
        else
        {
            target_identified = false;
        }
    }*/

    return -1;
}

/********************************************************************************
 * Function: follow_target_init
 * Description: Initialize all follow target variables.  Run once at the start
 *              of the program.
 ********************************************************************************/
bool Follow::follow_target_init(void)
{

    get_control_params();
    get_desired_target_size();

    target_too_close = false;
    target_identified = false;
    x_actual = 0.0;
    height_actual = 0.0;
    y_actual = 0.0;
    width_actual = 0.0;
    x_centroid_err = 0.0;
    target_height_err = 0.0;
    y_centroid_err = 0.0;
    target_left_side = 0.0;
    target_right_side = 0.0;
    target_left_err = 0.0;
    target_right_err = 0.0;
    target_height_err_rev = 0.0;
    vehicle_rel_height_err = 0;

    vx_adjust = 0.0;
    vy_adjust = 0.0;
    vz_adjust = 0.0;

    return true;
}

/********************************************************************************
 * Function: follow_control_loop
 * Description: Return control parameters for the vehicle to follow a designated
 *              target at a distance.
 ********************************************************************************/
void Follow::follow_control_loop(void)
{
    int target_ID = -1;

    get_control_params();
    get_desired_target_size();

    target_ID = dtrmn_target_ID();

    if (target_identified)
    {
        calc_target_size(target_ID);
        calc_follow_error();

        target_too_close = (height_actual > target_height_desired);

        if (target_too_close)
        {
            vx_adjust = pid_rev.pid_controller_3d(Kp_x_rev, Ki_x_rev, Kd_x_rev,
                                                  target_height_err, 0.0, 0.0,
                                                  w1_x_rev, 0.0, 0.0, CONTROL_DIM::X);
            vy_adjust = pid_rev.pid_controller_3d(Kp_y_rev, Ki_y_rev, Kd_y_rev,
                                                  y_centroid_err, 0.0, 0.0,
                                                  w1_y_rev, 0.0, 0.0, CONTROL_DIM::Y);
        }
        else
        {
            vx_adjust = pid_forwd.pid_controller_3d(Kp_x, Ki_x, Kd_x,
                                                    x_centroid_err, target_height_err, 0.0,
                                                    w1_x, w2_x, 0.0, CONTROL_DIM::X);
            vy_adjust = pid_forwd.pid_controller_3d(Kp_y, Ki_y, Kd_y,
                                                    y_centroid_err, 0.0, 0.0,
                                                    w1_y, 0.0, 0.0, CONTROL_DIM::Y);
        }
    }
}

#endif // ENABLE_CV