/********************************************************************************
 * @file    mavlink_print_info.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Formatted print statements for mavlink messages.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "mavlink_print_info.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: print_heartbeat
 * Description: Print the heartbeat mavlink message.
 ********************************************************************************/
void print_heartbeat(mavlink_heartbeat_t &heartbeat, const char *term)
{
    /* DebugTerm PrintHeartbeat(term);*/

    PrintPass::c_printf("Heartbeat:\n");
    PrintPass::c_printf("\tType: %d\n", heartbeat.type);
    PrintPass::c_printf("\tAutopilot: %d\n", heartbeat.autopilot);
    PrintPass::c_printf("\tBase mode: %d\n", heartbeat.base_mode);
    PrintPass::c_printf("\tCustom mode: %d\n", heartbeat.custom_mode);
    PrintPass::c_printf("\tSystem status: %d\n", heartbeat.system_status);
    PrintPass::c_printf("\tMavlink version: %d\n", heartbeat.mavlink_version);
}

/********************************************************************************
 * Function: print_system_time
 * Description: Print the system time mavlink message.
 ********************************************************************************/
void print_system_time(mavlink_system_time_t &system_time, const char *term)
{
    /* DebugTerm PrintSysTm(term); */

    PrintPass::c_printf("System Time:\n");
    PrintPass::c_printf("\tUnix timestamp (us): %lu\n", system_time.time_unix_usec);
    PrintPass::c_printf("\tTime since boot (ms): %u\n", system_time.time_boot_ms);
}

/********************************************************************************
 * Function: print_sys_status
 * Description: Print the system status mavlink message.
 ********************************************************************************/
void print_sys_status(mavlink_sys_status_t &sys_status, const char *term)
{
    /* DebugTerm PrintSysStat(term); */

    PrintPass::c_printf("System status:\n");
    PrintPass::c_printf("\tOnboard control sensors present: %u\n", sys_status.onboard_control_sensors_present);
    PrintPass::c_printf("\tOnboard control sensors enabled: %u\n", sys_status.onboard_control_sensors_enabled);
    PrintPass::c_printf("\tOnboard control sensors health: %u\n", sys_status.onboard_control_sensors_health);
    PrintPass::c_printf("\tLoad: %u%%\n", sys_status.load);
    PrintPass::c_printf("\tVoltage battery: %u mV\n", sys_status.voltage_battery);
    PrintPass::c_printf("\tCurrent battery: %d mA\n", sys_status.current_battery);
    PrintPass::c_printf("\tBattery remaining: %d%%\n", sys_status.battery_remaining);
    PrintPass::c_printf("\tDrop rate communication: %u%%\n", sys_status.drop_rate_comm);
    PrintPass::c_printf("\tErrors comm: %u\n", sys_status.errors_comm);
    PrintPass::c_printf("\tErrors count 1: %u\n", sys_status.errors_count1);
    PrintPass::c_printf("\tErrors count 2: %u\n", sys_status.errors_count2);
    PrintPass::c_printf("\tErrors count 3: %u\n", sys_status.errors_count3);
    PrintPass::c_printf("\tErrors count 4: %u\n", sys_status.errors_count4);
}

/********************************************************************************
 * Function: print_statustext
 * Description: Print the status text mavlink message.
 ********************************************************************************/
void print_statustext(mavlink_statustext_t &statustext, const char *term)
{
    /* DebugTerm PrintStatTxt(term); */

    PrintPass::c_printf("STATUSTEXT message:\n");
    PrintPass::c_printf("\tSeverity: %u\n", statustext.severity);
    PrintPass::c_printf("\tText: %s\n", statustext.text);
}

/********************************************************************************
 * Function: print_param_value
 * Description: Print the value of a parameter.
 ********************************************************************************/
void print_param_value(mavlink_param_value_t &param_value, const char *term)
{
    /* DebugTerm PrintParamVal(term); */

    PrintPass::c_printf("Parameter Name: %s\n", param_value.param_id);
    PrintPass::c_printf("\tValue: %.4f\n", param_value.param_value);
    PrintPass::c_printf("\tType: %d\n", param_value.param_type);
    PrintPass::c_printf("\tCount: %d\n", param_value.param_count);
    PrintPass::c_printf("\tIndex: %d\n", param_value.param_index);
}

/********************************************************************************
 * Function: print_autopilot_version
 * Description: Print the autopilot version mavlink message.
 ********************************************************************************/
void print_autopilot_version(mavlink_autopilot_version_t &autopilot_version, const char *term)
{
    /* DebugTerm PrintAutopilotVrsn(term); */

    PrintPass::c_printf("Autopilot Version:\n");
    PrintPass::c_printf("\tCapabilities: %lu\n", autopilot_version.capabilities);
    PrintPass::c_printf("\tFlight Software Version: %lu\n", autopilot_version.flight_sw_version);
    PrintPass::c_printf("\tMiddleware Software Version: %lu\n", autopilot_version.middleware_sw_version);
    PrintPass::c_printf("\tOS Software Version: %lu\n", autopilot_version.os_sw_version);
    PrintPass::c_printf("\tBoard Version: %lu\n", autopilot_version.board_version);
    PrintPass::c_printf("\tFlight Custom Version: %lu\n", autopilot_version.flight_custom_version);
    PrintPass::c_printf("\tMiddleware Custom Version: %lu\n", autopilot_version.middleware_custom_version);
    PrintPass::c_printf("\tOS Custom Version: %lu\n", autopilot_version.os_custom_version);
}

/********************************************************************************
 * Function: print_gps_global_origin
 * Description: Print the gps global origin mavlink message.
 ********************************************************************************/
void print_gps_global_origin(mavlink_gps_global_origin_t &gps_global_origin, const char *term)
{
    /* DebugTerm PrintGlobOrig(term); */

    PrintPass::c_printf("GPS_GLOBAL_ORIGIN message:\n");
    PrintPass::c_printf("\tLatitude: %f\n", (double)gps_global_origin.latitude);
    PrintPass::c_printf("\tLongitude: %f\n", (double)gps_global_origin.longitude);
    PrintPass::c_printf("\tAltitude: %f\n", (double)gps_global_origin.altitude);
}

/********************************************************************************
 * Function: print_home_position
 * Description: Print the home position mavlink message.
 ********************************************************************************/
void print_home_position(mavlink_home_position_t &home_position, const char *term)
{
    /* DebugTerm PrintHomePos(term);*/

    PrintPass::c_printf("HOME_POSITION message:\n");
    PrintPass::c_printf("\tLatitude: %f\n", (double)home_position.latitude);
    PrintPass::c_printf("\tLongitude: %f\n", (double)home_position.longitude);
    PrintPass::c_printf("\tAltitude: %f\n", (double)home_position.altitude);
}

/********************************************************************************
 * Function: print_global_position_int
 * Description: Print the global position integer mavlink message.
 ********************************************************************************/
void print_global_position_int(mavlink_global_position_int_t &global_pos_int, const char *term)
{
    /* DebugTerm PrintGPS(term); */

    PrintPass::c_printf("Global position:\n");
    PrintPass::c_printf("\tLatitude: %d degrees (1e-7)\n", global_pos_int.lat);
    PrintPass::c_printf("\tLongitude: %d degrees (1e-7)\n", global_pos_int.lon);
    PrintPass::c_printf("\tAltitude: %d mm\n", global_pos_int.alt);
    PrintPass::c_printf("\tRelative Altitude: %d mm\n", global_pos_int.relative_alt);
    PrintPass::c_printf("\tVelocity X: %d cm/s\n", global_pos_int.vx);
    PrintPass::c_printf("\tVelocity Y: %d cm/s\n", global_pos_int.vy);
    PrintPass::c_printf("\tVelocity Z: %d cm/s\n", global_pos_int.vz);
    PrintPass::c_printf("\tHeading: %d centi-degrees\n", global_pos_int.hdg);
}

/********************************************************************************
 * Function: print_scaled_imu
 * Description: Print the scaled imu mavlink message.
 ********************************************************************************/
void print_scaled_imu(mavlink_scaled_imu_t &scaled_imu, const char *term)
{
    /* DebugTerm PrintScaledIMU(term); */

    PrintPass::c_printf("Scaled IMU data:\n");
    PrintPass::c_printf("\tX acceleration: %d\n", scaled_imu.xacc);
    PrintPass::c_printf("\tY acceleration: %d\n", scaled_imu.yacc);
    PrintPass::c_printf("\tZ acceleration: %d\n", scaled_imu.zacc);
    PrintPass::c_printf("\tX gyro: %d\n", scaled_imu.xgyro);
    PrintPass::c_printf("\tY gyro: %d\n", scaled_imu.ygyro);
    PrintPass::c_printf("\tZ gyro: %d\n", scaled_imu.zgyro);
    PrintPass::c_printf("\tX magnetometer: %d\n", scaled_imu.xmag);
    PrintPass::c_printf("\tY magnetometer: %d\n", scaled_imu.ymag);
    PrintPass::c_printf("\tZ magnetometer: %d\n", scaled_imu.zmag);
}

/********************************************************************************
 * Function: print_optical_flow
 * Description: Print the optical flow mavlink message.
 ********************************************************************************/
void print_optical_flow(mavlink_optical_flow_t &optical_flow, const char *term)
{
    /* DebugTerm PrintOptFlow(term); */

    PrintPass::c_printf("Optical Flow data:\n");
    PrintPass::c_printf("\tTimestamp: %llu us\n", optical_flow.time_usec);
    PrintPass::c_printf("\tSensor ID: %d\n", optical_flow.sensor_id);
    PrintPass::c_printf("\tFlow in x-sensor direction: %d dpix\n", optical_flow.flow_x);
    PrintPass::c_printf("\tFlow in y-sensor direction: %d dpix\n", optical_flow.flow_y);
    PrintPass::c_printf("\tFlow in x-sensor direction (angular-speed compensated): %.2f m/s\n", optical_flow.flow_comp_m_x);
    PrintPass::c_printf("\tFlow in y-sensor direction (angular-speed compensated): %.2f m/s\n", optical_flow.flow_comp_m_y);
    PrintPass::c_printf("\tOptical flow quality / confidence: %d\n", optical_flow.quality);
    PrintPass::c_printf("\tGround distance: %.2f m\n", optical_flow.ground_distance);
    PrintPass::c_printf("\tFlow rate about X axis: %.2f rad/s\n", optical_flow.flow_rate_x);
    PrintPass::c_printf("\tFlow rate about Y axis: %.2f rad/s\n", optical_flow.flow_rate_y);
}

/********************************************************************************
 * Function: print_distance_sensor
 * Description: Print the distance sensor mavlink message.
 ********************************************************************************/
void print_distance_sensor(mavlink_distance_sensor_t &distance_sensor, const char *term)
{
    /* DebugTerm DistSnsrInfo(term); */

    PrintPass::cpp_cout("Distance Sensor data:\n");
    PrintPass::cpp_cout("\tTimestamp: " + std::to_string(distance_sensor.time_boot_ms) + " ms");
    PrintPass::cpp_cout("\tMinimum distance: " + std::to_string(distance_sensor.min_distance) + "cm");
    PrintPass::cpp_cout("\tMaximum distance: " + std::to_string(distance_sensor.max_distance) + "cm");
    PrintPass::cpp_cout("\tCurrent distance reading: " + std::to_string(distance_sensor.current_distance) + "cm");
    PrintPass::cpp_cout("\tType of distance sensor: " + std::to_string(distance_sensor.type));
    PrintPass::cpp_cout("\tOnboard ID of the sensor: " + std::to_string(distance_sensor.id));
    PrintPass::cpp_cout("\tOrientation of the sensor: " + std::to_string(distance_sensor.orientation));
    PrintPass::cpp_cout("\tMeasurement variance: " + std::to_string(distance_sensor.covariance) + "cm^2");
    PrintPass::cpp_cout("\tHorizontal Field of View: " + std::to_string(distance_sensor.horizontal_fov) + "rad");
    PrintPass::cpp_cout("\tVertical Field of View: " + std::to_string(distance_sensor.vertical_fov) + "rad");
    PrintPass::cpp_cout("\tSensor orientation quaternion: " +
                        std::to_string(distance_sensor.quaternion[0]) +
                        std::to_string(distance_sensor.quaternion[1]) +
                        std::to_string(distance_sensor.quaternion[2]) +
                        std::to_string(distance_sensor.quaternion[3]));
    PrintPass::cpp_cout("\tSignal quality: " + std::to_string(distance_sensor.signal_quality));
}

/********************************************************************************
 * Function: print_local_position
 * Description: Print the local position mavlink message.
 ********************************************************************************/
void print_local_position(mavlink_local_position_ned_t &local_position, const char *term)
{
    /* DebugTerm PrintLocPos(term); */

    PrintPass::c_printf("Local Position:\n");
    PrintPass::c_printf("\tTimestamp: %u\n", local_position.time_boot_ms);
    PrintPass::c_printf("\tPosition (meters):\n");
    PrintPass::c_printf("\t\tX: %f\n", local_position.x);
    PrintPass::c_printf("\t\tY: %f\n", local_position.y);
    PrintPass::c_printf("\t\tZ: %f\n", local_position.z);
    PrintPass::c_printf("\tVelocity (m/s):\n");
    PrintPass::c_printf("\t\tVX: %f\n", local_position.vx);
    PrintPass::c_printf("\t\tVY: %f\n", local_position.vy);
    PrintPass::c_printf("\t\tVZ: %f\n", local_position.vz);
}

/********************************************************************************
 * Function: print_position_target_local_ned
 * Description: Print the position_target_local_ned mavlink message.
 ********************************************************************************/
void print_position_target_local_ned(mavlink_position_target_local_ned_t &position_target_local_ned, const char *term)
{
    /* DebugTerm PrintPosTrgt(term); */

    PrintPass::c_printf("Position target local NED: %u\n", position_target_local_ned.time_boot_ms);
    PrintPass::c_printf("\tTime since boot (ms): %u\n", position_target_local_ned.time_boot_ms);
    PrintPass::c_printf("\tPosition (X, Y, Z): %.3f, %.3f, %.3f\n", position_target_local_ned.x, position_target_local_ned.y, position_target_local_ned.z);
    PrintPass::c_printf("\tVelocity (X, Y, Z): %.3f, %.3f, %.3f\n", position_target_local_ned.vx, position_target_local_ned.vy, position_target_local_ned.vz);
    PrintPass::c_printf("\tAcceleration/Force (X, Y, Z): %.3f, %.3f, %.3f\n", position_target_local_ned.afx, position_target_local_ned.afy, position_target_local_ned.afz);
    PrintPass::c_printf("\tYaw setpoint: %.3f radians\n", position_target_local_ned.yaw);
    PrintPass::c_printf("\tYaw rate setpoint: %.3f radians/second\n", position_target_local_ned.yaw_rate);
    PrintPass::c_printf("\tType mask: %u\n", position_target_local_ned.type_mask);
    PrintPass::c_printf("\tCoordinate frame: %d\n", position_target_local_ned.coordinate_frame);
}

/********************************************************************************
 * Function: print_set_position_target_local_ned
 * Description: Print the set_position_target_local_ned mavlink message.
 ********************************************************************************/
void print_set_position_target_local_ned(mavlink_set_position_target_local_ned_t &set_position_target_local_ned, const char *term)
{
    /* DebugTerm PrintSetPosTrgt(term); */

    PrintPass::c_printf("\tTime since boot (ms): %u\n", set_position_target_local_ned.time_boot_ms);
    PrintPass::c_printf("\tPosition (X, Y, Z): %.3f, %.3f, %.3f\n", set_position_target_local_ned.x, set_position_target_local_ned.y, set_position_target_local_ned.z);
    PrintPass::c_printf("\tVelocity (X, Y, Z): %.3f, %.3f, %.3f\n", set_position_target_local_ned.vx, set_position_target_local_ned.vy, set_position_target_local_ned.vz);
    PrintPass::c_printf("\tAcceleration/Force (X, Y, Z): %.3f, %.3f, %.3f\n", set_position_target_local_ned.afx, set_position_target_local_ned.afy, set_position_target_local_ned.afz);
    PrintPass::c_printf("\tYaw setpoint: %.3f radians\n", set_position_target_local_ned.yaw);
    PrintPass::c_printf("\tYaw rate setpoint: %.3f radians/second\n", set_position_target_local_ned.yaw_rate);
    PrintPass::c_printf("\tType mask: %u\n", set_position_target_local_ned.type_mask);
    PrintPass::c_printf("\tTarget system ID: %d\n", set_position_target_local_ned.target_system);
    PrintPass::c_printf("\tTarget component ID: %d\n", set_position_target_local_ned.target_component);
    PrintPass::c_printf("\tCoordinate frame: %d\n", set_position_target_local_ned.coordinate_frame);
}

/********************************************************************************
 * Function: print_attitude
 * Description: Print the attitude mavlink message.
 ********************************************************************************/
void print_attitude(mavlink_attitude_t &attitude, const char *term)
{
    /* DebugTerm PrintAtt(term); */

    PrintPass::c_printf("Attitude:\n");
    PrintPass::c_printf("\tRoll: %.4f radians\n", attitude.roll);
    PrintPass::c_printf("\tPitch: %.4f radians\n", attitude.pitch);
    PrintPass::c_printf("\tYaw: %.4f radians\n", attitude.yaw);
    PrintPass::c_printf("\tRoll speed: %.4f rad/s\n", attitude.rollspeed);
    PrintPass::c_printf("\tPitch speed: %.4f rad/s\n", attitude.pitchspeed);
    PrintPass::c_printf("\tYaw speed: %.4f rad/s\n", attitude.yawspeed);
}

/********************************************************************************
 * Function: print_attitude_target
 * Description: Print the attitude target mavlink message.
 ********************************************************************************/
void print_attitude_target(mavlink_attitude_target_t &attitude_target, const char *term)
{
    /*DebugTerm PrintAttTrgt(term); */

    PrintPass::c_printf("Attitude Target:\n");
    PrintPass::c_printf("\tTime Boot (ms): %lu\n", attitude_target.time_boot_ms);
    PrintPass::c_printf("\tType Mask: %u\n", attitude_target.type_mask);
    PrintPass::c_printf("\tQuaternion (q): [%f, %f, %f, %f]\n", attitude_target.q[0], attitude_target.q[1], attitude_target.q[2], attitude_target.q[3]);
    PrintPass::c_printf("\tBody Roll Rate: %f\n", attitude_target.body_roll_rate);
    PrintPass::c_printf("\tBody Pitch Rate: %f\n", attitude_target.body_pitch_rate);
    PrintPass::c_printf("\tBody Yaw Rate: %f\n", attitude_target.body_yaw_rate);
    PrintPass::c_printf("\tThrust: %f\n", attitude_target.thrust);
}

/********************************************************************************
 * Function: print_set_attitude_target
 * Description: Print the set_attitude_target mavlink message.
 ********************************************************************************/
void print_set_attitude_target(mavlink_set_attitude_target_t &set_attitude_target, const char *term)
{
    /* DebugTerm PrintSetAttTrgt(term); */

    PrintPass::c_printf("Set Attitude Target:\n");
    PrintPass::c_printf("\tTime Boot (ms): %lu\n", set_attitude_target.time_boot_ms);
    PrintPass::c_printf("\tTarget System: %u\n", set_attitude_target.target_system);
    PrintPass::c_printf("\tTarget Component: %u\n", set_attitude_target.target_component);
    PrintPass::c_printf("\tType Mask: %u\n", set_attitude_target.type_mask);
    PrintPass::c_printf("\tQuaternion (q): [%f, %f, %f, %f]\n", set_attitude_target.q[0], set_attitude_target.q[1], set_attitude_target.q[2], set_attitude_target.q[3]);
    PrintPass::c_printf("\tBody Roll Rate: %f\n", set_attitude_target.body_roll_rate);
    PrintPass::c_printf("\tBody Pitch Rate: %f\n", set_attitude_target.body_pitch_rate);
    PrintPass::c_printf("\tBody Yaw Rate: %f\n", set_attitude_target.body_yaw_rate);
    PrintPass::c_printf("\tThrust: %f\n", set_attitude_target.thrust);
}

/********************************************************************************
 * Function: print_attitude_quaternion
 * Description: Print the attitude quaternion mavlink message.
 ********************************************************************************/
void print_attitude_quaternion(mavlink_attitude_quaternion_t &attitude_quaternion, const char *term)
{
    /* DebugTerm PrintAttQuat(term); */

    PrintPass::c_printf("Attitude Quaternion:\n");
    PrintPass::c_printf("\tTime Boot (ms): %ul\n", attitude_quaternion.time_boot_ms);
    PrintPass::c_printf("\tQuaternion (q): [%f, %f, %f, %f]\n", attitude_quaternion.q1, attitude_quaternion.q2, attitude_quaternion.q3, attitude_quaternion.q4);
    PrintPass::c_printf("\tBody Roll Rate: %f\n", attitude_quaternion.rollspeed);
    PrintPass::c_printf("\tBody Pitch Rate: %f\n", attitude_quaternion.pitchspeed);
    PrintPass::c_printf("\tBody Yaw Rate: %f\n", attitude_quaternion.yawspeed);
    PrintPass::c_printf("\tRotation offset: [%f, %f, %f, %f]\n", attitude_quaternion.repr_offset_q[0], attitude_quaternion.repr_offset_q[1], attitude_quaternion.repr_offset_q[2], attitude_quaternion.repr_offset_q[3]);
}

/********************************************************************************
 * Function: print_attitude_quaternion
 * Description: Print the attitude quaternion mavlink message.
 ********************************************************************************/
void print_position_local_ned(const mavlink_local_position_ned_t &local_position, const char *term)
{
    /* DebugTerm PrintLocalPos(term); */

    PrintPass::c_printf("Local Position NED:\n");
    PrintPass::c_printf("\tX Position (m): %f\n", local_position.x);
    PrintPass::c_printf("\tY Position (m): %f\n", local_position.y);
    PrintPass::c_printf("\tZ Position (m): %f\n", local_position.z);
    PrintPass::c_printf("\tX Speed (m/s): %f\n", local_position.vx);
    PrintPass::c_printf("\tY Speed (m/s): %f\n", local_position.vy);
    PrintPass::c_printf("\tZ Speed (m/s): %f\n", local_position.vz);
}

/********************************************************************************
 * Function: print_command_ack
 * Description: Print the mavlink command received by the autopilot.
 ********************************************************************************/
void print_command_ack(mavlink_command_ack_t &command_ack, const char *term)
{
    DebugTerm CmdAck(term);

    PrintPass::cpp_cout("Command ACK received:");
    PrintPass::cpp_cout_oneline("\tCommand: ");

    switch (command_ack.command)
    {
    case MAV_CMD_NAV_WAYPOINT:
        PrintPass::cpp_cout("MAV_CMD_NAV_WAYPOINT");
        break;
    case MAV_CMD_NAV_LOITER_UNLIM:
        PrintPass::cpp_cout("MAV_CMD_NAV_LOITER_UNLIM");
        break;
    case MAV_CMD_NAV_LOITER_TURNS:
        PrintPass::cpp_cout("MAV_CMD_NAV_LOITER_TURNS");
        break;
    case MAV_CMD_NAV_LOITER_TIME:
        PrintPass::cpp_cout("MAV_CMD_NAV_LOITER_TIME");
        break;
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        PrintPass::cpp_cout("MAV_CMD_NAV_RETURN_TO_LAUNCH");
        break;
    case MAV_CMD_NAV_LAND:
        PrintPass::cpp_cout("MAV_CMD_NAV_LAND");
        break;
    case MAV_CMD_NAV_TAKEOFF:
        PrintPass::cpp_cout("MAV_CMD_NAV_TAKEOFF");
        break;
    case MAV_CMD_NAV_LAND_LOCAL:
        PrintPass::cpp_cout("MAV_CMD_NAV_LAND_LOCAL");
        break;
    case MAV_CMD_NAV_TAKEOFF_LOCAL:
        PrintPass::cpp_cout("MAV_CMD_NAV_TAKEOFF_LOCAL");
        break;
    case MAV_CMD_NAV_FOLLOW:
        PrintPass::cpp_cout("MAV_CMD_NAV_FOLLOW");
        break;
    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
        PrintPass::cpp_cout("MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT");
        break;
    case MAV_CMD_NAV_LOITER_TO_ALT:
        PrintPass::cpp_cout("MAV_CMD_NAV_LOITER_TO_ALT");
        break;
    case MAV_CMD_DO_FOLLOW:
        PrintPass::cpp_cout("MAV_CMD_DO_FOLLOW");
        break;
    case MAV_CMD_DO_FOLLOW_REPOSITION:
        PrintPass::cpp_cout("MAV_CMD_DO_FOLLOW_REPOSITION");
        break;
    case MAV_CMD_DO_ORBIT:
        PrintPass::cpp_cout("MAV_CMD_DO_ORBIT");
        break;
    case MAV_CMD_NAV_ROI:
        PrintPass::cpp_cout("MAV_CMD_NAV_ROI");
        break;
    case MAV_CMD_NAV_PATHPLANNING:
        PrintPass::cpp_cout("MAV_CMD_NAV_PATHPLANNING");
        break;
    case MAV_CMD_NAV_SPLINE_WAYPOINT:
        PrintPass::cpp_cout("MAV_CMD_NAV_SPLINE_WAYPOINT");
        break;
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        PrintPass::cpp_cout("MAV_CMD_NAV_VTOL_TAKEOFF");
        break;
    case MAV_CMD_NAV_VTOL_LAND:
        PrintPass::cpp_cout("MAV_CMD_NAV_VTOL_LAND");
        break;
    case MAV_CMD_NAV_GUIDED_ENABLE:
        PrintPass::cpp_cout("MAV_CMD_NAV_GUIDED_ENABLE");
        break;
    case MAV_CMD_NAV_DELAY:
        PrintPass::cpp_cout("MAV_CMD_NAV_DELAY");
        break;
    case MAV_CMD_NAV_PAYLOAD_PLACE:
        PrintPass::cpp_cout("MAV_CMD_NAV_PAYLOAD_PLACE");
        break;
    case MAV_CMD_NAV_LAST:
        PrintPass::cpp_cout("MAV_CMD_NAV_LAST");
        break;
    case MAV_CMD_CONDITION_DELAY:
        PrintPass::cpp_cout("MAV_CMD_CONDITION_DELAY");
        break;
    case MAV_CMD_CONDITION_CHANGE_ALT:
        PrintPass::cpp_cout("MAV_CMD_CONDITION_CHANGE_ALT");
        break;
    case MAV_CMD_CONDITION_DISTANCE:
        PrintPass::cpp_cout("MAV_CMD_CONDITION_DISTANCE");
        break;
    case MAV_CMD_CONDITION_YAW:
        PrintPass::cpp_cout("MAV_CMD_CONDITION_YAW");
        break;
    case MAV_CMD_CONDITION_LAST:
        PrintPass::cpp_cout("MAV_CMD_CONDITION_LAST");
        break;
    case MAV_CMD_DO_SET_MODE:
        PrintPass::cpp_cout("MAV_CMD_DO_SET_MODE");
        break;
    case MAV_CMD_DO_JUMP:
        PrintPass::cpp_cout("MAV_CMD_DO_JUMP");
        break;
    case MAV_CMD_DO_CHANGE_SPEED:
        PrintPass::cpp_cout("MAV_CMD_DO_CHANGE_SPEED");
        break;
    case MAV_CMD_DO_SET_HOME:
        PrintPass::cpp_cout("MAV_CMD_DO_SET_HOME");
        break;
    case MAV_CMD_DO_SET_PARAMETER:
        PrintPass::cpp_cout("MAV_CMD_DO_SET_PARAMETER");
        break;
    case MAV_CMD_DO_SET_RELAY:
        PrintPass::cpp_cout("MAV_CMD_DO_SET_RELAY");
        break;
    case MAV_CMD_DO_REPEAT_RELAY:
        PrintPass::cpp_cout("MAV_CMD_DO_REPEAT_RELAY");
        break;
    case MAV_CMD_DO_SET_SERVO:
        PrintPass::cpp_cout("MAV_CMD_DO_SET_SERVO");
        break;
    case MAV_CMD_DO_REPEAT_SERVO:
        PrintPass::cpp_cout("MAV_CMD_DO_REPEAT_SERVO");
        break;
    case MAV_CMD_DO_FLIGHTTERMINATION:
        PrintPass::cpp_cout("MAV_CMD_DO_FLIGHTTERMINATION");
        break;
    case MAV_CMD_DO_CHANGE_ALTITUDE:
        PrintPass::cpp_cout("MAV_CMD_DO_CHANGE_ALTITUDE");
        break;
    case MAV_CMD_DO_SET_ACTUATOR:
        PrintPass::cpp_cout("MAV_CMD_DO_SET_ACTUATOR");
        break;
    case MAV_CMD_DO_LAND_START:
        PrintPass::cpp_cout("MAV_CMD_DO_LAND_START");
        break;
    case MAV_CMD_DO_RALLY_LAND:
        PrintPass::cpp_cout("MAV_CMD_DO_RALLY_LAND");
        break;
    case MAV_CMD_DO_GO_AROUND:
        PrintPass::cpp_cout("MAV_CMD_DO_GO_AROUND");
        break;
    case MAV_CMD_DO_REPOSITION:
        PrintPass::cpp_cout("MAV_CMD_DO_REPOSITION");
        break;
    case MAV_CMD_DO_PAUSE_CONTINUE:
        PrintPass::cpp_cout("MAV_CMD_DO_PAUSE_CONTINUE");
        break;
    case MAV_CMD_DO_SET_REVERSE:
        PrintPass::cpp_cout("MAV_CMD_DO_SET_REVERSE");
        break;
    case MAV_CMD_DO_SET_ROI_LOCATION:
        PrintPass::cpp_cout("MAV_CMD_DO_SET_ROI_LOCATION");
        break;
    case MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET:
        PrintPass::cpp_cout("MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET");
        break;
    case MAV_CMD_DO_SET_ROI_NONE:
        PrintPass::cpp_cout("MAV_CMD_DO_SET_ROI_NONE");
        break;
    case MAV_CMD_DO_SET_ROI_SYSID:
        PrintPass::cpp_cout("MAV_CMD_DO_SET_ROI_SYSID");
        break;
    case MAV_CMD_DO_CONTROL_VIDEO:
        PrintPass::cpp_cout("MAV_CMD_DO_CONTROL_VIDEO");
        break;
    case MAV_CMD_DO_SET_ROI:
        PrintPass::cpp_cout("MAV_CMD_DO_SET_ROI");
        break;
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
        PrintPass::cpp_cout("MAV_CMD_DO_DIGICAM_CONFIGURE");
        break;
    case MAV_CMD_DO_DIGICAM_CONTROL:
        PrintPass::cpp_cout("MAV_CMD_DO_DIGICAM_CONTROL");
        break;
    case MAV_CMD_DO_MOUNT_CONFIGURE:
        PrintPass::cpp_cout("MAV_CMD_DO_MOUNT_CONFIGURE");
        break;
    case MAV_CMD_DO_MOUNT_CONTROL:
        PrintPass::cpp_cout("MAV_CMD_DO_MOUNT_CONTROL");
        break;
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        PrintPass::cpp_cout("MAV_CMD_DO_SET_CAM_TRIGG_DIST");
        break;
    case MAV_CMD_DO_FENCE_ENABLE:
        PrintPass::cpp_cout("MAV_CMD_DO_FENCE_ENABLE");
        break;
    case MAV_CMD_DO_PARACHUTE:
        PrintPass::cpp_cout("MAV_CMD_DO_PARACHUTE");
        break;
    case MAV_CMD_DO_MOTOR_TEST:
        PrintPass::cpp_cout("MAV_CMD_DO_MOTOR_TEST");
        break;
    case MAV_CMD_DO_INVERTED_FLIGHT:
        PrintPass::cpp_cout("MAV_CMD_DO_INVERTED_FLIGHT");
        break;
    case MAV_CMD_DO_GRIPPER:
        PrintPass::cpp_cout("MAV_CMD_DO_GRIPPER");
        break;
    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        PrintPass::cpp_cout("MAV_CMD_DO_AUTOTUNE_ENABLE");
        break;
    case MAV_CMD_NAV_SET_YAW_SPEED:
        PrintPass::cpp_cout("MAV_CMD_NAV_SET_YAW_SPEED");
        break;
    case MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
        PrintPass::cpp_cout("MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL");
        break;
    case MAV_CMD_DO_MOUNT_CONTROL_QUAT:
        PrintPass::cpp_cout("MAV_CMD_DO_MOUNT_CONTROL_QUAT");
        break;
    case MAV_CMD_DO_GUIDED_MASTER:
        PrintPass::cpp_cout("MAV_CMD_DO_GUIDED_MASTER");
        break;
    case MAV_CMD_DO_GUIDED_LIMITS:
        PrintPass::cpp_cout("MAV_CMD_DO_GUIDED_LIMITS");
        break;
    case MAV_CMD_DO_ENGINE_CONTROL:
        PrintPass::cpp_cout("MAV_CMD_DO_ENGINE_CONTROL");
        break;
    case MAV_CMD_DO_SET_MISSION_CURRENT:
        PrintPass::cpp_cout("MAV_CMD_DO_SET_MISSION_CURRENT");
        break;
    case MAV_CMD_DO_LAST:
        PrintPass::cpp_cout("MAV_CMD_DO_LAST");
        break;
    case MAV_CMD_PREFLIGHT_CALIBRATION:
        PrintPass::cpp_cout("MAV_CMD_PREFLIGHT_CALIBRATION");
        break;
    case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
        PrintPass::cpp_cout("MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS");
        break;
    case MAV_CMD_PREFLIGHT_UAVCAN:
        PrintPass::cpp_cout("MAV_CMD_PREFLIGHT_UAVCAN");
        break;
    case MAV_CMD_PREFLIGHT_STORAGE:
        PrintPass::cpp_cout("MAV_CMD_PREFLIGHT_STORAGE");
        break;
    case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
        PrintPass::cpp_cout("MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN");
        break;
    case MAV_CMD_OVERRIDE_GOTO:
        PrintPass::cpp_cout("MAV_CMD_OVERRIDE_GOTO");
        break;
    case MAV_CMD_OBLIQUE_SURVEY:
        PrintPass::cpp_cout("MAV_CMD_OBLIQUE_SURVEY");
        break;
    case MAV_CMD_MISSION_START:
        PrintPass::cpp_cout("MAV_CMD_MISSION_START");
        break;
    case MAV_CMD_ACTUATOR_TEST:
        PrintPass::cpp_cout("MAV_CMD_ACTUATOR_TEST");
        break;
    case MAV_CMD_CONFIGURE_ACTUATOR:
        PrintPass::cpp_cout("MAV_CMD_CONFIGURE_ACTUATOR");
        break;
    case MAV_CMD_COMPONENT_ARM_DISARM:
        PrintPass::cpp_cout("MAV_CMD_COMPONENT_ARM_DISARM");
        break;
    case MAV_CMD_RUN_PREARM_CHECKS:
        PrintPass::cpp_cout("MAV_CMD_RUN_PREARM_CHECKS");
        break;
    case MAV_CMD_ILLUMINATOR_ON_OFF:
        PrintPass::cpp_cout("MAV_CMD_ILLUMINATOR_ON_OFF");
        break;
    case MAV_CMD_GET_HOME_POSITION:
        PrintPass::cpp_cout("MAV_CMD_GET_HOME_POSITION");
        break;
    case MAV_CMD_INJECT_FAILURE:
        PrintPass::cpp_cout("MAV_CMD_INJECT_FAILURE");
        break;
    case MAV_CMD_START_RX_PAIR:
        PrintPass::cpp_cout("MAV_CMD_START_RX_PAIR");
        break;
    case MAV_CMD_GET_MESSAGE_INTERVAL:
        PrintPass::cpp_cout("MAV_CMD_GET_MESSAGE_INTERVAL");
        break;
    case MAV_CMD_SET_MESSAGE_INTERVAL:
        PrintPass::cpp_cout("MAV_CMD_SET_MESSAGE_INTERVAL");
        break;
    case MAV_CMD_REQUEST_MESSAGE:
        PrintPass::cpp_cout("MAV_CMD_REQUEST_MESSAGE");
        break;
    case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
        PrintPass::cpp_cout("MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES");
        break;
    case MAV_CMD_SET_CAMERA_MODE:
        PrintPass::cpp_cout("MAV_CMD_SET_CAMERA_MODE");
        break;
    default:
        PrintPass::cpp_cout("Unknown command");
        break;
    }

    PrintPass::cpp_cout_oneline("\tResult: ");

    switch (command_ack.result)
    {
    case 0:
        PrintPass::cpp_cout("MAV_RESULT_ACCEPTED");
        break;
    case 1:
        PrintPass::cpp_cout("MAV_RESULT_TEMPORARILY_REJECTED");
        break;
    case 2:
        PrintPass::cpp_cout("MAV_RESULT_DENIED");
        break;
    case 3:
        PrintPass::cpp_cout("MAV_RESULT_UNSUPPORTED");
        break;
    case 4:
        PrintPass::cpp_cout("MAV_RESULT_FAILED");
        break;
    case 5:
        PrintPass::cpp_cout("MAV_RESULT_IN_PROGRESS");
        break;
    case 6:
        PrintPass::cpp_cout("MAV_RESULT_CANCELLED");
        break;
    case 7:
        PrintPass::cpp_cout("MAV_RESULT_COMMAND_LONG_ONLY");
        break;
    case 8:
        PrintPass::cpp_cout("MAV_RESULT_COMMAND_INT_ONLY");
        break;
    case 9:
        PrintPass::cpp_cout("MAV_RESULT_COMMAND_COMMAND_UNSUPPORTED_MAV_FRAME");
        break;
    default:
        PrintPass::cpp_cout("Unkown result");
    }

    PrintPass::cpp_cout("\tprogress: " + std::to_string(command_ack.progress));
    PrintPass::cpp_cout("\tresult param 2: " + std::to_string(command_ack.result_param2));
    PrintPass::cpp_cout("\ttarget sys: " + std::to_string(command_ack.target_system));
    PrintPass::cpp_cout("\ttarget comp: " + std::to_string(command_ack.target_component));
}