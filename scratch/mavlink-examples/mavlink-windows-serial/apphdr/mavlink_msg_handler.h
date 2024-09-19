#pragma once

/********************************************************************************
 * @file    mavlink_msg_handler.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef MAVLINK_MSG_HANDLER_H
#define MAVLINK_MSG_HANDLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "mavlink_print_info.h"
#include "mavlink_cmd_handler.h"
#include "serial_comm.h"
#include <mavlink.h>
#include <common.h>

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern const uint8_t SENDER_SYS_ID;
extern const uint8_t SENDER_COMP_ID;
extern const uint8_t TARGET_SYS_ID;
extern const uint8_t TARGET_COMP_ID;
extern const int32_t MESSAGE_RATE_DEFAULT;
extern const int32_t MESSAGE_RATE_1Hz;
extern const int32_t MESSAGE_RATE_40Hz;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern uint16_t mav_veh_command_id;
extern uint8_t mav_veh_command_result;
extern uint8_t mav_veh_command_progress;
extern int32_t mav_veh_command_result_param2;
extern uint8_t mav_veh_command_target_system;
extern uint8_t mav_veh_command_target_component;

extern uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_present;
extern uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_enabled;
extern uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_health;
extern uint16_t mav_veh_sys_stat_load;
extern uint16_t mav_veh_sys_stat_voltage_battery;
extern int16_t mav_veh_sys_stat_current_battery;
extern uint16_t mav_veh_sys_stat_drop_rate_comm;
extern uint16_t mav_veh_sys_stat_errors_comm;
extern uint16_t mav_veh_sys_stat_errors_count1;
extern uint16_t mav_veh_sys_stat_errors_count2;
extern uint16_t mav_veh_sys_stat_errors_count3;
extern uint16_t mav_veh_sys_stat_errors_count4;
extern int8_t mav_veh_sys_stat_battery_remaining;
extern uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_prsnt_extnd;
extern uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_enbld_extnd;
extern uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_health_extnd;

extern uint32_t mav_veh_custom_mode;
extern uint8_t mav_veh_type;
extern uint8_t mav_veh_autopilot_type;
extern uint8_t mav_veh_base_mode;
extern uint8_t mav_veh_state;
extern uint8_t mav_veh_mavlink_version;

extern int32_t mav_veh_lat;
extern int32_t mav_veh_lon;
extern int32_t mav_veh_alt;
extern int32_t mav_veh_rel_alt;
extern int16_t mav_veh_gps_vx;
extern int16_t mav_veh_gps_vy;
extern int16_t mav_veh_gps_vz;
extern uint16_t mav_veh_gps_hdg;

extern float mav_veh_roll;
extern float mav_veh_pitch;
extern float mav_veh_yaw;
extern float mav_veh_rollspeed;
extern float mav_veh_pitchspeed;
extern float mav_veh_yawspeed;
extern int16_t mav_veh_imu_ax;
extern int16_t mav_veh_imu_ay;
extern int16_t mav_veh_imu_az;
extern int16_t mav_veh_imu_xgyro;
extern int16_t mav_veh_imu_ygyro;
extern int16_t mav_veh_imu_zgyro;
extern int16_t mav_veh_imu_xmag;
extern int16_t mav_veh_imu_ymag;
extern int16_t mav_veh_imu_zmag;

extern float mav_veh_q1_target;
extern float mav_veh_q2_target;
extern float mav_veh_q3_target;
extern float mav_veh_q4_target;
extern float mav_veh_roll_rate_target;
extern float mav_veh_pitch_rate_target;
extern float mav_veh_yaw_rate_target;
extern float mav_veh_thrust_target;

extern float mav_veh_q1_actual;
extern float mav_veh_q2_actual;
extern float mav_veh_q3_actual;
extern float mav_veh_q4_actual;
extern float mav_veh_roll_rate_actual;
extern float mav_veh_pitch_rate_actual;
extern float mav_veh_yaw_rate_actual;
extern float mav_veh_repr_offset_q[4];

extern uint16_t mav_veh_rngfdr_min_distance;
extern uint16_t mav_veh_rngfdr_max_distance;
extern uint16_t mav_veh_rngfdr_current_distance;
extern uint8_t mav_veh_rngfdr_type;
extern uint8_t mav_veh_rngfdr_id;
extern uint8_t mav_veh_rngfdr_orientation;
extern uint8_t mav_veh_rngfdr_covariance;
extern float mav_veh_rngfdr_horizontal_fov;
extern float mav_veh_rngfdr_vertical_fov;
extern float mav_veh_rngfdr_quaternion[4];
extern uint8_t mav_veh_rngfdr_signal_quality;

extern float mav_veh_flow_comp_m_x;
extern float mav_veh_flow_comp_m_y;
extern float mav_veh_ground_distance;
extern int16_t mav_veh_flow_x;
extern int16_t mav_veh_flow_y;
extern uint8_t mav_veh_sensor_id;
extern uint8_t mav_veh_quality;
extern float mav_veh_flow_rate_x;
extern float mav_veh_flow_rate_y;

extern float mav_veh_local_ned_x;
extern float mav_veh_local_ned_y;
extern float mav_veh_local_ned_z;
extern float mav_veh_local_ned_vx;
extern float mav_veh_local_ned_vy;
extern float mav_veh_local_ned_vz;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class MavMsg
{
public:
    MavMsg();
    ~MavMsg();

    static bool mav_comm_init(void);
    static void mav_comm_loop(void);
    static void mav_comm_shutdown(void);
    static bool start_mav_comm(void) { return SerialComm::start_uart_comm(); }; // Open up uart port for mavlink messages
    static void stop_mav_comm(void) { SerialComm::stop_uart_comm(); };          // Stop mavlink comms on uart port
    static uint8_t read_mav_msg(void) { return SerialComm::read_uart(); };      // Read a byte
    static void subscribe(uint16_t msg_id, float msg_interval);
    static bool start_message_subscriptions(void);
    static void parse_mav_msgs(void);

    static void proc_mav_heartbeat_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_gps_int_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_system_time_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_sys_status_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_statustext_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_param_value_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_autopilot_version_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_scaled_imu_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_local_position_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_position_target_local_ned_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_set_position_target_local_ned_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_attitude_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_attitude_target_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_set_attitude_target_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_attitude_quaternion_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_command_ack_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_optical_flow_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_distance_sensor_msg(const mavlink_message_t *msg, const char *term);
    static void proc_mav_position_local_ned_msg(const mavlink_message_t *msg, const char *term);

private:
    static void set_mav_msg_rate(uint16_t msg_id, float msg_interval) { MavCmd::set_mav_msg_rate(msg_id, msg_interval); };
    static void req_mav_msg(uint16_t msg_id) { MavCmd::req_mav_msg(msg_id); };
};

#endif // MAVLINK_MSG_HANDLER_H