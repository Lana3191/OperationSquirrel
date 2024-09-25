/********************************************************************************
 * @file    system_controller.cpp
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
#include "system_controller.h"
#include "mavlink_msg_handler.h"
#include "mavlink_cmd_handler.h"
#include "datalog.h"
#include "vehicle_controller.h"

#ifdef JETSON_B01

#include "video_IO.h"
#include "object_detection.h"
#include <jsoncpp/json/json.h> //sudo apt-get install libjsoncpp-dev THEN target_link_libraries(your_executable_name jsoncpp)
#include "follow_target.h"

#endif // JETSON_B01

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
DebugTerm SysStat("");
bool systems_initialized;
SYSTEM_STATE system_state;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: SystemController
 * Description: Class constructor
 ********************************************************************************/
SystemController::SystemController(void) {}

/********************************************************************************
 * Function: ~SystemController
 * Description: Class destructor
 ********************************************************************************/
SystemController::~SystemController(void) {}

/********************************************************************************
 * Function: system_init
 * Description: Return 0 if all system init tasks have successfully completed.
 ********************************************************************************/
int SystemController::system_init(void)
{
    systems_initialized = false;

#ifdef JETSON_B01

    StatusIndicators::gpio_init();
    StatusIndicators::status_initializing();

    if (!Video::video_init() ||
        !Detection::detection_net_init() ||
        !Follow::follow_target_init() ||
        !VehicleController::vehicle_control_init())
    {
        StatusIndicators::status_bad_blink();
        return 1;
    }

    StatusIndicators::status_initializing();

    if (!MavMsg::mav_comm_init() ||
        !DataLogger::data_log_init() ||
        !VehicleController::vehicle_control_init())
    {
        StatusIndicators::status_bad_blink();
        return 1;
    }

#elif _WIN32 && ENABLE_CV

    if (!Video::video_init() ||
        !Detection::detection_net_init())
    {
        return 1;
    }

#else

    if (!MavMsg::mav_comm_init() ||
        /* !DataLogger::data_log_init() || */
        !VehicleController::vehicle_control_init())
    {
        return 1;
    }

#endif // JETSON_B01

    systems_initialized = true;

    return 0;
}

/********************************************************************************
 * Function: system_state_machine
 * Description: Determine system state,.
 ********************************************************************************/
int SystemController::system_state_machine(void)
{
    // Initialize system status on startup
    if (first_loop_after_start)
    {
        system_state = SYSTEM_STATE::DEFAULT;
    }
    else
    {
        bool mav_type_is_quad = (mav_veh_type == MAV_TYPE_QUADROTOR && mav_veh_autopilot_type == MAV_AUTOPILOT_ARDUPILOTMEGA);
        bool prearm_checks = false;

#ifdef ENABLE_CV

        prearm_checks = ((mav_veh_sys_stat_onbrd_cntrl_snsrs_present & MAV_SYS_STATUS_PREARM_CHECK) != 0 && valid_image_rcvd);

#else

        prearm_checks = ((mav_veh_sys_stat_onbrd_cntrl_snsrs_present & MAV_SYS_STATUS_PREARM_CHECK) != 0);

#endif // ENABLE_CV

        // Switch case determines how we transition from one state to another
        switch (system_state)
        {
        // Default state is the first state, nothing is initialialized, no systems are active
        case SYSTEM_STATE::DEFAULT:
            if (systems_initialized)
            {
                system_state = SYSTEM_STATE::INIT;
            }
            break;
        // After video, inference, SLAM, and other systems have successfully initialized we are in the init state
        case SYSTEM_STATE::INIT:
            if (prearm_checks)
            {
                system_state = SYSTEM_STATE::PRE_ARM_GOOD;
            }
            break;
        // Pre arm good means that the data is from a drone and pre arm checks are good
        case SYSTEM_STATE::PRE_ARM_GOOD:
            if (mav_type_is_quad && mav_veh_state == MAV_STATE_STANDBY)
            {
                system_state = SYSTEM_STATE::STANDBY;
            }
            else if (mav_veh_rel_alt > 1000 && mav_type_is_quad && mav_veh_state == MAV_STATE_ACTIVE)
            {
                system_state = SYSTEM_STATE::IN_FLIGHT_GOOD;
            }
            break;
        // Standby means we are ready to takeoff
        case SYSTEM_STATE::STANDBY:
            if ((mav_veh_rel_alt > 1450 || mav_veh_rngfdr_current_distance > 145) && mav_veh_state == MAV_STATE_ACTIVE)
            {
                system_state = SYSTEM_STATE::IN_FLIGHT_GOOD;
            }
            break;
        // In flight good means the vehicle is in the air and has no system failures
        case SYSTEM_STATE::IN_FLIGHT_GOOD:

            break;
        // In flight good means the vehicle is in the air with some system failures (e.g. video feed stopped)
        case SYSTEM_STATE::IN_FLIGHT_ERROR:

            break;
        }
    }

    return 0;
}

#ifdef JETSON_B01

/********************************************************************************
 * Function: led_system_indicators
 * Description: Control external leds to describe the system state.
 ********************************************************************************/
void SystemController::led_system_indicators(void)
{
    if (system_state == SYSTEM_STATE::DEFAULT ||
        system_state == SYSTEM_STATE::INIT ||
        system_state == SYSTEM_STATE::PRE_ARM_GOOD ||
        system_state == SYSTEM_STATE::IN_FLIGHT_GOOD)
    {
        StatusIndicators::status_good();
    }
    else
    {
        StatusIndicators::status_good();
    }
}

#endif // JETSON_B01

/********************************************************************************
 * Function: system_control_loop
 * Description: Main loop for functions that monitor and control system states.
 ********************************************************************************/
void SystemController::system_control_loop(void)
{
    system_state_machine();

#ifdef JETSON_B01

    led_system_indicators();

#endif // JETSON_B01
}

/********************************************************************************
 * Function: system_shutdown
 * Description: All shutdown functions are called here.
 ********************************************************************************/
void SystemController::system_shutdown(void)
{
#ifdef JETSON_B01

    Video::shutdown();
    Detection::shutdown();

#elif _WIN32 && ENABLE_CV



#else

    // Nothing to shutdown

#endif // JETSON_B01

    MavMsg::mav_comm_shutdown();
}
