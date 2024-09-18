/********************************************************************************
 * @file    target_tracking.cpp
 * @author  Cameron Rose
 * @date    6/7/2024
 * @brief   Main source file where initializations, loops, and shutdown
 * 			sequences are executed.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "mavlink_msg_handler.h"
#include "mavlink_cmd_handler.h"
#include "debugger.h"
#include <mutex>
#include <signal.h>

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
bool stop_program;
std::mutex mutex_main;
bool first_loop_after_start;

extern bool save_button_press;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: sig_handler
 * Description: Stop the program if Crl+C is entered in the terminal.
 ********************************************************************************/
void sig_handler(int signo)
{
    if (signo == SIGINT)
    {
        stop_program = true;
        PrintPass::c_fprintf("received SIGINT\n");
    }
}

/********************************************************************************
 * Function: attach_sig_handler
 * Description: Attach sig handler to enable program termination by Crl+C.
 ********************************************************************************/
void attach_sig_handler(void)
{
    if (signal(SIGINT, sig_handler) == SIG_ERR)
    {
        PrintPass::c_fprintf("can't catch SIGINT");
    }
}

/********************************************************************************
 * Function: app_first_init
 * Description: Updates variable for rest of program to know that the first loop
 * 				is over.
 ********************************************************************************/
void app_first_init(void)
{
    if (first_loop_after_start == true)
    {
        first_loop_after_start = false;
    }
}

/********************************************************************************
 * Function: main
 * Description: Entry point for the program.  Runs the main loop.
 ********************************************************************************/
int main(void)
{
    attach_sig_handler();
    stop_program = false;
    first_loop_after_start = true;
    MavMsg::mav_comm_init();

    while (!stop_program)


    {
        std::lock_guard<std::mutex> lock(mutex_main);

        MavMsg::mav_comm_loop();

        app_first_init();

    }

    MavMsg::mav_comm_shutdown();

    return 0;
}
