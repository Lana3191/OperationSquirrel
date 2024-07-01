#ifdef USE_JETSON

/********************************************************************************
 * @file    jetson_IO.cpp
 * @author  Cameron Rose
 * @date    5/22/2024
 * @brief   Configure Jetson GPIO and provide methods for using the pins on the
 *          jetson.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "jetson_IO.h"

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
const int GREEN_LED_PIN = 19;
const int RED_LED_PIN = 18;

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: StatusIndicators
 * Description: Constructor
 ********************************************************************************/
StatusIndicators::StatusIndicators(void) {}

/********************************************************************************
 * Function: Video
 * Description: Destructor
 ********************************************************************************/
StatusIndicators::~StatusIndicators(void) {}

/********************************************************************************
 * Function: gpio_init
 * Description: Initialize the pins on the jetson.
 ********************************************************************************/
bool StatusIndicators::gpio_init(void)
{
    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(GREEN_LED_PIN, GPIO::OUT, GPIO::LOW);
    GPIO::setup(RED_LED_PIN, GPIO::OUT, GPIO::LOW);

    return true;
}

/********************************************************************************
 * Function: gpio_shutdown
 * Description: Cleanup tasks for jetson io.
 ********************************************************************************/
void StatusIndicators::gpio_shutdown(void)
{
    GPIO::cleanup();
}

/********************************************************************************
 * Function: status_initializing
 * Description: Use this sequence to indicate program is still in the init phase.
 ********************************************************************************/
void StatusIndicators::status_initializing(void)
{
    for (int j = 0; j < 3; j++)
    {
        // Blink three times
        for (int i = 0; i < 3; i++)
        {
            GPIO::output(RED_LED_PIN, GPIO::HIGH);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(RED_LED_PIN, GPIO::LOW);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(GREEN_LED_PIN, GPIO::HIGH);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            GPIO::output(GREEN_LED_PIN, GPIO::LOW);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        // Pause before repeating
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    GPIO::output(GREEN_LED_PIN, GPIO::HIGH);
    GPIO::output(RED_LED_PIN, GPIO::HIGH);
}

/********************************************************************************
 * Function: status_good
 * Description: Indicate that the system is good with a green led.
 ********************************************************************************/
void StatusIndicators::status_good(void)
{
    GPIO::output(GREEN_LED_PIN, GPIO::HIGH);
    GPIO::output(RED_LED_PIN, GPIO::LOW);
}

/********************************************************************************
 * Function: status_bad
 * Description: Indicate that the system is bad with a red led.
 ********************************************************************************/
void StatusIndicators::status_bad(void)
{
    GPIO::output(RED_LED_PIN, GPIO::HIGH);
    GPIO::output(GREEN_LED_PIN, GPIO::LOW);
}

/********************************************************************************
 * Function: status_bad_blink
 * Description: Indicate that the system is bad with a red led.
 ********************************************************************************/
void StatusIndicators::status_bad_blink(void)
{
    GPIO::output(GREEN_LED_PIN, GPIO::LOW);

    for (int i = 0; i < 3; i++)
    {
        GPIO::output(RED_LED_PIN, GPIO::HIGH);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(RED_LED_PIN, GPIO::LOW);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(RED_LED_PIN, GPIO::HIGH);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(RED_LED_PIN, GPIO::LOW);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(RED_LED_PIN, GPIO::HIGH);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(RED_LED_PIN, GPIO::LOW);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        GPIO::output(RED_LED_PIN, GPIO::HIGH);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    GPIO::output(RED_LED_PIN, GPIO::HIGH);
}

/********************************************************************************
 * Function: clear_all_leds
 * Description: Indicate that the system is bad with a red led.
 ********************************************************************************/
void StatusIndicators::clear_all_leds(void)
{
    GPIO::output(GREEN_LED_PIN, GPIO::LOW);
    GPIO::output(RED_LED_PIN, GPIO::LOW);
}

#endif // USE_JETSON