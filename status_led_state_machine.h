/* 
 * File:   status_led_state_machine.h
 * Author: vladimir
 *
 * Created on 15 January 2018, 10:58 PM
 */
#include <stdbool.h>

#ifndef STATUS_LED_STATE_MACHINE_H
#define	STATUS_LED_STATE_MACHINE_H

#ifdef	__cplusplus
extern "C" {
#endif

// LED updated happens every 250ms, with timer period 100n, we need count 2500 timer events
#define LED_UPDATE_EVENT_TIMER_COUNTER 2500

/* 
 * This is FSM to control LED states. The way rules are written is that states 
 * are looped and FSM remains within particular state loop unless commanded 
 * (for example if in STEPPING state, FSM will be looping through 
 * STATE_STEPPING_01 and STATE_STEPPING_02 indefinitely).
 */
typedef enum 
{
    LED_STATE_IDLE,
    LED_STATE_STEPPING_01,  // Enable green LED
    LED_STATE_STEPPING_02,  // Disable green LED
    LED_STATE_HOLD_01,      // Enable green LED
    LED_STATE_HOLD_02,      // Wait
    LED_STATE_HOLD_03,      // Disable green LED
    LED_STATE_HOLD_04,      // Wait
    LED_STATE_FAULT_01,     // Disable green LED, enable red LED
    LED_STATE_FAULT_02,     // Disable green LED, disable red LED
    LED_STATE_FAULT_W1_01,  // Enable green LED, enable red LED
    LED_STATE_FAULT_W1_02,  // Disable green LED, disable red LED
    LED_STATE_FAULT_W1_03,  // Enable red LED
    LED_STATE_FAULT_W1_04,  // Disable red LED
    LED_STATE_FAULT_W1_05,  // Enable red LED
    LED_STATE_FAULT_W1_06,  // Disable red LED
    LED_STATE_FAULT_W2_01,  // Enable green LED, enable red LED
    LED_STATE_FAULT_W2_02,  // Disable green LED, disable red LED
    LED_STATE_FAULT_W2_03,  // Enable green LED, enable red LED
    LED_STATE_FAULT_W2_04,  // Disable green LED, disable red LED
    LED_STATE_FAULT_W2_05,  // Enable red LED
    LED_STATE_FAULT_W2_06,  // Disable red LED
    LED_STATE_MEASURE_01,   // Enable green LED
    LED_STATE_MEASURE_02,   // Enable red LED
    LED_STATE_MEASURE_03,   // Disable red LED
    LED_STATE_MEASURE_04,   // Disable green LED, enable red LED
    LED_STATE_MEASURE_05,   // Enable green LED, disable red LED
    LED_STATE_MEASURE_06,   // Disable green LED, enable red LED
    LED_STATE_MEASURE_07,   // Enable green LED
    LED_STATE_MEASURE_08,   // Disable green LED
    LED_STATES_COUNT
} LedState;

/*
 * FSM transition table type to control LED status.
 * it's not ideal as we waste memory for led state (keeping them i 8-bit variable), but so far firmware fits into controller memory, so KISS
 */
typedef struct
{
    LedState    next_state;
    bool        red_led;
    bool        green_led;
} LedRule;

extern volatile LedState led_state;
void StatusLedTimerEvent (void);

#ifdef	__cplusplus
}
#endif

#endif	/* STATUS_LED_STATE_MACHINE_H */

