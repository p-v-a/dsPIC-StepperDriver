/******************************************************************************* 
 * File:   status_led_state_machine.c
 * Author: Vladimir Pasashnikov, vladimir.pasashnikov@gmail.com
 *
 * Created on 31 August 2017
 ******************************************************************************/
#include "status_led_state_machine.h"
#include "hardware.h"
#include <stdbool.h>

const LedRule led_rules_table[LED_STATES_COUNT] = {
    /* STATE_IDLE           */ {LED_STATE_IDLE, false, false}, 
    /* STATE_STEPPING_01    */ {LED_STATE_STEPPING_02, false, true},
    /* STATE_STEPPING_02    */ {LED_STATE_STEPPING_01, false, false},
    /* STATE_HOLD_01        */ {LED_STATE_HOLD_02, false, true},
    /* STATE_HOLD_02        */ {LED_STATE_HOLD_03, false, true},
    /* STATE_HOLD_03        */ {LED_STATE_HOLD_04, false, false},
    /* STATE_HOLD_04        */ {LED_STATE_HOLD_01, false, false},
    /* STATE_FAULT_01       */ {LED_STATE_FAULT_02, false, true},
    /* STATE_FAULT_02       */ {LED_STATE_FAULT_01, false, false},
    /* STATE_FAULT_W1_01    */ {LED_STATE_FAULT_W1_02, true, true},
    /* STATE_FAULT_W1_02    */ {LED_STATE_FAULT_W1_03, false, false},
    /* STATE_FAULT_W1_03    */ {LED_STATE_FAULT_W1_04, true, false},
    /* STATE_FAULT_W1_04    */ {LED_STATE_FAULT_W1_05, false, false},
    /* STATE_FAULT_W1_05    */ {LED_STATE_FAULT_W1_06, true, false},
    /* STATE_FAULT_W1_06    */ {LED_STATE_FAULT_W1_01, false, false},
    /* STATE_FAULT_W2_01    */ {LED_STATE_FAULT_W2_02, true, true},
    /* STATE_FAULT_W2_02    */ {LED_STATE_FAULT_W2_03, false, false},
    /* STATE_FAULT_W2_03    */ {LED_STATE_FAULT_W2_04, true, true},
    /* STATE_FAULT_W2_04    */ {LED_STATE_FAULT_W2_05, false, false},
    /* STATE_FAULT_W2_05    */ {LED_STATE_FAULT_W2_06, true, false},
    /* STATE_FAULT_W2_06    */ {LED_STATE_FAULT_W2_01, false, false},
    /* STATE_MEASURE_01     */ {LED_STATE_MEASURE_02, false, true},
    /* STATE_MEASURE_02     */ {LED_STATE_MEASURE_03, true, true},
    /* STATE_MEASURE_03     */ {LED_STATE_MEASURE_04, false, true},
    /* STATE_MEASURE_04     */ {LED_STATE_MEASURE_05, true, false},
    /* STATE_MEASURE_05     */ {LED_STATE_MEASURE_06, false, true},
    /* STATE_MEASURE_06     */ {LED_STATE_MEASURE_07, true, false},
    /* STATE_MEASURE_07     */ {LED_STATE_MEASURE_08, true, true},
    /* STATE_MEASURE_08     */ {LED_STATE_MEASURE_01, true, false}
};

volatile LedState led_state;

/******************************************************************************* 
 * Function:        void StatusLedTimerEvent (void)
 * Output:          None
 * Description:     status LEDs state machine timer event, get called from Timer1 interrupt routine
 ******************************************************************************/
void StatusLedTimerEvent (void) {
    static int timer_counter = LED_UPDATE_EVENT_TIMER_COUNTER;
    
    if (! --timer_counter) {
        timer_counter = LED_UPDATE_EVENT_TIMER_COUNTER;
        LED_GREEN_PIN = led_rules_table[led_state].green_led;
        LED_RED_PIN = led_rules_table[led_state].red_led;
            
        // Transit to a new state
        led_state = led_rules_table[led_state].next_state;
    }
}

void SetStatusLedStatus (LedState state) {
    
}

