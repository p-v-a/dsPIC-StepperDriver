/******************************************************************************* 
 * File:   hardware_interrupts.c
 * Author: Vladimir Pasashnikov, vladimir.pasashnikov@gmail.com
 *
 * Created on 31 August 2017
 ******************************************************************************/
#include "hardware.h"
#include "main.h"
#include "sine_table.h"
#include "status_led_state_machine.h"
#include <p33FJ12MC202.h>
#include <libpic30.h>

/******************************************************************************* 
 * Function:        _INT0Interrupt ()
 * Output:          None
 * Description:     Processing step pulse on PIN RB6
 * Note:            None
 ******************************************************************************/
void __attribute__((__interrupt__, auto_psv)) _INT0Interrupt () {
    // Update stepCount
    if (DIR_PIN) {
        stepper_state.step_count = TABLE_SIZE_MUL4 + stepper_state.step_count - stepper_state.step_increment;
    }
    else
    {
        stepper_state.step_count = TABLE_SIZE_MUL4 + stepper_state.step_count + stepper_state.step_increment;
    }

    stepper_state.step_count %= TABLE_SIZE_MUL4;
    
    UpdateStepperState ();
    
    IFS0bits.INT0IF = 0;
}


/******************************************************************************* 
 * Function:        _ADC1Interrupt ()
 * Output:          None
 * Description:     ADC completion interrupt.
 * Note:            Calls actual subroutine depending on state machine
 ******************************************************************************/
void __attribute__((__interrupt__, auto_psv)) _ADC1Interrupt () {
    // Clear ADC interrupt flag
    IFS0bits.AD1IF = 0;
    
    /*
     * Interrupt depends on controller FSM state, thus switch here
     */
    switch (controller_state) {
        case CTRL_STATE_HOLD:
        case CTRL_STATE_STEPPING:
            ClosedLoopADCInterruptRoutine ();
            break;
        default:
            break;
    }
}

/******************************************************************************* 
 * Function:        _T1Interrupt ()
 * Output:          None
 * Description:     Household timer
 * Note:            Detects halt state, provides base for step speed
 *                  calculation, LED FSM state changes, etc
 ******************************************************************************/
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt () {
    StatusLedTimerEvent();
    controller_state = CTRL_STATE_FAULT_W1;
    DisableOutput ();

    IFS0bits.T1IF = 0;
}

/******************************************************************************* 
 * Function:        _FLTA1Interrupt ()
 * Output:          None
 * Description:     Overcurrent event for winding 1 processing
 * Note:            None
 ******************************************************************************/
void __attribute__((__interrupt__, auto_psv)) _FLTA1Interrupt() {
    led_state = LED_STATE_FAULT_W1_01;
    controller_state = CTRL_STATE_FAULT_W2;
    DisableOutput ();
    
    IFS3bits.FLTA1IF = 0; 
}

/******************************************************************************* 
 * Function:        _FLTA2Interrupt ()
 * Output:          None
 * Description:     Overcurrent event for winding 2 processing
 * Note:            None
 ******************************************************************************/
void __attribute__((__interrupt__, auto_psv)) _FLTA2Interrupt() {
    led_state = LED_STATE_FAULT_W2_01;

    IFS4bits.FLTA2IF = 0;
}
