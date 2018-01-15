/******************************************************************************* 
 * File:   hardware.h
 * Author: Vladimir Pasashnikov, vladimir.pasashnikov@gmail.com
 *
 * Created on 31 August 2017
 ******************************************************************************/
#include "hardware.h"
#include "status_led_state_machine.h"
#include "pid_controller.h"

#ifndef MAIN_H
#define	MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif

/*
 * Controller states. This FSM implemented as a switch block in the main loop.
 * it seems that's more practical, as we don't need to mess around function pointers.
 */
typedef enum 
{
    CTRL_STATE_IDLE,
    CTRL_STATE_UPDATE_PID,
    CTRL_STATE_STEPPING,
    CTRL_STATE_HOLD,
    CTRL_STATE_FAULT,
    CTRL_STATE_FAULT_W1,
    CTRL_STATE_FAULT_W2,
    CTRL_STATES_COUNT
} ControllerState;

typedef struct {
    /* Step mode
     * step_size = 0 -> Full Step
     * step_size = 1 -> 1/2 Step
     * step_size = 2 -> 1/4 Step
     * step_size = 3 -> 1/8 Step
     * step_size = 4 -> 1/16 Step
     * step_size = 5 -> 1/32 Step
     * step_size = 6 -> 1/64 Step
     * step_size = 7 -> 1/128 Step
     * step_size = 8 -> 1/256 Step
     */
    int step_size;

    /* used to increment micro-stepping counter stepCount
     * step_increment = 256 -> Full Step
     * step_increment = 128 -> 1/2 Step
     * step_increment = 64  -> 1/4 Step
     * step_increment = 32  -> 1/8 Step
     * step_increment = 16  -> 1/16 Step
     * step_increment = 8   -> 1/32 Step
     * step_increment = 8   -> 1/32 Step
     * step_increment = 4   -> 1/64 Step
     * step_increment = 2   -> 1/128 Step
     * step_increment = 1   -> 1/256 Step
     */
    int step_increment; 

    volatile int step_count;             //micro-step counter
} StepperState;

extern ControllerState controller_state;
extern StepperState stepper_state;

extern PidControllerParameters pid_parameters;

extern WindingState winding1;
extern PidControllerState winding1_pid_state;

extern WindingState winding2;
extern PidControllerState winding2_pid_state;

void MainControlLoop ();
void UpdateStepperState ();
void ClosedLoopADCInterruptRoutine ();

#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */

