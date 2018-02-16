/******************************************************************************* 
 * File:   main.c
 * Author: Vladimir Pasashnikov, vladimir.pasashnikov@gmail.com
 *
 * Created on 31 August 2017
 ******************************************************************************/
#include "main.h"
#include "hardware.h"
#include "status_led_state_machine.h"
#include "pid_controller.h"
#include "sine_table.h"

ControllerState controller_state;
PowerSupplyState power_supply_state;
StepperState stepper_state;

PidControllerParameters pid_parameters;

WindingState winding1;
PidControllerState winding1_pid_state;

WindingState winding2;
PidControllerState winding2_pid_state;

/******************************************************************************* 
 * Function:        void main ()
 * Output:          None
 * Description:     Starting point
 *                  
 * Note:            None
 ******************************************************************************/
int main(void) {
    // Perform a clock switch to 40MIPS(80Mhz)
    SwitchClockTo80MHzPll();

    // Perform initial hardware initialization
    InitPeripherals();
    InitADC();
    InitPWM();
    InitTimer1();
    
    MainControlLoop ();
    
    return 0;
}

/******************************************************************************* 
 * Function:        void InitPid ()
 * Output:          None
 * Description:     initialize PID controller global parameters
 *                  
 * Note:            None
 ******************************************************************************/
void InitPid () {
    pid_parameters.kp = _Q16ftoi (10.0);
    pid_parameters.ki = _Q16ftoi (3);
    pid_parameters.kd = _Q16ftoi (10);
    pid_parameters.kt = _Q16ftoi (3.0);
    
    pid_parameters.wp = _Q16ftoi (1.0);
    pid_parameters.wd = _Q16ftoi (1.0);
    
    pid_parameters.n = 8;
    
    pid_parameters.ts = _Q16ftoi (0.001 / PWM_FCY_SET);
    
    pid_parameters.output_max_limit = 16600;
    pid_parameters.output_min_limit = _Q16neg(pid_parameters.output_max_limit);
}

/******************************************************************************* 
 * Function:        void MainControlLoop ()
 * Output:          None
 * Description:     main control loop, responsible for handling drier state
 *                  
 * Note:            None
 ******************************************************************************/
void MainControlLoop () {
    while (1) {
        /*
         * Controller state machine
         */
        switch (controller_state) {
            case CTRL_STATE_IDLE:
                InitPid ();
                
                // Move stepping state
                controller_state = CTRL_STATE_UPDATE_PID;
                led_state = LED_STATE_STEPPING_01;
                break;

            case CTRL_STATE_UPDATE_PID:
                power_supply_state.measured_voltage_dc_bus = 16600;//_Q16ftoi(24.0); //XXX - temp, need a fix
                // Update PID controller
                UpdatePid(&pid_parameters);
                
                UpdateStepperState ();

                UpdatePwm (&winding1, &power_supply_state, _Q16ftoi(0));
                UpdatePwm (&winding2, &power_supply_state, _Q16ftoi(0));
                
                SetPwm();
                
                // Enable output
                EnableOutput ();

                // Move to measurement state
                controller_state = CTRL_STATE_STEPPING;

                /* Set PWM to minimum duty cycle, precaution, PWM should be well defined state, if not, set to a minimum PWM cycle*/
                winding2.desired_pwm = PWM_MIN;
                winding1.desired_pwm = PWM_MIN;

                // Enable PWM, this will trigger ADC interrupt routine and measurement cycle begins
                P1TCONbits.PTEN = 1;
                break;
            case CTRL_STATE_STEPPING:
            case CTRL_STATE_HOLD:
            case CTRL_STATE_FAULT_W1:
            case CTRL_STATE_FAULT_W2:
            case CTRL_STATE_FAULT:
                break;
        }
    }

}

/******************************************************************************* 
 * Function:        CalcStep ()
 * Output:          None
 * Description:     Calculates reference values for step defined by stepCount
 *                  from look-up table.
 * Note:            Can be called when board is initialized, as routine will
 *                  enable ADC interrupt
 ******************************************************************************/
void UpdateStepperState () {
    _Q16 target_step_current_windng1_tmp, target_step_current_windng2_tmp;
    _Q16 motor_rated_current = _Q16ftoi(0.5);
    
    if (stepper_state.step_count < TABLE_SIZE_MUL2) { // 0 to 180 degrees; stepCount 0 to 511
        if (stepper_state.step_count < TABLE_SIZE) {
            // Adjust look-up table to fit 0-90 degrees (no changes)
            target_step_current_windng1_tmp = _Q16mpy(sine_table[stepper_state.step_count], motor_rated_current);
            target_step_current_windng2_tmp = _Q16mpy(sine_table[TABLE_SIZE - stepper_state.step_count - 1], motor_rated_current);
        }
        else {
            // Adjust look-up table to fit 90-180 degrees
            target_step_current_windng1_tmp = _Q16neg(_Q16mpy(sine_table[TABLE_SIZE_MUL2 - stepper_state.step_count - 1], motor_rated_current));
            target_step_current_windng2_tmp = _Q16mpy(sine_table[stepper_state.step_count - TABLE_SIZE], motor_rated_current);
        }
    }
    else { // 180 to 360 degrees; stepCount 512 to 1023
        if (stepper_state.step_count < TABLE_SIZE_MUL3) {
            // Adjust look-up table to fit 180-270 degrees
            target_step_current_windng1_tmp = _Q16neg(_Q16mpy(sine_table[stepper_state.step_count - TABLE_SIZE_MUL2], motor_rated_current));
            target_step_current_windng2_tmp = _Q16neg(_Q16mpy(sine_table[TABLE_SIZE_MUL3 - stepper_state.step_count - 1], motor_rated_current));
        }
        else {
            // Adjust table to 360-270 degrees
            target_step_current_windng1_tmp = _Q16mpy(sine_table[TABLE_SIZE_MUL4 - stepper_state.step_count - 1], motor_rated_current);
            target_step_current_windng2_tmp = -_Q16neg(_Q16mpy(sine_table[stepper_state.step_count - TABLE_SIZE_MUL3], motor_rated_current));
        }
    }
    
    // Disable ADC interrupt
    IEC0bits.AD1IE = 0;
    
    // Loads new target current and current direction
    winding1.target_current = target_step_current_windng1_tmp;
    winding2.target_current = target_step_current_windng2_tmp;;
    
    // Enable ADC interrupt
    IEC0bits.AD1IE = 1;
}

typedef struct {
//    _Q16 target_current;
    _Q16 measured_current;
    CurrentDirection current_direction;
    _Q16 adc_value;
} PIDSample;
#define SAMPLE_TABLE_SIZE 32

PIDSample samples[SAMPLE_TABLE_SIZE];

/******************************************************************************* 
 * Function:        ClosedLoopADCInterruptRoutine ()
 * Output:          None
 * Description:     Handles ADC complete event, when controller is running in a normal state, i.e. regulates current through both windings
 * Note:            Alternates winding 1 and 2, as there is not enough time to calculate both PID within given time frame.
 ******************************************************************************/
inline 
void ClosedLoopADCInterruptRoutine () {
    static int sample_count = 0;
    static int alternatePID = 0;
    alternatePID = !alternatePID;
    // Read real-time DC voltage
    power_supply_state.measured_voltage_dc_bus = (unsigned int) ADC1BUF3;

    // update PID parameters for winding 1
    if (alternatePID) {
        if(sample_count > SAMPLE_TABLE_SIZE) {
            sample_count = 0;
        } 

        // 32768 is 0.5 in Q16. this is due to reference of the current amplifier, which is AVdd/2
        winding1.measured_current = (unsigned int)ADC1BUF1 - 32768;
        samples[sample_count].adc_value = winding1.measured_current;
        samples[sample_count].current_direction = winding1.current_direction;
        // Change current sign, if reverse current direction commanded 
        if (winding1.current_direction == Reverse)
            winding1.measured_current = _Q16neg(winding1.measured_current);
        
        CalculatePIDOutput(&pid_parameters, &winding1_pid_state , winding1.target_current, winding1.measured_current);
        
        UpdatePwm (&winding1, &power_supply_state, winding1_pid_state.u0);
                
        samples[sample_count].measured_current = winding1.measured_current;
//        samples[sample_count].target_current = winding1.target_current;
        //samples[sample_count].pwm_value = winding1.desired_pwm;
        //samples[sample_count].controller_output = winding1_pid_state.u0;
        
        ++sample_count;
        SetPwm();

        P1SECMPbits.SEVTCMP = (winding2.desired_pwm / 2 - 1);// - PWM_ACQ_DELAY; //PWM_FCY - PWM_ACQ_DELAY;
        P1SECMPbits.SEVTDIR = 0;
    }
    // update PID parameters for winding 2
    else {
        // 32768 is 0.5 in Q16. this is due to reference of the current amplifier, which is AVdd/2
        winding2.measured_current = (unsigned int)ADC1BUF2 - 32768;

        // Change current sign, if reverse current direction commanded 
        if (winding2.current_direction == Reverse)
            winding2.measured_current = _Q16neg(winding2.measured_current);
        
        CalculatePIDOutput(&pid_parameters, &winding2_pid_state, winding2.target_current, winding2.measured_current);

        UpdatePwm (&winding2, &power_supply_state, winding2_pid_state.u0);

        SetPwm();

        P1SECMPbits.SEVTCMP = (winding1.desired_pwm / 2 - 1);// - PWM_ACQ_DELAY; //PWM_FCY - PWM_ACQ_DELAY;
        P1SECMPbits.SEVTDIR = 0;
    }
}
