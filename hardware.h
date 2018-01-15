/******************************************************************************* 
 * File:   hardware.h
 * Author: Vladimir Pasashnikov, vladimir.pasashnikov@gmail.com
 *
 * Created on 31 August 2017
 ******************************************************************************/
#ifndef HARDWARE_H
#define	HARDWARE_H

#ifdef	__cplusplus
extern "C" {
#endif

#define FCY 40000000UL
#include <p33FJ12MC202.h>
#include <libpic30.h>
#include <libq.h>


/* Fixed parameters based on hardware*/    
#define LED_GREEN_PIN           LATBbits.LATB2          // Green LED anode connected to that pin
#define LED_RED_PIN             LATBbits.LATB5          // Red LED anode connected to that pin
#define L298_ENABLE_W1          LATBbits.LATB14         // L298 EN_A
#define L298_ENABLE_W2          LATBbits.LATB15         // L298 EN_A
#define L298_ENABLE_IN1         LATBbits.LATB10         // L298 IN_1
#define L298_ENABLE_IN2         LATBbits.LATB11         // L298 IN_2
#define L298_ENABLE_IN3         LATBbits.LATB12         // L298 IN_3
#define L298_ENABLE_IN4         LATBbits.LATB13         // L298 IN_4

#define DIR_PIN                 PORTBbits.RB6           // Direction control pin
#define STEP_PIN                PORTBbits.RB7           // Step control pin

//value in KHz
#define PWM_FCY_SET             (32.768)
//do not change, must match MCU frequency
#define MIPS                    40
//system clock in ns
#define SYSTEM_CLOCK            (0.000001/MIPS)                             
//PWM frequency value to be written in PxTPER
#define PWM_FCY                 (int)((0.5*MIPS*1000/PWM_FCY_SET)-1)

    //value corresponding to 100% duty cycle
#define PWM_MAX                 (int)((PWM_FCY+1)*2+1)
//minimum duty cycle used in closed loop PI control in percent
#define PWM_MINIMUM_DUTY        7
//PWM_MINIMUM_DUTY converted to PWM duty cycle values
#define PWM_MIN                 (int)(PWM_MINIMUM_DUTY * PWM_MAX / 100)


typedef enum {
    Forward,
    Reverse
} CurrentDirection;

typedef struct {
    _Q16    measured_current;               // Measured current for winding, in Q16, with correct direction
    _Q16    target_current;                 // Target current for winding, in Q16
    _Q16    step_target_current;            // Helper, current for normal operation
    _Q16    hold_target_current;            // Helper, current for hold state 
    int     desired_pwm;                    // PWM value to be written into PWM registers, must be within min, and max values.
    CurrentDirection current_direction;     // This is desired current direction as commanded by PID output
} WindingState;

typedef struct {
    _Q16 measured_voltage_dc_bus;
} PowerSupplyState;

void SwitchClockTo80MHzPll(void);
void InitADC(void);
void InitPWM(void);
void InitTimer1(void);
void InitPeripherals(void);
void DisableOutput ();
void EnableOutput ();
void UpdatePwm(WindingState *w, PowerSupplyState *ps, _Q16 new_target);
void SetPwm();

#ifdef	__cplusplus
}
#endif

#endif	/* HARDWARE_H */

