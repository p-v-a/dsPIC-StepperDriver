/******************************************************************************* 
 * File:   hardware.c
 * Author: Vladimir Pasashnikov, vladimir.pasashnikov@gmail.com
 *
 * Created on 31 August 2017
 ******************************************************************************/
#include "hardware.h"
#include "main.h"
#include <p33FJ12MC202.h>
#include <libpic30.h>

/*********************************************************************
   Setting Configuration Bits
 ********************************************************************/
_FBS (BSS_NO_FLASH & BWRP_WRPROTECT_OFF);
/* no Boot sector and
   write protection disabled */

_FOSCSEL (IESO_OFF & FNOSC_FRC);
/* The chip is started using FRC then switch */

_FWDT (FWDTEN_OFF);
/* Turn off Watchdog Timer */

_FGS (GSS_OFF & GCP_OFF & GWRP_OFF);
/* Set Code Protection Off for the General Segment */

_FOSC (FCKSM_CSECMD & IOL1WAY_OFF & OSCIOFNC_OFF & POSCMD_XT);
 /* clock switch & monitoring disabled
    remapable I/O enabled
   OSC2 pin is clock O/P
   oscilator is XT
 */

_FPOR (PWMPIN_ON & HPOL_ON & LPOL_ON & FPWRT_PWR128);
/* PWM mode is Port registers
   PWM high & low active high
   FPOR power on reset 128ms
*/
_FICD(JTAGEN_OFF & ICS_PGD3);
/* JTAG Enable Bit: JTAG is disabled
   ICD communication channel select bits: communicate on PGC3/EMUC3 and PGD3/EMUD3 
*/

/******************************************************************************* 
 * Function:        SwitchClockTo80MHzPll ()
 * Output:          None
 * Description:     Switch clock to 80MHz
 *                  
 * Note:            None
 ******************************************************************************/
void SwitchClockTo80MHzPll(void) {
/*
	Perform a clock switch to 40MIPS (80MHz)

	The source clock is an external crystal (8MHz in our case) or the internal oscillator.

	N1 = This factor (2 through 33) scales the source clock into the range
		 of 0.8 to 8MHz.  It's value is PLLPRE+2

	M  = PLL Feedback divisor (2 through 513).  The factor by which the source clock is 
         multiplied.  The resulting value must be between 100-200MHz.  It's value is
         PLLFBD+2

	N2 = This factor (2,4 or 8) scales the 100-200MHz clock into the 12.5-80MHz range.
 		 It's value is 2*(PLLPOST+1)

	Our target is 80MHz

	80MHz = ((8MHz/N1)*M)/N2
		  = ((8MHz/2)*40)/2

          N1 = 2 so PLLPRE = 0
		  M = 40 so PLLFBD = 38
          N2 = 2 so PLLPOST = 0
*/

	PLLFBD = 38;
	CLKDIVbits.PLLPOST = 0;
	CLKDIVbits.PLLPRE = 0;
	
	//unlock OSCCON register
	__builtin_write_OSCCONH(0x03);
	__builtin_write_OSCCONL(0x01);
	
    //wait for clock to stabilize	
	while(OSCCONbits.COSC != 0b011);
	//wait for PLL to lock
	while(OSCCONbits.LOCK !=1);
    //clock switch finished
}

/******************************************************************************* 
 * Function:        InitADC ()
 * Output:          None
 * Description:     Initialize ADC for reading winding currents and DC bus voltage
 * Note:            None
 ******************************************************************************/
void InitADC(void) {
    // Configure AN0(RA0), AN1(RA1) and AN2(RB0) in Analog mode
    AD1PCFGLbits.PCFG0 = 0;
    AD1PCFGLbits.PCFG1 = 0;
    AD1PCFGLbits.PCFG2 = 0;      
    
    // Set RA0, RA1 and RB0 as inputs
    TRISAbits.TRISA0 = 1; // Winding 1 current sensing
    TRISAbits.TRISA1 = 1; // Winding 1 current sensing
    TRISBbits.TRISB0 = 1; // DC_BUS sensing
    
    // Turn ADC off for now
    AD1CON1bits.ADON = 0;
    // Disable operation in MCU idle mode
    AD1CON1bits.ADSIDL = 0;
    // 10-bit operation mode
    AD1CON1bits.AD12B = 0;
    // Data Output Format unsigned fractional dddd dddd dd00 0000
    AD1CON1bits.FORM = 0x2;
    // Sample 4 channels simultaneously
    AD1CON1bits.SIMSAM = 1;
    // Start immediately after SAMP bit is auto-set
    AD1CON1bits.ASAM = 1;
    // Motor Control PWM1 interval ends sampling and starts conversion
    AD1CON1bits.SSRC = 0x03;
    
    // MUX a CH1, CH2, CH3 negative input to VREF-
    AD1CHS123bits.CH123NA = 0x00;
    AD1CHS123bits.CH123NB = 0x00;
    // CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2
    AD1CHS123bits.CH123SA = 0x00;
    AD1CHS123bits.CH123SB = 0x00;

    // MUX A Channel 0 negative input is
    AD1CHS0bits.CH0NA = 0x00;
    // MUX A Channel 0 positive input is AN5
    AD1CHS0bits.CH0SA = 0x05;
    // MUX B Channel 0 negative input is
    AD1CHS0bits.CH0NB = 0x00;
    // MUX B Channel 0 positive input is AN5
    AD1CHS0bits.CH0SB = 0x05;
    
    //  Skip all ANx for input scan
    AD1CSSL = 0x00;
    
    // ADC Conversion clock derived from system clock
    AD1CON3bits.ADRC = 0x00;
    // Autosample time bits = 0 TAD, as PWM controls sampling
    AD1CON3bits.SAMC = 0x00;
    // TAD = 2 TCY, thus TAD = 50 nS
    AD1CON3bits.ADCS = 0x01;
    
    // ADREF+ = Avdd, ADREF- = Avss
    AD1CON2bits.VCFG = 0x00;
    // Do not scan inputs
    AD1CON2bits.CSCNA = 0;
    // Converts CH0, CH1, CH2 and CH3
    AD1CON2bits.CHPS = 0x02;
    // Interrupts at the completion of conversion for each sample/convert sequence
    AD1CON2bits.SMPI = 0x0;
    // Always starts filling buffer from the beginning
    AD1CON2bits.BUFM = 0;
    
    // Set priority
    IPC3bits.AD1IP = 5;
    // Clear ADC interrupt flag
    IFS0bits.AD1IF = 0;
    // Enable ADC interrupt
    IEC0bits.AD1IE = 1;

    // Turn ADC on, it wont happen until PWM is enabled
    AD1CON1bits.ADON = 1;
}

/******************************************************************************* 
 * Function:        InitPWM ()
 * Output:          None
 * Description:     Initialize PWM1 pairs 2 and 3 to generate H-bridge control signals
 * Note:            None
 ******************************************************************************/
void InitPWM(void) {
    P1DC1 = 0;
    P1DC2 = 0;
    P1DC3 = 0;

    // Set PWM period
    P1TPER = PWM_FCY;
    
    // Set PWM1 pair 2 and 3 into independent mode and enable pin for PWM output
    PWM1CON1bits.PMOD2 = 1;
    PWM1CON1bits.PEN2H = 1;
    PWM1CON1bits.PEN2L = 1;
    PWM1CON1bits.PMOD3 = 1;
    PWM1CON1bits.PEN3H = 1;
    PWM1CON1bits.PEN3L = 1;

    // Special event pre-scale 1:1
    PWM1CON2bits.SEVOPS = 0;
    // Updates to PxDC are sync to the PWM time base
    PWM1CON2bits.IUE = 0;
    // Output overrides via the PxOVDCON register are synchronized to the PWM time base
    PWM1CON2bits.OSYNC = 1;

    P1SECMPbits.SEVTCMP = 1;
    
    // Override all PWM pairs, all off.
    P1OVDCON = 0;
    
    // PWM time base operates in a Continuous Up/Down Count mode
    P1TCONbits.PTMOD = 2; 
    
    // Set pwm1 pair 2 and 3 pins as outputs
    TRISBbits.TRISB10 = 0;
    TRISBbits.TRISB11 = 0;
    TRISBbits.TRISB12 = 0;
    TRISBbits.TRISB13 = 0;

    // Set ADC trigger to PWM Active pulse middle
    SEVTCMP = 0;
    
    // Configure faults to pins PR8 and PR9.
    //RPINR12bits.FLTA1R = 0x08;
    //RPINR13bits.FLTA2R = 0x09;
    

    // Set both PWM output low on FAULT.
    // ?? Possible or don't in complementary mode?
    //P1FLTACONbits.FAOV2H = 0;
    //P1FLTACONbits.FAOV2L = 0;
    //P1FLTACONbits.FAOV3H = 0;
    //P1FLTACONbits.FAOV3L = 0;
    //P1FLTACONbits.FAEN2 = 1;
    //P1FLTACONbits.FAEN3 = 1;
    
    // Clear FLT1 interrupt flag
    IFS3bits.FLTA1IF = 0;
    // Clear FLT2 interrupt flag
    IFS4bits.FLTA2IF = 0;
    
    // And activate FAULT interrupts
    //IEC3bits.FLTA1IE = 1;
    //IEC4bits.FLTA2IE = 1;

    // Clear PWM interrupt flags
    IFS3bits.PWM1IF = 0;
    IFS4bits.PWM2IF = 0;
    
    // And disable PWM interrupts
    IEC3bits.PWM1IE = 0;
    IEC4bits.PWM2IE = 0;
    
    // Done, disable PWM for now
    P1TCONbits.PTEN = 0;
}

/******************************************************************************* 
 * Function:        InitTimer1 ()
 * Output:          None
 * Description:     Initialize Timer1
 * Note:            None
 ******************************************************************************/
void InitTimer1(void) {
	T1CONbits.TON = 0; 			// Disable Timer
	T1CONbits.TCS = 0; 			// Select internal instruction cycle clock
	T1CONbits.TGATE = 0; 		// Disable Gated Timer mode
	T1CONbits.TCKPS = 0b00; 	// Select 1:1 Prescaler
	T1CONbits.TSIDL = 0;		// Stop module operation in Idle mode

	TMR1 = 0x00; 				// Clear timer register
	PR1 = 4000;                  // Load the period value for 10 KHz (10KHz = 250)

	IPC0bits.T1IP = 4; 			// mid priority interrupts (highest priority interrupt = 7)
	IFS0bits.T1IF = 0; 			// Clear Timer1 Interrupt Flag
	IEC0bits.T1IE = 1; 			// Enable Timer1 interrupt
	T1CONbits.TON = 1; 			// Start Timer
}

/******************************************************************************* 
 * Function:        InitPorts ()
 * Output:          None
 * Description:     Initialize board peripherals, pins directions, etc
 * Note:            None
 ******************************************************************************/
void InitPeripherals(void) {
    /*
     * Initially set all ports as outputs, later on specialized init subroutines
     * override it and configure required pins with special functions and directions 
     */
    TRISA = 0;
    TRISB = 0;

    // Make sure L293 is disabled
    L298_ENABLE_W1 = 0;
    L298_ENABLE_W2 = 0;
    
    /*
     * Configure step and dir pins
     */
    TRISBbits.TRISB6 = 1;   // Direction
    CNPU2bits.CN24PUE = 1;  // Eliminate signals when step and dir are not connected
    
    TRISBbits.TRISB7 = 1;   // Step
    CNPU2bits.CN23PUE = 1;  // Eliminate signals when step and dir are not connected
    
    INTCON2bits.INT0EP = 1; // trigger on rising edge
    IPC0bits.INT0IP = 6;    // Step interrupt has highest priority.
    IFS0bits.INT0IF = 0;    // Clean INT0 interrupt flag
    IEC0bits.INT0IE = 0;    // Disable step interrupt

    /*
     * Configure fault pins 
     */
    // Set PR8 and PR9 as inputs
/*
    TRISBbits.TRISB8 = 1;
    TRISBbits.TRISB9 = 1;
	
    __builtin_write_OSCCONL(OSCCON & 0xbf);		//unlock peripheral remapping

	RPINR0bits.INT1R = 8;				//tie INT1 to RP8 which is RB8
	RPINR1bits.INT2R = 9;				//tie INT2 to RP9 which is RB9

	__builtin_write_OSCCONL(OSCCON | 0x40);		//lock remapping

	INTCON2bits.INT1EP = 0;			//trigger on rising edge
 	IPC5bits.INT1IP = 7;			//(7 is highest)
	IFS1bits.INT1IF = 0;			//clear the INT1 flag
	IEC1bits.INT1IE = 1;			//enable the INT1 interrupt

	INTCON2bits.INT2EP = 0;			//trigger on rising edge
 	IPC7bits.INT2IP = 7;			//(7 is highest)
	IFS1bits.INT2IF = 0;			//clear the INT2 flag
	IEC1bits.INT2IE = 1;			//enable the INT2 interrupt    
*/
    // Enable nested interrupts
    INTCON1bits.NSTDIS = 0;
}

/******************************************************************************* 
 * Function:        DisableOutput ()
 * Output:          None
 * Description:     Disable L298 output
 * Note:            None
 ******************************************************************************/
void DisableOutput () {
    L298_ENABLE_W1 = 0;
    L298_ENABLE_W2 = 0;
}

/******************************************************************************* 
 * Function:        EnableOutput ()
 * Output:          None
 * Description:     Enable L298 output
 * Note:            None
 ******************************************************************************/
void EnableOutput () {
    L298_ENABLE_W1 = 1;
    L298_ENABLE_W2 = 1;
}

/******************************************************************************* 
 * Function:        SetPWM ()
 * Output:          None
 * Description:     Loads PWM duty cycle and override registers with
 *                  previously calculated values
 * Note:            None
 ******************************************************************************/
void SetPwm() {
    P1DC3 = winding1.desired_pwm;
    P1DC2 = winding2.desired_pwm;
    //P1DC2 = MINIMUM_DUTY;
    //P1DC3 = MINIMUM_DUTY;

    unsigned short temp = P1OVDCON;
    temp &= ~0b11110000000000;
    
    if (winding1.current_direction == Forward) {
        temp |=  0b10000000000000;
    }
    else  {
        temp |=  0b01000000000000;
    }
    
    if (winding2.current_direction == Forward) {
        temp |=  0b00100000000000;
    }
    else  {
        temp |=  0b00010000000000;
    }

    P1OVDCON = temp;
}

/******************************************************************************* 
 * Function:        SetPWM ()
 * Output:          None
 * Description:     Converts PID output into desired PWM duty cycle 
 * Note:            None
 ******************************************************************************/
void UpdatePwm(WindingState *w, PowerSupplyState *ps, _Q16 new_target) {
    _Q16 limited_pid_output;
    // limit controller output to the available driver voltage
    if (new_target > ps->measured_voltage_dc_bus)
        limited_pid_output = ps->measured_voltage_dc_bus;
    else if (new_target < _Q16neg(ps->measured_voltage_dc_bus))
        limited_pid_output = _Q16neg(ps->measured_voltage_dc_bus);
    else
        limited_pid_output = new_target;
    
    w->desired_pwm = (int) _itofQ16 (_Q16div (limited_pid_output * (long)PWM_MAX, ps->measured_voltage_dc_bus));

    // Make sure calculated PWM value are within the acceptable PWM range
    if (w->desired_pwm > PWM_MAX)
        w->desired_pwm = PWM_MAX;
    if (w->desired_pwm < -PWM_MAX)
        w->desired_pwm = -PWM_MAX;

    // Limit the minimum PWM duty cycle and update winding direction
    if (w->desired_pwm >= 0) {
        w->current_direction = Forward;
        if (w->desired_pwm < PWM_MIN)
            w->desired_pwm = PWM_MIN;
    }
    else {
        w->current_direction = Reverse;
        w->desired_pwm = -w->desired_pwm;
        if (w->desired_pwm < PWM_MIN)
            w->desired_pwm = PWM_MIN;
    }
}
