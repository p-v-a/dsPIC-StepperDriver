/******************************************************************************* 
* File:   pid_controller.c
* Author: Vladimir Pasashnikov, vladimir.pasashnikov@gmail.com
*
* Implements PID controller:
*   - Full PID controller with parameters Kp, Ki, and Kd using backward difference relationship
*   - Back calculation anti-windup scheme with tracking gain Kt
*   - Derivative part as low path filter with parameter with parameter N
*   - Setpoints for proportional and derivative paths, with parameters Wp and Wd respectively
* Based on http://controlsystemslab.com/advanced-pid-controller-implementation/
* Site seems to be down, link to old version via web archive - https://web.archive.org/web/20170716045041/http://controlsystemslab.com/advanced-pid-controller-implementation/
*
* Created on 15 January 2018
******************************************************************************/

#ifndef PID_CONTROLLER_H
#define	PID_CONTROLLER_H

#include <libq.h>
#include <stdbool.h>

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct {
    /*
     * PID controller variables are all in Q15.16
     */
    _Q16 e0, e1, e2;            // True error, z, z^-1, and z^-2
    _Q16 ep0, ep1, ep2;         // Error, proportional path, z, z^-1, and z^-2
    _Q16 ed0, ed1, ed2;         // Error, derivative path, z, z^-1, and z^-2
    _Q16 eus0, eus1, eus2;      // Anti-windup calculation path, z, z^-1, and z^-2
    _Q16 u0, u1, u2;            // Controller output, z, z^-1, and z^-2
} PidControllerState;

typedef struct {
    /*
     * PID controller parameters are all in Q15.16.
     */
    _Q16 kp;                    // proportional gain
    _Q16 ki;                    // integral gain
    _Q16 kt;                    // tracking gain
    _Q16 kd;                    // derivative gain
    _Q16 wp;                    // proportional weight
    _Q16 wd;                    // derivative weight
    int n;                      // derivative filter coefficient

    /* PID control parameter limits - e.g. DAC max/min outputs or PWN max/min values */
    _Q16 output_max_limit;
    _Q16 output_min_limit;
    
    /* Sampling time interval */
    _Q16 ts;

    /* 
     * coefficients of PID algorithm derived from classic control parameters above.
     */
    _Q16 a1, a2;
    _Q16 b1, b2, b3;
    _Q16 c1, c2, c3, c4, c5, c6;
    _Q16 d1, d2, d3;
} PidControllerParameters;


void UpdatePid (PidControllerParameters *controller);
_Q16 CalculatePIDOutput (PidControllerParameters *c, PidControllerState *p, _Q16 reference_command, _Q16 plant_output);
        
#ifdef	__cplusplus
}
#endif

#endif	/* PID_CONTROLLER_H */

