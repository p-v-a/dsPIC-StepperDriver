/******************************************************************************* 
 * File:   pid_controller.c
 * Author: Vladimir Pasashnikov, vladimir.pasashnikov@gmail.com
 *
 * Created on 31 August 2017
 ******************************************************************************/
#include <libq.h>
#include "pid_controller.h"

/******************************************************************************* 
 * Function:        UpdatePid (PidControllerParameters *controller)
 * Output:          None
 * Description:     Initializes PID coefficients in Controller - a1, a2, etc.
 * Note:            Assumes that PID parameters have been configured - kp, ki, kd, n, kt, wp, wd and ts
 ******************************************************************************/
void UpdatePid (PidControllerParameters *controller) {
    _Q16 kOne = _Q16ftoi(1.0);
    _Q16 kTwo = _Q16ftoi(2.0);
    
    _Q16 x1 = _Q16mac (controller->n, controller->ts, kOne);
    _Q16 x2 = _Q16mac (controller->n, controller->ts,  kTwo);
    
    controller->a1 = _Q16div(x2, x1);
    controller->a2 = _Q16div(_Q16neg(kOne), x1);

    controller->b1 = controller->kp;
    controller->b2 = _Q16neg(_Q16div(_Q16mpy(controller->kp, x2),x1));
    controller->b3 = _Q16div(controller->kp, x1);
    
    controller->c1 = _Q16mpy(controller->ki, controller->ts);
    controller->c2 = _Q16neg(_Q16div(_Q16mpy(controller->ki, controller->ts),x1));
    controller->c3 = _Q16mpy(controller->kt, controller->ts);
    controller->c4 = _Q16neg(_Q16div(_Q16mpy(controller->kt, controller->ts),x1));

    controller->d1 = _Q16div(_Q16mpy(controller->kd, controller->n),x1);
    controller->d2 = _Q16neg(_Q16div(_Q16mpy(_Q16mpy(controller->kd, controller->n), kTwo),x1));
    controller->d3 = _Q16div(_Q16mpy(controller->kd, controller->n),x1);
}

/******************************************************************************* 
 * Function:        CalculatePIDOutput (PidControllerParameters *c, PidControllerState *p, _Q16 target_current, _Q16 measured_current)
 * Output:          Calculated new controller output in Q16 format
 * Description:     Calculates new PID output for new sample period
 * Note:            See http://controlsystemslab.com/advanced-pid-controller-implementation/ for exact PID transfer function
 ******************************************************************************/
_Q16 CalculatePIDOutput (PidControllerParameters *c, PidControllerState *p, _Q16 reference_command, _Q16 plant_output) {
    // PID output historic values
    p->u2 = p->u1;
    p->u1 = p->u0;
    
    // weighted proportional error
    p->ep2 = p->ep1;
    p->ep1 = p->ep0;
    p->ep0 = _Q16mac(c->wp, reference_command, _Q16neg(plant_output)); 
    
    // true error
    p->e1 = p->e0;
    p->e0 = reference_command - plant_output; 
    
    // back calculation error
    p->eus1 = p->eus0;      
    if (p->u0 <= c->output_max_limit && p->u0 >= c->output_min_limit) {
        p->eus0 = 0;
    }
    else if (p->u0 > c->output_max_limit) {
        p->eus0 = c->output_max_limit - p->u0;
    }
    else {
        p->eus0 = _Q16neg(p->u0) - c->output_max_limit;
    }
    
    //weighted derivative error
    p->ed2 = p->ed1;
    p->ed1 = p->ed0;
    p->ed0 = _Q16mac(c->wd, reference_command, _Q16neg(plant_output)); 
    
    // Final sums and multiplications to obtain PID output
    p->u0 = _Q16mpy(c->a1, p->u1);
    p->u0 = _Q16mac(c->a2, p->u2, p->u0);
    
    p->u0 = _Q16mac(c->b1, p->ep0, p->u0);
    p->u0 = _Q16mac(c->b2, p->ep1, p->u0);
    p->u0 = _Q16mac(c->b3, p->ep2, p->u0);
    
    p->u0 = _Q16mac(c->c1, p->e0, p->u0);
    p->u0 = _Q16mac(c->c2, p->e1, p->u0);
    p->u0 = _Q16mac(c->c3, p->eus0, p->u0);
    p->u0 = _Q16mac(c->c4, p->eus1, p->u0);
    
    p->u0 = _Q16mac(c->d1, p->ed0, p->u0);
    p->u0 = _Q16mac(c->d2, p->ed1, p->u0);
    p->u0 = _Q16mac(c->d3, p->ed2, p->u0);

    return p->u0;
}
