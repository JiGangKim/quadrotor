/*
 * mycontroller.c - controller
 *
 *  Created on: 2016. 7. 18.
 *      Author: jgkim2020
 *
 * I do not take any responsibility for any consequences resulting from the
 * use of this software. USE AT YOUR OWN RISK!
 *
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "driverlib/pwm.h"
#include "inc/hw_memmap.h"
#include "myquaternion.h"
#include "myvariable.h"
#include "mykalman.h"
#include "mycontroller.h"
#include "myperipheral.h"

void PDPController(void)
{	// quaternion-based nonlinear p^2 controller
	// quaternion_delta = conj(quaternionK)*quaternion_set
	if(STATUS_CTRL_ad == -1)
	{
	 	euler_set[2] = eulerK[2];
	 	yaw_set_delta = 0.0;
	}
	else if(abs(Tick1000Hz - STATUS_CTRL_ad) > 400) STATUS_CTRL_ad = -1;
	else euler_set[2] = eulerK[2] + yaw_set_delta;
	if(STATUS_CTRL_58 == -1) euler_set[1] = 0.0;
	else if(abs(Tick1000Hz - STATUS_CTRL_46) > 400) STATUS_CTRL_58 = -1;
	if(STATUS_CTRL_46 == -1) euler_set[0] = 0.0;
	else if(abs(Tick1000Hz - STATUS_CTRL_46) > 400) STATUS_CTRL_46 = -1;

	if(STATUS_CTRL_p == 0 || thrust_base == 0)
 	{
 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PWMLoad*0.0);
 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PWMLoad*0.0);
 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWMLoad*0.0);
 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWMLoad*0.0);
 	}
 	else
 	{
 		int i;
 		float max = 0.0;
 		TBXYZtoQUATERNION(euler_set, quaternion_set);
 		quaternion_set[1] *= -1;
 		quaternion_set[2] *= -1;
 		quaternion_set[3] *= -1;
 		Quaternionmultiplication(quaternion_set, quaternionK, quaternion_delta);
 		quaternion_delta[1] *= -1;
 		quaternion_delta[2] *= -1;
 		quaternion_delta[3] *= -1;
 		if(quaternion_delta[0] < 0) // angle range 0 ~ 180 degree
 		{
 			quaternion_delta[0] *= -1;
 			quaternion_delta[1] *= -1;
 			quaternion_delta[2] *= -1;
 			quaternion_delta[3] *= -1;
 		}
 	 	angle_delta = 2*acosf(quaternion_delta[0]);
 	 	w_req[0] = (p1 + d1*(angle_delta - prev_angle_delta))*quaternion_delta[1];
 	 	w_req[1] = (p1 + d1*(angle_delta - prev_angle_delta))*quaternion_delta[2];
 	 	w_req[2] = (p1 + d1*(angle_delta - prev_angle_delta))*quaternion_delta[3];
 	 	prev_angle_delta = angle_delta;
 	 	torque_req[0] = p2*jx*(w_req[0] - w[0]);
 	 	torque_req[1] = p2*jy*(w_req[1] - w[1]);
 	 	torque_req[2] = p2*jz*(w_req[2] - w[2]);
 	 	thrust_delta[0] = -torque_req[0] + torque_req[1] + torque_req[2];
 	 	thrust_delta[1] = -torque_req[0] - torque_req[1] - torque_req[2];
 	 	thrust_delta[2] =  torque_req[0] - torque_req[1] + torque_req[2];
 	 	thrust_delta[3] =  torque_req[0] + torque_req[1] - torque_req[2];
 		for(i = 0; i < 4; i++)
 		{
 			if(fabsf(thrust_delta[i]) > max) max = fabsf(thrust_delta[i]);
 		}
 		if(max > l2)
 		{
 			for(i = 0; i < 4; i++) thrust_delta[i] = thrust_delta[i]/max*l2;
 		}
 		thrust[0] = 0.25*thrust_base + thrust_delta[0];
 		thrust[1] = 0.25*thrust_base + thrust_delta[1];
 		thrust[2] = 0.25*thrust_base + thrust_delta[2];
 		thrust[3] = 0.25*thrust_base + thrust_delta[3];

 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PWMLoad/100.0*LifttoPWM(thrust[0]));
 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PWMLoad/100.0*LifttoPWM(thrust[1]));
 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWMLoad/100.0*LifttoPWM(thrust[2]));
 		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWMLoad/100.0*LifttoPWM(thrust[3]));
 	}
}
