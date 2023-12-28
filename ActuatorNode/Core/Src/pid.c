/*
 * pid.c
 *
 *  Created on: Dec 28, 2023
 *      Author: 20521
 */


#include "pid.h"
#include "motor.h"
float kp = 2.2;
float ki = 0.18;
float kd = 0.003;
float errorIntergral = 0, prevError = 0, error;
float outputPID;
float currT = 0, prevT = 0, deltaT;
extern int timeCountPID;
/*
 * @brief apply PID algorithm for DC motor
 * @param refVel: reference velocity
 * @param currVel: current velocity
 * @return velocity control output
 */
float PID(float refVel, float currVel)
{
	if (timeCountPID >= 100)
	{
		error = refVel - currVel;
		currT = HAL_GetTick();
		deltaT = (currT - prevT) / 1000; // convert millisecond to second
		prevT = currT;

		errorIntergral += error*deltaT;
		outputPID = kp*error + ki*errorIntergral + kd*(error - prevError)/deltaT;
		prevError = error;
		timeCountPID = 0;
	}
	return outputPID;
}
