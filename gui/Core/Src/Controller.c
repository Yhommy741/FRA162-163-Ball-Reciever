/*
 * Controller.c
 *
 * Created on: Apr 19, 2026
 * Author: Yhommy
 */

#include "Controller.h"

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float u_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->u_max = u_max;

    pid->u = 0.0f;
    pid->e[0] = 0.0f;
    pid->e[1] = 0.0f;
    pid->e[2] = 0.0f;
}

float PID_Compute(PID_Controller_t *pid, float setpoint, float measurement) {

    pid->e[2] = pid->e[1];
    pid->e[1] = pid->e[0];

    pid->e[0] = setpoint - measurement;
    float ek = pid->e[0];

    if (!((pid->u >= pid->u_max && ek > 0) || (pid->u <= -pid->u_max && ek < 0))) {
        pid->u += ((pid->Kp + pid->Ki + pid->Kd) * ek)
                - ((pid->Kp + (2.0f * pid->Kd)) * pid->e[1])
                + (pid->Kd * pid->e[2]);
    }

    if (pid->u > pid->u_max) {
        pid->u = pid->u_max;
    } else if (pid->u < -pid->u_max) {
        pid->u = -pid->u_max;
    }

    return pid->u;
}
