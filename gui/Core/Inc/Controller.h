/*
 * Controller.h
 *
 * Created on: Apr 19, 2026
 * Author: Yhommy's
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float u;
    float u_max;
    float e[3];   // e[0] = e[K], e[1] = e[K-1], e[2] = e[K-2]
} PID_Controller_t;


void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float u_max);
float PID_Compute(PID_Controller_t *pid, float setpoint, float measurement);

#endif /* INC_CONTROLLER_H_ */
