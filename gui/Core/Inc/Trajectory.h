/*
 * Trajectory.h
 *
 * Created on: Apr 18, 2026
 * Author: Yhommy
 */

#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_

typedef struct {
    float q;      // Position in degrees
    float q_dot;  // Velocity in RPM
    float dt;     // Timer interrupt period (seconds)
} Trajectory_t;

void Trajectory_Init(Trajectory_t* Trajectory, float dt);
void Trajectory_Update(Trajectory_t* Trajectory);
void Trajectory_Start(Trajectory_t* Trajectory, float RPM);
void Trajectory_Stop(Trajectory_t* Trajectory);

#endif /* INC_TRAJECTORY_H_ */
