/*
 * Trajectory.c
 *
 * Created on: Apr 18, 2026
 * Author: Yhommy
 */

#include "Trajectory.h"

void Trajectory_Init(Trajectory_t* Trajectory, float dt) {
    Trajectory->dt = dt;
    Trajectory->q = 0.0f;
    Trajectory->q_dot = 0.0f;
}

void Trajectory_Start(Trajectory_t* Trajectory, float RPM) {
    // Set the desired velocity (q_dot)
    Trajectory->q_dot = RPM;
}

void Trajectory_Stop(Trajectory_t* Trajectory) {
    // Halting the velocity stops the trajectory from advancing
    Trajectory->q_dot = 0.0f;
}

void Trajectory_Update(Trajectory_t* Trajectory) {
    // If we have an active velocity, calculate the new position
    if (Trajectory->q_dot != 0.0f) {

        float deg_per_sec = Trajectory->q_dot * 6.0f;

        // Calculate how far to move in this specific time step
        float step = deg_per_sec * Trajectory->dt;

        // Advance the position (q)
        Trajectory->q += step;

        // --- PREVENT FLOAT OVERFLOW ---
        // Wrap the target angle perfectly between 0.0 and 360.0
        if (Trajectory->q >= 360.0f) {
            Trajectory->q -= 360.0f;
        } else if (Trajectory->q < 0.0f) {
            Trajectory->q += 360.0f;
        }
    }
}
