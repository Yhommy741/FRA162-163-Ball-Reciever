/*
 * LowPassFilter.c
 *
 *  Created on: Apr 19, 2026
 *      Author: Yhommy
 */

#include "LowPassFilter.h"

void LowpassFilter_Init(LowpassFilter_t* Filter,float cutoff_freq, float dt) {
    // Clear history arrays
    Filter->y[0] = 0.0f; // Current output (y_k)
    Filter->y[1] = 0.0f; // Previous output (y_{k-1})
    Filter->u[0] = 0.0f; // Current input (x_k)
    Filter->u[1] = 0.0f; // Previous input (x_{k-1})
    Filter->Wc = cutoff_freq;
    Filter->Ts = dt;

    // Pre-calculate the T*Wc term to save processing power
    float TWc = Filter->Ts * Filter->Wc;
    float denominator = 2.0f + TWc;

    // Prevent divide-by-zero just in case Ts or Wc are improperly set
    if (denominator != 0.0f) {
        Filter->a = (2.0f - TWc) / denominator;
        Filter->b = TWc / denominator;
    } else {
        Filter->a = 0.0f;
        Filter->b = 0.0f;
    }
}

float LowpassFilter(LowpassFilter_t* Filter, float input) {
    // 1. Shift the history arrays back by one time step
    Filter->u[1] = Filter->u[0]; // x_{k-1} = x_k
    Filter->y[1] = Filter->y[0]; // y_{k-1} = y_k

    // 2. Store the brand new input
    Filter->u[0] = input;        // x_k

    // 3. Compute the new output using the Bilinear Transform formula:
    // y_k = a * y_{k-1} + b * (x_k + x_{k-1})
    Filter->y[0] = (Filter->a * Filter->y[1]) + (Filter->b * (Filter->u[0] + Filter->u[1]));

    // 4. Return the filtered output
    return Filter->y[0];
}
