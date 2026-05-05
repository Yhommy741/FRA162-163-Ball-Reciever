/*
 * LowPassFilter.h
 *
 *  Created on: Apr 19, 2026
 *      Author: Yhommy
 */

#ifndef INC_LOWPASSFILTER_H_
#define INC_LOWPASSFILTER_H_

typedef struct{
	float y[2];	// Output
	float u[2];	// Input
	float Wc;	// Cut-Off Frequency
	float Ts;	// Sampling Time
	float a;	// Constant = [(2-Ts*Wc)/2+Ts*Wc]
	float b;	// Constant = [T*Wc/(2+Ts*Wc)]
}LowpassFilter_t;

void LowpassFilter_Init(LowpassFilter_t* Filter,float cutoff_freq, float dt);
float LowpassFilter(LowpassFilter_t* Filter,float input);

#endif /* INC_LOWPASSFILTER_H_ */
