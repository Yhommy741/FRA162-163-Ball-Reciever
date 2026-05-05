/*
 * MotorDriver.h
 *
 * Created on: Apr 9, 2026
 * Author: Yhommy
 */

#ifndef INC_MOTORDRIVER_H_
#define INC_MOTORDRIVER_H_

#include "PWM.h"

typedef struct {
    TIM_HandleTypeDef* PWM_htim;    // Timer PWM
    PWM_t In1;                      // PWM Channel 1
    PWM_t In2;                      // PWM Channel 2
    GPIO_TypeDef* EN_Port;          // GPIO ENA Port
    uint16_t EN_Pin;                // GPIO ENA Pin

    float DutyCycle1;               // PWM Channel 1 Value
    float DutyCycle2;               // PWM Channel 2 Value

} MotorDriver_t;

void MotorDriver_init(MotorDriver_t* MotorDriver, TIM_HandleTypeDef* PWM_htim, PWM_t PWM_Channel_1, PWM_t PWM_Channel_2, GPIO_TypeDef* EN_Port, uint16_t EN_Pin);
void MotorDriver_write(MotorDriver_t* MotorDriver ,uint16_t Frequency , float Power);
void MotorDriver_enable(MotorDriver_t* MotorDriver);
void MotorDriver_disable(MotorDriver_t* MotorDriver);

#endif /* INC_MOTORDRIVER_H_ */
