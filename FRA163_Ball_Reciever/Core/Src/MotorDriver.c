/*
 * MotorDriver.c
 *
 * Created on: Apr 11, 2026
 * Author: Yhommy
 */

#include "MotorDriver.h"

void MotorDriver_init(MotorDriver_t* MotorDriver, TIM_HandleTypeDef* PWM_htim, PWM_t PWM_Channel_1, PWM_t PWM_Channel_2, GPIO_TypeDef* EN_Port, uint16_t EN_Pin) {
    MotorDriver->PWM_htim = PWM_htim;
    MotorDriver->EN_Port = EN_Port;
    MotorDriver->EN_Pin = EN_Pin;

    PWM_init(&MotorDriver->In1, PWM_htim, PWM_Channel_1.PWM_Channel);
    PWM_init(&MotorDriver->In2, PWM_htim, PWM_Channel_2.PWM_Channel);

    MotorDriver->DutyCycle1 = 0.0f;
    MotorDriver->DutyCycle2 = 0.0f;

    MotorDriver_enable(MotorDriver);
}

void MotorDriver_write(MotorDriver_t* MotorDriver, uint16_t Frequency, float Power) {

    if (Power > 100.0f) Power = 100.0f;
    if (Power < -100.0f) Power = -100.0f;


    if (Power > 0.0f) {
        MotorDriver->DutyCycle1 = Power;
        MotorDriver->DutyCycle2 = 0.0f;
    } else if (Power < 0.0f) {
        MotorDriver->DutyCycle1 = 0.0f;
        MotorDriver->DutyCycle2 = -Power;
    } else {
        MotorDriver->DutyCycle1 = 0.0f;
        MotorDriver->DutyCycle2 = 0.0f;
    }


    PWM_write(&MotorDriver->In1, (float)Frequency, MotorDriver->DutyCycle1);
    PWM_write(&MotorDriver->In2, (float)Frequency, MotorDriver->DutyCycle2);
}

void MotorDriver_enable(MotorDriver_t* MotorDriver) {

    HAL_GPIO_WritePin(MotorDriver->EN_Port, MotorDriver->EN_Pin, GPIO_PIN_SET);
}

void MotorDriver_disable(MotorDriver_t* MotorDriver) {

    HAL_GPIO_WritePin(MotorDriver->EN_Port, MotorDriver->EN_Pin, GPIO_PIN_RESET);
}
