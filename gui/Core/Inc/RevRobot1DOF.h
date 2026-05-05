/*
 * RevRobot1DOF.h
 *
 * Created on: Apr 11, 2026
 * Author: Yhommy
 */

#ifndef INC_REVROBOT1DOF_H_
#define INC_REVROBOT1DOF_H_

#include "LowPassFilter.h"
#include "MotorDriver.h"
#include "QEI.h"
#include "Controller.h"
#include "Trajectory.h"

// --- Control State Machine ---
typedef enum {
    ROBOT_MODE_IDLE = 0,
    ROBOT_MODE_VELOCITY,
    ROBOT_MODE_CASCADE_POSITION
} RevRobot1DOF_Mode_t;

typedef struct {
    // --- Sub-Modules ---
    MotorDriver_t    Motor;
    QEI_t            Encoder;
    LowpassFilter_t  VeloFilter;
    PID_Controller_t PosPID;
    PID_Controller_t VeloPID;
    Trajectory_t     Trajectory;

    // --- Hardware Timers ---
    TIM_HandleTypeDef* Observer_htim;
    TIM_HandleTypeDef* PosControl_htim;
    TIM_HandleTypeDef* VeloControl_htim;

    // --- High-Level Robot States ---
    float Position;       // Raw Angular Position [Rad]
    float Velocity;       // Clean, filtered Angular Velocity [Rad/s]
    float Degree;         // Raw Angular Position [Degree]
    float RPM;            // Clean, filtered Angular Velocity [RPM]

    float PrevEncoderRad;
    float ObserverDt;

    // --- Control Targets ---
    RevRobot1DOF_Mode_t Mode;
    float TargetDegree;
    float TargetRPM;
    float FeedforwardRPM;

} RevRobot1DOF_t;

// ==============================================================================
// Initialization Prototypes
// ==============================================================================
void RevRobot1DOF_init(RevRobot1DOF_t* robot);
void RevRobot1DOF_init_Motor(RevRobot1DOF_t* robot, TIM_HandleTypeDef* PWM_htim, uint16_t Motor_Ch1, uint16_t Motor_Ch2, GPIO_TypeDef* EN_Port, uint16_t EN_Pin);
void RevRobot1DOF_init_Encoder(RevRobot1DOF_t* robot, TIM_HandleTypeDef* QEI_htim, TIM_HandleTypeDef* Observer_htim, uint32_t PPR, uint32_t X, uint32_t QEI_OverflowCount, float ObserverPeriod);
void RevRobot1DOF_init_Filter(RevRobot1DOF_t* robot, float velo_cutoff_hz, float dt);

void RevRobot1DOF_init_Position_PID(RevRobot1DOF_t* robot, float Kp, float Ki, float Kd, float max_rpm_output, TIM_HandleTypeDef* PosControl_htim, float pos_dt);
void RevRobot1DOF_init_Velocity_PID(RevRobot1DOF_t* robot, float Kp, float Ki, float Kd, float max_power_output, TIM_HandleTypeDef* VeloControl_htim);

// ==============================================================================
// State Updates & Control Prototypes
// ==============================================================================
void RevRobot1DOF_update(RevRobot1DOF_t* robot, TIM_HandleTypeDef* Timer);

void RevRobot1DOF_spin_continuous_position(RevRobot1DOF_t* robot, float target_rpm);
void RevRobot1DOF_set_degree(RevRobot1DOF_t* robot, float target_degree);
void RevRobot1DOF_set_RPM(RevRobot1DOF_t* robot, float target_rpm);
void RevRobot1DOF_set_power(RevRobot1DOF_t* robot, float power);

#endif /* INC_REVROBOT1DOF_H_ */
