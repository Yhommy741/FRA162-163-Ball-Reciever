/*
 * RevRobot1DOF.c
 *
 * Created on: Apr 11, 2026
 * Author: Yhommy
 */

#include "RevRobot1DOF.h"
#include <math.h>

#define PI 3.14159265358979323846f

// ==============================================================================
// Initialization
// ==============================================================================

void RevRobot1DOF_init(RevRobot1DOF_t *robot) {
    robot->Observer_htim = NULL;
    robot->PosControl_htim = NULL;
    robot->VeloControl_htim = NULL;

    robot->Position = 0.0f;
    robot->Velocity = 0.0f;
    robot->Degree = 0.0f;
    robot->RPM = 0.0f;

    robot->TargetDegree = 0.0f;
    robot->TargetRPM = 0.0f;
    robot->FeedforwardRPM = 0.0f;
    robot->Mode = ROBOT_MODE_IDLE;

    robot->PrevEncoderRad = 0.0f;
    robot->ObserverDt = 0.001f;
}

void RevRobot1DOF_init_Motor(RevRobot1DOF_t *robot, TIM_HandleTypeDef *PWM_htim,
        uint16_t Motor_Ch1, uint16_t Motor_Ch2, GPIO_TypeDef *EN_Port, uint16_t EN_Pin) {
    PWM_t ch1_struct;
    ch1_struct.PWM_Channel = Motor_Ch1;
    PWM_t ch2_struct;
    ch2_struct.PWM_Channel = Motor_Ch2;

    MotorDriver_init(&robot->Motor, PWM_htim, ch1_struct, ch2_struct, EN_Port, EN_Pin);
}

void RevRobot1DOF_init_Encoder(RevRobot1DOF_t *robot, TIM_HandleTypeDef *QEI_htim,
        TIM_HandleTypeDef *Observer_htim, uint32_t PPR, uint32_t X,
        uint32_t QEI_OverflowCount, float ObserverPeriod) {
    robot->Observer_htim = Observer_htim;
    robot->ObserverDt = ObserverPeriod;
    QEI_init(&robot->Encoder, QEI_htim, Observer_htim, PPR, X, QEI_OverflowCount, ObserverPeriod);

    if (robot->Observer_htim != NULL) HAL_TIM_Base_Start_IT(robot->Observer_htim);
}

void RevRobot1DOF_init_Filter(RevRobot1DOF_t* robot, float velo_cutoff_hz, float dt) {
    LowpassFilter_Init(&robot->VeloFilter, velo_cutoff_hz, dt);
}

void RevRobot1DOF_init_Position_PID(RevRobot1DOF_t *robot, float Kp, float Ki,
        float Kd, float max_rpm_output, TIM_HandleTypeDef* PosControl_htim, float pos_dt) {
    robot->PosControl_htim = PosControl_htim;
    PID_Init(&robot->PosPID, Kp, Ki, Kd, max_rpm_output);

    // Initialize Trajectory generator here!
    Trajectory_Init(&robot->Trajectory, pos_dt);

    if (robot->PosControl_htim != NULL) HAL_TIM_Base_Start_IT(robot->PosControl_htim);
}

void RevRobot1DOF_init_Velocity_PID(RevRobot1DOF_t *robot, float Kp, float Ki,
        float Kd, float max_power_output, TIM_HandleTypeDef* VeloControl_htim) {
    robot->VeloControl_htim = VeloControl_htim;
    PID_Init(&robot->VeloPID, Kp, Ki, Kd, max_power_output);

    if (robot->VeloControl_htim != NULL) HAL_TIM_Base_Start_IT(robot->VeloControl_htim);
}

// ==============================================================================
// Background Timer Interrupt Handler
// ==============================================================================
void RevRobot1DOF_update(RevRobot1DOF_t *robot, TIM_HandleTypeDef *Timer) {

    // 1. OBSERVER LOOP
    if (Timer == robot->Observer_htim) {
        QEI_update(&robot->Encoder, Timer);

        float current_raw_rad = robot->Encoder.Rad;
        float raw_velocity = (current_raw_rad - robot->PrevEncoderRad) / robot->ObserverDt;
        robot->PrevEncoderRad = current_raw_rad;

        robot->Position = current_raw_rad;
        robot->Velocity = LowpassFilter(&robot->VeloFilter, raw_velocity);

        robot->Degree = robot->Position * 180.0f / PI;
        robot->RPM = robot->Velocity * 30.0f / PI;
    }

    // 2. POSITION LOOP - OUTER CASCADE
    if (Timer == robot->PosControl_htim) {
        if (robot->Mode == ROBOT_MODE_CASCADE_POSITION) {

            // 1. Manually tick the trajectory forward
            Trajectory_Update(&robot->Trajectory);

            // 2. Read 'q' and 'q_dot' directly from the library
            robot->TargetDegree = robot->Trajectory.q;
            robot->FeedforwardRPM = robot->Trajectory.q_dot;

            // 3. Shortest Path Error Calculation
            float error = robot->TargetDegree - robot->Degree;
            error = fmodf(error, 360.0f);
            if (error > 180.0f) error -= 360.0f;
            else if (error < -180.0f) error += 360.0f;

            // 4. Compute Position PID
            float pid_correction_rpm = PID_Compute(&robot->PosPID, error, 0.0f);

            // 5. Combine correction with feedforward
            robot->TargetRPM = pid_correction_rpm + robot->FeedforwardRPM;
        }
    }

    // 3. VELOCITY LOOP - INNER CASCADE
    if (Timer == robot->VeloControl_htim) {
        if (robot->Mode == ROBOT_MODE_VELOCITY || robot->Mode == ROBOT_MODE_CASCADE_POSITION) {
            float motor_power = PID_Compute(&robot->VeloPID, robot->TargetRPM, robot->RPM);
            MotorDriver_write(&robot->Motor, 5000, motor_power);
        }
    }
}

// ==============================================================================
// User Control Functions
// ==============================================================================

// Spin forever under tight position holding!
void RevRobot1DOF_spin_continuous_position(RevRobot1DOF_t* robot, float target_rpm){
    Trajectory_Start(&robot->Trajectory, target_rpm);

    // Sync the trajectory's starting point to current degree to avoid jumping
    robot->Trajectory.q = fmodf(robot->Degree, 360.0f);
    if (robot->Trajectory.q < 0.0f) robot->Trajectory.q += 360.0f;

    robot->Mode = ROBOT_MODE_CASCADE_POSITION;
}

// Snap directly to a degree and hold it
void RevRobot1DOF_set_degree(RevRobot1DOF_t* robot, float target_degree){
    Trajectory_Stop(&robot->Trajectory);
    robot->TargetDegree = target_degree;
    robot->Trajectory.q = target_degree; // Keep generator synced
    robot->FeedforwardRPM = 0.0f;
    robot->Mode = ROBOT_MODE_CASCADE_POSITION;
}

// Pure velocity loop (no position correction)
void RevRobot1DOF_set_RPM(RevRobot1DOF_t* robot, float target_rpm){
    Trajectory_Stop(&robot->Trajectory);
    robot->TargetRPM = target_rpm;
    robot->Mode = ROBOT_MODE_VELOCITY;
}

// Manual override
void RevRobot1DOF_set_power(RevRobot1DOF_t *robot, float power) {
    Trajectory_Stop(&robot->Trajectory);
    robot->Mode = ROBOT_MODE_IDLE;
    MotorDriver_write(&robot->Motor, 5000, power);
}
