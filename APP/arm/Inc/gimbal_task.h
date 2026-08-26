#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include <stdint.h>

#define GIMBAL_PITCH_MIN_X10 2300U
#define GIMBAL_PITCH_MAX_X10 4050U
#define GIMBAL_PITCH_MOTOR_WRAP_X10 3600U
#define GIMBAL_PITCH_MOTOR_LOW_MAX_X10 450U
#define GIMBAL_INITIAL_YAW_X10 1300U
#define GIMBAL_INITIAL_PITCH_X10 3300U

typedef enum {
  GIMBAL_STATE_HOMING = 0,
  GIMBAL_STATE_MANUAL = 3,
  GIMBAL_STATE_HOME_FAULT = 4
} GimbalRunState_t;

typedef struct {
  int16_t yaw_target_rpm;
  int32_t yaw_target_x10;
  uint8_t yaw_target_valid;
  uint16_t pitch_target_x10;
  int16_t pitch_target_rpm;
  int32_t yaw_current_x10;
  int32_t pitch_current_x10;
  uint32_t feedback_tick_ms;
  uint8_t yaw_feedback_valid;
  uint8_t pitch_feedback_valid;
  uint8_t target_valid;
  uint8_t timed_out;
  uint8_t homed;
  uint8_t home_fault;
  uint8_t pitch_speed_active;
  uint8_t pitch_soft_limit_active;
  uint8_t run_state;
} GimbalStatus_t;

void StartGimbalTask(void *argument);
uint8_t Gimbal_GetStatus(GimbalStatus_t *status);
uint8_t Gimbal_SetBusinessTarget(int16_t yaw_rpm, uint16_t pitch_angle_x10);
uint8_t Gimbal_SetInitialPose(uint16_t yaw_angle_x10, uint16_t pitch_angle_x10);
uint8_t Gimbal_ReturnToInitialPose(void);
uint8_t Gimbal_SetYawSpeed(int16_t yaw_rpm);
uint8_t Gimbal_SetYawTarget(uint16_t yaw_angle_x10);
uint8_t Gimbal_SetPitchTarget(uint16_t pitch_angle_x10);
uint8_t Gimbal_SetPitchSpeed(int16_t pitch_rpm);
uint8_t Gimbal_StopAndHold(void);
uint8_t Gimbal_HandleControlCommandLine(const char *line,
                                        char *reply,
                                        uint16_t reply_size);
uint8_t Gimbal_NormalizePitchFeedbackX10(int32_t motor_angle_x10,
                                         int32_t *business_angle_x10);

#endif
