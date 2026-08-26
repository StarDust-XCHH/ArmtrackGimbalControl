#ifndef BLDC_MOTOR_H
#define BLDC_MOTOR_H

#include <stdint.h>

#define BLDC_ADDR_YAW   0x01U
#define BLDC_ADDR_PITCH 0x02U

#define BLDC_MODE_SPEED 0x0000U
#define BLDC_MODE_MULTI_POS_T 0x0001U
#define BLDC_MODE_SINGLE_POS_T 0x0002U
#define BLDC_MODE_MULTI_POS_DIRECT 0x0003U
#define BLDC_MODE_SINGLE_POS_DIRECT 0x0004U
#define BLDC_MODE_SINGLE_POS_L BLDC_MODE_SINGLE_POS_DIRECT
#define BLDC_MODE_DEFAULT_POSITION BLDC_MODE_MULTI_POS_T
#define BLDC_FEEDBACK_SPEED 0x00U
#define BLDC_FEEDBACK_MULTI_ANGLE 0x01U
#define BLDC_FEEDBACK_SINGLE_ANGLE 0x02U
/* BLDC_GetLastUartError() value used when a feedback request receives no
 * complete response frame.  It is distinct from STM32 HAL UART errors. */
#define BLDC_UART_ERROR_FEEDBACK_TIMEOUT 0x80000001UL

void BLDC_SendCmd(uint8_t addr, uint8_t cmd, const uint8_t *data, uint8_t len);
void BLDC_Enable(uint8_t addr);
void BLDC_Disable(uint8_t addr);
void BLDC_SetMode(uint8_t addr, uint16_t mode);
void BLDC_SetSpeed(uint8_t addr, int16_t rpm);
void BLDC_SetPosition(uint8_t addr, int32_t angle_x10);
void BLDC_SetSingleAngle(uint8_t addr, uint16_t angle_x10);
void BLDC_SetAcc(uint8_t addr, uint16_t acc);
uint8_t BLDC_RequestFeedbackValue(uint8_t addr, uint8_t type, int32_t *value);
uint32_t BLDC_GetLastUartError(void);
uint8_t BLDC_GetStartupAttemptCount(void);
uint8_t BLDC_GetStartupFeedbackOk(uint8_t addr);
void BLDC_GimbalPowerOnWake(void);
void BLDC_GimbalStartTracking(uint16_t pitch_angle_x10, int16_t pitch_limit_rpm);
void BLDC_GimbalSelectTrackingMode(uint16_t pitch_angle_x10, int16_t pitch_limit_rpm);
void BLDC_GimbalRefreshTracking(int16_t yaw_rpm, uint16_t pitch_angle_x10);
void BLDC_GimbalRefreshSpeedTracking(int16_t yaw_rpm, int16_t pitch_rpm);
void BLDC_GimbalStartYawSpeed(int16_t yaw_rpm);
void BLDC_GimbalRefreshYawSpeed(int16_t yaw_rpm);
void BLDC_GimbalStartYawPosition(int32_t yaw_angle_x10, int16_t yaw_limit_rpm);
void BLDC_GimbalStartYawSingleAngle(uint16_t yaw_angle_x10, int16_t yaw_limit_rpm);
void BLDC_GimbalArmYawSingleAngle(uint16_t yaw_angle_x10, int16_t yaw_limit_rpm);
void BLDC_GimbalRefreshYawSingleAngle(uint16_t yaw_angle_x10);
void BLDC_GimbalStartPitchPosition(uint16_t pitch_angle_x10, int16_t pitch_limit_rpm);
void BLDC_GimbalArmPitchPosition(uint16_t pitch_angle_x10, int16_t pitch_limit_rpm);
void BLDC_GimbalStartPitchSpeed(int16_t pitch_rpm);
void BLDC_GimbalRefreshPitchSpeed(int16_t pitch_rpm);

#endif
