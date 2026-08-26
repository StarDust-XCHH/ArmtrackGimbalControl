#include "gimbal_task.h"

#include <stdio.h>
#include <string.h>

#include "bldc_motor.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

#define GIMBAL_PERIOD_MS 5U
#define GIMBAL_HOME_YAW_LIMIT_RPM 5
#define GIMBAL_HOME_PITCH_LIMIT_RPM 5
#define GIMBAL_POSITION_LIMIT_RPM 20
#define GIMBAL_HOME_STOP_SETTLE_MS 100U
#define GIMBAL_HOME_TARGET_CONFIRM_MS 50U
#define GIMBAL_HOME_FEEDBACK_MS 100U
#define GIMBAL_HOME_TIMEOUT_MS 15000U
#define GIMBAL_HOME_TOLERANCE_X10 20L
#define GIMBAL_HOME_STABLE_SAMPLES 3U
#define GIMBAL_STATUS_FEEDBACK_MS 250U
#define GIMBAL_PITCH_SPEED_FEEDBACK_MS 50U
#define GIMBAL_PITCH_SPEED_STOP_MARGIN_X10 30L
#define GIMBAL_PITCH_POSITIVE_RPM_BUSINESS_DIR (-1)
#define GIMBAL_POSITION_REFRESH_MS 250U
#define GIMBAL_YAW_LIMIT_RPM 100
#define GIMBAL_PITCH_SPEED_LIMIT_RPM 30
#define GIMBAL_AXIS_YAW 0x01U
#define GIMBAL_AXIS_PITCH 0x02U
#define GIMBAL_AXIS_BOTH (GIMBAL_AXIS_YAW | GIMBAL_AXIS_PITCH)

typedef enum {
  CONTROL_HOME = 0,
  CONTROL_BUSINESS,
  CONTROL_POSE,
  CONTROL_YAW_POSITION,
  CONTROL_PITCH_SPEED
} ControlMode_t;

extern osMutexId_t debugUartMutexHandle;

static GimbalStatus_t gimbal_status;
static uint8_t gimbal_status_valid;
static volatile uint8_t gimbal_home_ready;
static uint16_t last_pitch_target = GIMBAL_INITIAL_PITCH_X10;
static ControlMode_t command_mode = CONTROL_HOME;
static int16_t command_yaw_rpm;
static uint16_t command_yaw_angle = GIMBAL_INITIAL_YAW_X10;
static uint16_t command_pitch_angle = GIMBAL_INITIAL_PITCH_X10;
static int16_t command_pitch_rpm;
static uint8_t command_axis = GIMBAL_AXIS_BOTH;
static uint32_t command_sequence = 1U;

static int32_t abs_i32(int32_t value) { return value < 0 ? -value : value; }

static float abs_float(float value) { return value < 0.0f ? -value : value; }

static int32_t round_i32(float value)
{
  return value >= 0.0f ? (int32_t)(value + 0.5f) :
                         (int32_t)(value - 0.5f);
}

static uint16_t clamp_pitch(int32_t angle)
{
  if (angle < (int32_t)GIMBAL_PITCH_MIN_X10) return GIMBAL_PITCH_MIN_X10;
  if (angle > (int32_t)GIMBAL_PITCH_MAX_X10) return GIMBAL_PITCH_MAX_X10;
  return (uint16_t)angle;
}

static uint16_t pitch_to_motor(uint16_t angle)
{
  angle = clamp_pitch(angle);
  return angle >= GIMBAL_PITCH_MOTOR_WRAP_X10 ?
      (uint16_t)(angle - GIMBAL_PITCH_MOTOR_WRAP_X10) : angle;
}

static int32_t normalize_yaw(int32_t angle)
{
  angle %= 3600L;
  if (angle < 0) angle += 3600L;
  return angle;
}

static int32_t yaw_distance(int32_t actual, int32_t target)
{
  int32_t distance = abs_i32(normalize_yaw(actual) - normalize_yaw(target));
  return distance > 1800L ? 3600L - distance : distance;
}

uint8_t Gimbal_NormalizePitchFeedbackX10(int32_t motor_angle_x10,
                                         int32_t *business_angle_x10)
{
  int32_t turn;
  if (business_angle_x10 == NULL) return 0U;
  turn = motor_angle_x10 % 3600L;
  if (turn < 0) turn += 3600L;
  if (turn >= (int32_t)GIMBAL_PITCH_MIN_X10)
  {
    *business_angle_x10 = turn;
    return 1U;
  }
  if (turn <= (int32_t)GIMBAL_PITCH_MOTOR_LOW_MAX_X10)
  {
    *business_angle_x10 = turn + 3600L;
    return 1U;
  }
  return 0U;
}

static void publish_status(const GimbalStatus_t *status)
{
  if (status == NULL) return;
  taskENTER_CRITICAL();
  gimbal_status = *status;
  last_pitch_target = status->pitch_target_x10;
  gimbal_status_valid = 1U;
  taskEXIT_CRITICAL();
}

uint8_t Gimbal_GetStatus(GimbalStatus_t *status)
{
  uint8_t valid;
  if (status == NULL) return 0U;
  taskENTER_CRITICAL();
  valid = gimbal_status_valid;
  if (valid != 0U) *status = gimbal_status;
  taskEXIT_CRITICAL();
  return valid;
}

static uint16_t get_last_pitch_target(void)
{
  uint16_t value;
  taskENTER_CRITICAL();
  value = last_pitch_target;
  taskEXIT_CRITICAL();
  return value;
}

static void submit_command(ControlMode_t mode, int16_t yaw_rpm,
                           uint16_t yaw_angle, uint16_t pitch_angle,
                           int16_t pitch_rpm, uint8_t axis)
{
  taskENTER_CRITICAL();
  command_mode = mode;
  command_yaw_rpm = yaw_rpm;
  command_yaw_angle = yaw_angle;
  command_pitch_angle = clamp_pitch(pitch_angle);
  command_pitch_rpm = pitch_rpm;
  command_axis = axis;
  command_sequence++;
  taskEXIT_CRITICAL();
}

uint8_t Gimbal_SetBusinessTarget(int16_t yaw_rpm, uint16_t pitch_angle_x10)
{
  if (yaw_rpm < -GIMBAL_YAW_LIMIT_RPM || yaw_rpm > GIMBAL_YAW_LIMIT_RPM)
    return 0U;
  submit_command(CONTROL_BUSINESS, yaw_rpm, 0U, pitch_angle_x10, 0,
                 GIMBAL_AXIS_BOTH);
  return 1U;
}

uint8_t Gimbal_SetInitialPose(uint16_t yaw_angle_x10, uint16_t pitch_angle_x10)
{
  if (yaw_angle_x10 > 3599U || pitch_angle_x10 < GIMBAL_PITCH_MIN_X10 ||
      pitch_angle_x10 > GIMBAL_PITCH_MAX_X10) return 0U;
  submit_command(CONTROL_POSE, 0, yaw_angle_x10, pitch_angle_x10, 0,
                 GIMBAL_AXIS_BOTH);
  return 1U;
}

uint8_t Gimbal_ReturnToInitialPose(void)
{
  taskENTER_CRITICAL();
  gimbal_home_ready = 0U;
  taskEXIT_CRITICAL();
  submit_command(CONTROL_HOME, 0, GIMBAL_INITIAL_YAW_X10,
                 GIMBAL_INITIAL_PITCH_X10, 0, GIMBAL_AXIS_BOTH);
  return 1U;
}

uint8_t Gimbal_SetYawSpeed(int16_t yaw_rpm)
{
  if (yaw_rpm < -GIMBAL_YAW_LIMIT_RPM || yaw_rpm > GIMBAL_YAW_LIMIT_RPM)
    return 0U;
  submit_command(CONTROL_BUSINESS, yaw_rpm, 0U, get_last_pitch_target(), 0,
                 GIMBAL_AXIS_YAW);
  return 1U;
}

uint8_t Gimbal_SetYawTarget(uint16_t yaw_angle_x10)
{
  if (yaw_angle_x10 > 3599U) return 0U;
  submit_command(CONTROL_YAW_POSITION, 0, yaw_angle_x10,
                 get_last_pitch_target(), 0, GIMBAL_AXIS_YAW);
  return 1U;
}

uint8_t Gimbal_SetPitchTarget(uint16_t pitch_angle_x10)
{
  if (pitch_angle_x10 < GIMBAL_PITCH_MIN_X10 ||
      pitch_angle_x10 > GIMBAL_PITCH_MAX_X10) return 0U;
  submit_command(CONTROL_BUSINESS, 0, 0U, pitch_angle_x10, 0,
                 GIMBAL_AXIS_PITCH);
  return 1U;
}

uint8_t Gimbal_SetPitchSpeed(int16_t pitch_rpm)
{
  if (pitch_rpm < -GIMBAL_PITCH_SPEED_LIMIT_RPM ||
      pitch_rpm > GIMBAL_PITCH_SPEED_LIMIT_RPM) return 0U;
  submit_command(CONTROL_PITCH_SPEED, 0, 0U, get_last_pitch_target(),
                 pitch_rpm, GIMBAL_AXIS_PITCH);
  return 1U;
}

uint8_t Gimbal_StopAndHold(void)
{
  GimbalStatus_t status;
  uint16_t yaw = GIMBAL_INITIAL_YAW_X10;
  uint16_t pitch = get_last_pitch_target();
  if (Gimbal_GetStatus(&status) != 0U)
  {
    if (status.yaw_feedback_valid != 0U) yaw = (uint16_t)normalize_yaw(status.yaw_current_x10);
    else if (status.yaw_target_valid != 0U) yaw = (uint16_t)normalize_yaw(status.yaw_target_x10);
    if (status.pitch_feedback_valid != 0U) pitch = clamp_pitch(status.pitch_current_x10);
  }
  return Gimbal_SetInitialPose(yaw, pitch);
}

static void debug_write(const char *text)
{
  uint16_t length = 0U;
  while (text != NULL && text[length] != '\0') length++;
  if (debugUartMutexHandle != NULL) (void)osMutexAcquire(debugUartMutexHandle, osWaitForever);
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)text, length, 100U);
  if (debugUartMutexHandle != NULL) (void)osMutexRelease(debugUartMutexHandle);
}

static uint8_t read_yaw_feedback(GimbalStatus_t *status)
{
  int32_t yaw_motor;
  uint8_t yaw_ok = BLDC_RequestFeedbackValue(BLDC_ADDR_YAW,
                                              BLDC_FEEDBACK_SINGLE_ANGLE,
                                              &yaw_motor);

  status->yaw_feedback_valid = yaw_ok;
  if (yaw_ok != 0U)
  {
    status->yaw_current_x10 = normalize_yaw(yaw_motor);
    status->feedback_tick_ms = HAL_GetTick();
  }
  return yaw_ok;
}

static uint8_t read_pitch_feedback(GimbalStatus_t *status)
{
  int32_t pitch_motor;
  int32_t pitch_business;
  uint8_t pitch_ok = BLDC_RequestFeedbackValue(BLDC_ADDR_PITCH,
                                                BLDC_FEEDBACK_SINGLE_ANGLE,
                                                &pitch_motor);

  if ((pitch_ok != 0U) &&
      (Gimbal_NormalizePitchFeedbackX10(pitch_motor, &pitch_business) != 0U))
  {
    status->pitch_current_x10 = pitch_business;
    status->feedback_tick_ms = HAL_GetTick();
  }
  else
  {
    pitch_ok = 0U;
  }
  status->pitch_feedback_valid = pitch_ok;
  return pitch_ok;
}

static uint8_t read_feedback(GimbalStatus_t *status)
{
  uint8_t yaw_ok = read_yaw_feedback(status);
  uint8_t pitch_ok;

  osDelay(2U);
  pitch_ok = read_pitch_feedback(status);
  return ((yaw_ok != 0U) && (pitch_ok != 0U)) ? 1U : 0U;
}

static int16_t limit_pitch_speed(int16_t requested_rpm, GimbalStatus_t *status)
{
  int32_t direction;
  int32_t projected;
  int32_t margin;
  status->pitch_soft_limit_active = 0U;
  if (requested_rpm == 0 || status->pitch_feedback_valid == 0U) return 0;
  direction = (int32_t)requested_rpm * GIMBAL_PITCH_POSITIVE_RPM_BUSINESS_DIR;
  projected = (abs_i32((int32_t)requested_rpm) * 60L *
               (int32_t)(2U * GIMBAL_PITCH_SPEED_FEEDBACK_MS)) / 1000L;
  margin = GIMBAL_PITCH_SPEED_STOP_MARGIN_X10 + projected;
  if ((status->pitch_current_x10 < (int32_t)GIMBAL_PITCH_MIN_X10) ||
      (status->pitch_current_x10 > (int32_t)GIMBAL_PITCH_MAX_X10) ||
      ((direction > 0) && (status->pitch_current_x10 >=
       (int32_t)GIMBAL_PITCH_MAX_X10 - margin)) ||
      ((direction < 0) && (status->pitch_current_x10 <=
       (int32_t)GIMBAL_PITCH_MIN_X10 + margin)))
  {
    status->pitch_soft_limit_active = 1U;
    return 0;
  }
  return requested_rpm;
}

static int16_t limit_pitch_speed_by_feedback(int16_t requested_rpm,
                                             GimbalStatus_t *status)
{
  if (read_pitch_feedback(status) == 0U)
  {
    status->pitch_soft_limit_active = 0U;
    return 0;
  }
  return limit_pitch_speed(requested_rpm, status);
}

static uint8_t execute_home(GimbalStatus_t *status, uint16_t pitch_target)
{
  uint32_t started;
  uint32_t last_feedback_report = 0U;
  uint8_t stable = 0U;
  uint8_t feedback_reported = 0U;
  gimbal_home_ready = 0U;
  status->run_state = GIMBAL_STATE_HOMING;
  status->homed = 0U;
  status->home_fault = 0U;
  status->target_valid = 0U;
  status->timed_out = 1U;
  status->yaw_target_rpm = 0;
  status->yaw_target_x10 = GIMBAL_INITIAL_YAW_X10;
  status->yaw_target_valid = 1U;
  status->pitch_target_x10 = pitch_target;
  status->pitch_target_rpm = 0;
  status->pitch_speed_active = 0U;
  status->pitch_soft_limit_active = 0U;
  publish_status(status);

  BLDC_GimbalStartYawSpeed(0);
  osDelay(GIMBAL_HOME_STOP_SETTLE_MS);
  BLDC_GimbalArmYawSingleAngle(GIMBAL_INITIAL_YAW_X10, GIMBAL_HOME_YAW_LIMIT_RPM);
  osDelay(GIMBAL_HOME_TARGET_CONFIRM_MS);
  BLDC_GimbalRefreshYawSingleAngle(GIMBAL_INITIAL_YAW_X10);
  BLDC_GimbalArmPitchPosition(pitch_to_motor(pitch_target), GIMBAL_HOME_PITCH_LIMIT_RPM);
  started = HAL_GetTick();

  while ((HAL_GetTick() - started) < GIMBAL_HOME_TIMEOUT_MS)
  {
    if (read_feedback(status) != 0U &&
        yaw_distance(status->yaw_current_x10, GIMBAL_INITIAL_YAW_X10) <= GIMBAL_HOME_TOLERANCE_X10 &&
        abs_i32(status->pitch_current_x10 - pitch_target) <= GIMBAL_HOME_TOLERANCE_X10)
      stable++;
    else stable = 0U;
    publish_status(status);
    if ((feedback_reported == 0U) ||
        ((HAL_GetTick() - last_feedback_report) >= 1000U))
    {
      char feedback_line[180];
      (void)snprintf(feedback_line,
                     sizeof(feedback_line),
                     "GIMBAL home feedback yaw_ok=%u pitch_ok=%u yaw=%ld.%ld pitch=%ld.%ld uart_err=0x%08lX\r\n",
                     (unsigned int)status->yaw_feedback_valid,
                     (unsigned int)status->pitch_feedback_valid,
                     (long)(status->yaw_current_x10 / 10L),
                     (long)abs_i32(status->yaw_current_x10 % 10L),
                     (long)(status->pitch_current_x10 / 10L),
                     (long)abs_i32(status->pitch_current_x10 % 10L),
                     (unsigned long)BLDC_GetLastUartError());
      debug_write(feedback_line);
      feedback_reported = 1U;
      last_feedback_report = HAL_GetTick();
    }
    if (stable >= GIMBAL_HOME_STABLE_SAMPLES)
    {
      status->run_state = GIMBAL_STATE_MANUAL;
      status->homed = 1U;
      status->home_fault = 0U;
      status->timed_out = 0U;
      gimbal_home_ready = 1U;
      publish_status(status);
      char line[80];
      (void)snprintf(line, sizeof(line), "GIMBAL HOME OK yaw=%u.%u pitch=%u.%u\r\n",
                     GIMBAL_INITIAL_YAW_X10 / 10U, GIMBAL_INITIAL_YAW_X10 % 10U,
                     pitch_target / 10U, pitch_target % 10U);
      debug_write(line);
      return 1U;
    }
    osDelay(GIMBAL_HOME_FEEDBACK_MS);
  }
  status->run_state = GIMBAL_STATE_HOME_FAULT;
  status->home_fault = 1U;
  status->homed = 0U;
  publish_status(status);
  debug_write("GIMBAL HOME FAULT feedback timeout; control remains gated\r\n");
  return 0U;
}

static void hold_pitch_position(GimbalStatus_t *status)
{
  uint16_t hold_x10 = status->pitch_target_x10;

  BLDC_GimbalRefreshPitchSpeed(0);
  osDelay(2U);
  if (read_pitch_feedback(status) != 0U)
  {
    hold_x10 = clamp_pitch(status->pitch_current_x10);
  }
  BLDC_GimbalArmPitchPosition(pitch_to_motor(hold_x10),
                              GIMBAL_POSITION_LIMIT_RPM);
  status->pitch_target_x10 = hold_x10;
  status->pitch_target_rpm = 0;
  status->pitch_speed_active = 0U;
  status->pitch_soft_limit_active = 0U;
}

static void apply_manual(ControlMode_t mode, int16_t yaw_rpm,
                         uint16_t yaw_angle, uint16_t pitch_angle,
                         int16_t pitch_rpm, uint8_t axis, GimbalStatus_t *status)
{
  uint8_t pitch_was_speed_active = status->pitch_speed_active;

  status->run_state = GIMBAL_STATE_MANUAL;
  status->target_valid = 0U;
  status->timed_out = 0U;
  status->pitch_speed_active = 0U;
  status->pitch_soft_limit_active = 0U;
  if (mode == CONTROL_POSE || mode == CONTROL_YAW_POSITION)
  {
    BLDC_GimbalStartYawSpeed(0);
    osDelay(GIMBAL_HOME_STOP_SETTLE_MS);
    BLDC_GimbalArmYawSingleAngle(yaw_angle, GIMBAL_POSITION_LIMIT_RPM);
    osDelay(GIMBAL_HOME_TARGET_CONFIRM_MS);
    BLDC_GimbalRefreshYawSingleAngle(yaw_angle);
    status->yaw_target_x10 = yaw_angle;
    status->yaw_target_valid = 1U;
    status->yaw_target_rpm = 0;
    if (mode == CONTROL_POSE)
    {
      BLDC_GimbalArmPitchPosition(pitch_to_motor(pitch_angle), GIMBAL_POSITION_LIMIT_RPM);
      status->pitch_target_x10 = pitch_angle;
      status->pitch_target_rpm = 0;
    }
    else if (pitch_was_speed_active != 0U)
    {
      hold_pitch_position(status);
    }
  }
  else if (mode == CONTROL_BUSINESS)
  {
    if ((axis & GIMBAL_AXIS_YAW) != 0U)
    {
      BLDC_GimbalStartYawSpeed(yaw_rpm);
      status->yaw_target_rpm = yaw_rpm;
      status->yaw_target_x10 = 0;
      status->yaw_target_valid = 0U;
    }
    else
    {
      BLDC_GimbalStartYawSpeed(0);
      status->yaw_target_rpm = 0;
      status->yaw_target_x10 = 0;
      status->yaw_target_valid = 0U;
    }
    if ((axis & GIMBAL_AXIS_PITCH) != 0U)
    {
      if (pitch_was_speed_active != 0U)
      {
        BLDC_GimbalArmPitchPosition(pitch_to_motor(pitch_angle),
                                    GIMBAL_POSITION_LIMIT_RPM);
      }
      else
      {
        BLDC_GimbalStartPitchPosition(pitch_to_motor(pitch_angle),
                                      GIMBAL_POSITION_LIMIT_RPM);
      }
      status->pitch_target_x10 = pitch_angle;
      status->pitch_target_rpm = 0;
    }
    else if (pitch_was_speed_active != 0U)
    {
      hold_pitch_position(status);
    }
  }
  else if (mode == CONTROL_PITCH_SPEED)
  {
    int16_t limited = limit_pitch_speed_by_feedback(pitch_rpm, status);
    if (status->pitch_feedback_valid != 0U)
    {
      status->pitch_target_x10 = clamp_pitch(status->pitch_current_x10);
    }
    BLDC_GimbalStartYawSpeed(0);
    BLDC_GimbalStartPitchSpeed(limited);
    status->yaw_target_rpm = 0;
    status->yaw_target_x10 = 0;
    status->yaw_target_valid = 0U;
    status->pitch_target_rpm = limited;
    status->pitch_speed_active = 1U;
  }
  publish_status(status);
}

static uint8_t next_token(const char **cursor, char *token, uint8_t size)
{
  uint8_t length = 0U;
  const char *p = *cursor;
  while (*p == ' ' || *p == '\t' || *p == ',' || *p == ';') p++;
  while (*p != '\0' && *p != '\r' && *p != '\n' && *p != ' ' && *p != '\t' && *p != ',' && *p != ';')
  {
    if (length + 1U < size) token[length++] = (*p >= 'A' && *p <= 'Z') ? (char)(*p + 32) : *p;
    p++;
  }
  token[length] = '\0';
  *cursor = p;
  return length != 0U;
}

static uint8_t parse_float(const char *text, float *value)
{
  float parsed = 0.0f;
  float fraction_scale = 0.1f;
  int32_t sign = 1;
  uint8_t has_digit = 0U;
  uint8_t fractional = 0U;

  if ((text == NULL) || (value == NULL)) return 0U;
  if (*text == '-') { sign = -1; text++; }
  else if (*text == '+') { text++; }
  while (*text != '\0')
  {
    if (*text == '.')
    {
      if (fractional != 0U) return 0U;
      fractional = 1U;
    }
    else if (*text >= '0' && *text <= '9')
    {
      has_digit = 1U;
      if (fractional == 0U) parsed = parsed * 10.0f + (float)(*text - '0');
      else
      {
        parsed += (float)(*text - '0') * fraction_scale;
        fraction_scale *= 0.1f;
      }
      if (parsed > 100000.0f) return 0U;
    }
    else return 0U;
    text++;
  }
  if (has_digit == 0U) return 0U;
  *value = parsed * (float)sign;
  return 1U;
}

static uint8_t parse_i32(const char *text, int32_t *value)
{
  float parsed;
  int32_t rounded;

  if ((parse_float(text, &parsed) == 0U) ||
      (parsed < -32768.0f) || (parsed > 32767.0f)) return 0U;
  rounded = round_i32(parsed);
  if (abs_float(parsed - (float)rounded) > 0.0001f) return 0U;
  *value = rounded;
  return 1U;
}

static uint8_t parse_angle(const char *text, uint16_t low, uint16_t high, uint16_t *value)
{
  float parsed;
  int32_t rounded;

  if ((text == NULL) || (value == NULL)) return 0U;
  if (*text == 'x' || *text == 'X')
  {
    int32_t raw = 0;
    if ((parse_i32(text + 1, &raw) == 0U) || (raw < 0)) return 0U;
    rounded = raw;
  }
  else
  {
    if (parse_float(text, &parsed) == 0U) return 0U;
    rounded = round_i32(parsed * 10.0f);
  }
  if ((rounded < low) || (rounded > high)) return 0U;
  *value = (uint16_t)rounded;
  return 1U;
}

uint8_t Gimbal_HandleControlCommandLine(const char *line, char *reply, uint16_t reply_size)
{
  const char *cursor = line;
  char command[16], first[20], second[20];
  int32_t integer;
  uint16_t angle, pitch;
  if (line == NULL || reply == NULL || reply_size == 0U || !next_token(&cursor, command, sizeof(command))) return 0U;
  if (!strcmp(command, "home") || !strcmp(command, "origin"))
  {
    Gimbal_ReturnToInitialPose();
    snprintf(reply, reply_size, "OK home requested %u.%u %u.%u\r\n", GIMBAL_INITIAL_YAW_X10 / 10U, GIMBAL_INITIAL_YAW_X10 % 10U, GIMBAL_INITIAL_PITCH_X10 / 10U, GIMBAL_INITIAL_PITCH_X10 % 10U);
    return 1U;
  }
  if (!strcmp(command, "stop")) { Gimbal_StopAndHold(); snprintf(reply, reply_size, "OK stop yaw=hold pitch=hold\r\n"); return 1U; }
  if ((!strcmp(command, "yaw") || !strcmp(command, "yawspd")) && next_token(&cursor, first, sizeof(first)) && parse_i32(first, &integer) && integer >= -GIMBAL_YAW_LIMIT_RPM && integer <= GIMBAL_YAW_LIMIT_RPM && Gimbal_SetYawSpeed((int16_t)integer)) { snprintf(reply, reply_size, "OK yaw=%ld rpm\r\n", (long)integer); return 1U; }
  if (!strcmp(command, "yawpos") && next_token(&cursor, first, sizeof(first)) && parse_angle(first, 0U, 3599U, &angle) && Gimbal_SetYawTarget(angle)) { snprintf(reply, reply_size, "OK yawpos=%u.%u\r\n", angle / 10U, angle % 10U); return 1U; }
  if ((!strcmp(command, "pitch") || !strcmp(command, "pitchang") || !strcmp(command, "pitchpos")) && next_token(&cursor, first, sizeof(first)) && parse_angle(first, GIMBAL_PITCH_MIN_X10, GIMBAL_PITCH_MAX_X10, &pitch) && Gimbal_SetPitchTarget(pitch)) { snprintf(reply, reply_size, "OK pitch=%u.%u\r\n", pitch / 10U, pitch % 10U); return 1U; }
  if ((!strcmp(command, "pitchspd") || !strcmp(command, "pitchspeed")) && next_token(&cursor, first, sizeof(first)) && parse_i32(first, &integer) && integer >= -GIMBAL_PITCH_SPEED_LIMIT_RPM && integer <= GIMBAL_PITCH_SPEED_LIMIT_RPM && Gimbal_SetPitchSpeed((int16_t)integer)) { snprintf(reply, reply_size, "OK pitch_speed=%ld rpm\r\n", (long)integer); return 1U; }
  if ((!strcmp(command, "track") || !strcmp(command, "business")) && next_token(&cursor, first, sizeof(first)) && next_token(&cursor, second, sizeof(second)) && parse_i32(first, &integer) && integer >= -GIMBAL_YAW_LIMIT_RPM && integer <= GIMBAL_YAW_LIMIT_RPM && parse_angle(second, GIMBAL_PITCH_MIN_X10, GIMBAL_PITCH_MAX_X10, &pitch) && Gimbal_SetBusinessTarget((int16_t)integer, pitch)) { snprintf(reply, reply_size, "OK track yaw=%ld pitch=%u.%u\r\n", (long)integer, pitch / 10U, pitch % 10U); return 1U; }
  if ((!strcmp(command, "pose") || !strcmp(command, "initial")) && next_token(&cursor, first, sizeof(first)) && next_token(&cursor, second, sizeof(second)) && parse_angle(first, 0U, 3599U, &angle) && parse_angle(second, GIMBAL_PITCH_MIN_X10, GIMBAL_PITCH_MAX_X10, &pitch) && Gimbal_SetInitialPose(angle, pitch)) { snprintf(reply, reply_size, "OK pose yaw=%u.%u pitch=%u.%u\r\n", angle / 10U, angle % 10U, pitch / 10U, pitch % 10U); return 1U; }
  snprintf(reply, reply_size, "ERR commands: tick home stop yaw yawpos pitch pitchspd track pose\r\n");
  return 0U;
}

void StartGimbalTask(void *argument)
{
  GimbalStatus_t status = {0};
  ControlMode_t mode = CONTROL_HOME;
  uint32_t seen_sequence = 0U;
  uint32_t last_refresh = 0U;
  uint32_t last_feedback = 0U;
  (void)argument;
  status.pitch_target_x10 = GIMBAL_INITIAL_PITCH_X10;
  status.yaw_target_x10 = GIMBAL_INITIAL_YAW_X10;
  status.yaw_target_valid = 1U;
  status.timed_out = 1U;
  status.run_state = GIMBAL_STATE_HOMING;
  publish_status(&status);
  BLDC_GimbalPowerOnWake();
  for (;;)
  {
    int16_t yaw_rpm, pitch_rpm;
    uint16_t yaw_angle, pitch_angle;
    uint8_t axis;
    uint32_t sequence;
    taskENTER_CRITICAL();
    mode = command_mode; yaw_rpm = command_yaw_rpm; yaw_angle = command_yaw_angle;
    pitch_angle = command_pitch_angle; pitch_rpm = command_pitch_rpm;
    axis = command_axis; sequence = command_sequence;
    taskEXIT_CRITICAL();
    if (sequence != seen_sequence)
    {
      seen_sequence = sequence;
      if (mode == CONTROL_HOME) (void)execute_home(&status, pitch_angle);
      else apply_manual(mode, yaw_rpm, yaw_angle, pitch_angle, pitch_rpm, axis, &status);
      last_refresh = HAL_GetTick();
    }
    uint32_t now = HAL_GetTick();
    if ((status.yaw_target_valid != 0U) && (now - last_refresh >= GIMBAL_POSITION_REFRESH_MS))
    {
      BLDC_GimbalRefreshYawSingleAngle((uint16_t)normalize_yaw(status.yaw_target_x10));
      last_refresh = now;
    }
    if (mode == CONTROL_PITCH_SPEED && now - last_feedback >= GIMBAL_PITCH_SPEED_FEEDBACK_MS)
    {
      last_feedback = now;
      int16_t limited = limit_pitch_speed_by_feedback(pitch_rpm, &status);
      if (status.pitch_feedback_valid != 0U)
      {
        status.pitch_target_x10 = clamp_pitch(status.pitch_current_x10);
      }
      if (limited != status.pitch_target_rpm)
      {
        BLDC_GimbalRefreshPitchSpeed(limited);
        status.pitch_target_rpm = limited;
      }
      publish_status(&status);
    }
    else if (mode != CONTROL_PITCH_SPEED && now - last_feedback >= GIMBAL_STATUS_FEEDBACK_MS)
    {
      last_feedback = now;
      (void)read_feedback(&status);
      publish_status(&status);
    }
    vTaskDelay(pdMS_TO_TICKS(GIMBAL_PERIOD_MS));
  }
}
