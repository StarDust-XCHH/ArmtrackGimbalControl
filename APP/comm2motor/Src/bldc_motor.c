#include "bldc_motor.h"

#include <string.h>

#include "cmsis_os.h"
#include "usart.h"

#define BLDC_HEADER 0x7AU
#define BLDC_TAIL   0x7BU

#define BLDC_CMD_MODE    0x00U
#define BLDC_CMD_SPEED   0x01U
#define BLDC_CMD_POSITION 0x02U
#define BLDC_CMD_SINGLE_POS_LEGACY 0x03U
#define BLDC_CMD_DISABLE 0x05U
#define BLDC_CMD_ENABLE  0x06U
#define BLDC_CMD_ACC     0x07U
#define BLDC_CMD_FEEDBACK 0x0EU

#define BLDC_TX_TIMEOUT_MS 20U
#define BLDC_FEEDBACK_VALUE_FIRST_BYTE_TIMEOUT_MS 12U
#define BLDC_FEEDBACK_VALUE_NEXT_BYTE_TIMEOUT_MS  5U
#define BLDC_FEEDBACK_FRAME_LEN 9U
#define BLDC_CMD_GAP_MS     2U
#define BLDC_POWER_ON_DELAY_MS 1500U
#define BLDC_STARTUP_RETRY_COUNT 3U
#define BLDC_STARTUP_RETRY_DELAY_MS 100U
#define BLDC_POSITION_LEGACY_FALLBACK_ENABLE 0U

extern osMutexId_t bldcUartMutexHandle;

static uint32_t bldc_last_uart_error = HAL_UART_ERROR_NONE;
static volatile uint8_t bldc_startup_attempt_count;
static volatile uint8_t bldc_startup_yaw_feedback_ok;
static volatile uint8_t bldc_startup_pitch_feedback_ok;

static void bldc_flush_uart_rx(void)
{
  uint32_t ignored;

  __HAL_UART_CLEAR_OREFLAG(&huart3);

  while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET)
  {
    ignored = huart3.Instance->DR;
    (void)ignored;
  }
}

static uint8_t bldc_calc_bcc(const uint8_t *data, uint8_t len)
{
  uint8_t bcc = 0U;

  for (uint8_t i = 0U; i < len; i++)
  {
    bcc ^= data[i];
  }

  return bcc;
}

static uint8_t bldc_build_cmd(uint8_t addr,
                              uint8_t cmd,
                              const uint8_t *data,
                              uint8_t len,
                              uint8_t *tx_buf,
                              uint8_t tx_buf_size)
{
  uint8_t idx = 0U;

  if ((tx_buf == NULL) || (tx_buf_size < (uint8_t)(len + 5U)))
  {
    return 0U;
  }

  tx_buf[idx++] = BLDC_HEADER;
  tx_buf[idx++] = addr;
  tx_buf[idx++] = cmd;

  if ((data != NULL) && (len > 0U))
  {
    memcpy(&tx_buf[idx], data, len);
    idx = (uint8_t)(idx + len);
  }

  tx_buf[idx] = bldc_calc_bcc(tx_buf, idx);
  idx++;
  tx_buf[idx++] = BLDC_TAIL;

  return idx;
}

static void bldc_send_array(const uint8_t *data, uint8_t len)
{
  HAL_StatusTypeDef result;

  if (bldcUartMutexHandle != NULL)
  {
    (void)osMutexAcquire(bldcUartMutexHandle, osWaitForever);
  }

  result = HAL_UART_Transmit(&huart3, (uint8_t *)data, len, BLDC_TX_TIMEOUT_MS);
  if (result != HAL_OK)
  {
    bldc_last_uart_error = HAL_UART_GetError(&huart3);
    if (bldc_last_uart_error == HAL_UART_ERROR_NONE)
    {
      bldc_last_uart_error = HAL_UART_ERROR_DMA;
    }
  }
  else
  {
    bldc_last_uart_error = HAL_UART_ERROR_NONE;
  }

  if (bldcUartMutexHandle != NULL)
  {
    (void)osMutexRelease(bldcUartMutexHandle);
  }
}

static void bldc_send_legacy_single_angle(uint8_t addr, uint16_t angle_x10)
{
  uint8_t data[2];

  if (angle_x10 > 3599U)
  {
    angle_x10 = 3599U;
  }

  data[0] = (uint8_t)((angle_x10 >> 8) & 0xFFU);
  data[1] = (uint8_t)(angle_x10 & 0xFFU);
  BLDC_SendCmd(addr, BLDC_CMD_SINGLE_POS_LEGACY, data, sizeof(data));
}

static uint8_t bldc_request_feedback_frame(uint8_t addr,
                                           uint8_t type,
                                           uint8_t *rx_buf,
                                           uint8_t rx_buf_size,
                                           uint32_t first_byte_timeout_ms,
                                           uint32_t next_byte_timeout_ms)
{
  uint8_t tx_buf[6];
  uint8_t data[1];
  uint8_t tx_len;
  uint8_t rx_len = 0U;

  if ((rx_buf == NULL) || (rx_buf_size == 0U))
  {
    return 0U;
  }

  data[0] = type;
  tx_len = bldc_build_cmd(addr,
                          BLDC_CMD_FEEDBACK,
                          data,
                          sizeof(data),
                          tx_buf,
                          sizeof(tx_buf));
  if (tx_len == 0U)
  {
    return 0U;
  }

  bldc_last_uart_error = HAL_UART_ERROR_NONE;
  bldc_flush_uart_rx();

  if (bldcUartMutexHandle != NULL)
  {
    (void)osMutexAcquire(bldcUartMutexHandle, osWaitForever);
  }

  if (HAL_UART_Transmit(&huart3, tx_buf, tx_len, BLDC_TX_TIMEOUT_MS) == HAL_OK)
  {
    uint32_t wait_start = HAL_GetTick();

    while (rx_len < rx_buf_size)
    {
      if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET)
      {
        rx_buf[rx_len] = (uint8_t)(huart3.Instance->DR & 0xFFU);
        rx_len++;
        wait_start = HAL_GetTick();

        if ((rx_buf_size != BLDC_FEEDBACK_FRAME_LEN) &&
            (rx_len >= 5U) &&
            (rx_buf[rx_len - 1U] == BLDC_TAIL))
        {
          break;
        }
      }
      else
      {
        uint32_t timeout = (rx_len == 0U) ? first_byte_timeout_ms : next_byte_timeout_ms;

        if ((HAL_GetTick() - wait_start) >= timeout)
        {
          break;
        }
      }
    }
  }
  else
  {
    bldc_last_uart_error = HAL_UART_GetError(&huart3);
  }

  if (HAL_UART_GetError(&huart3) != HAL_UART_ERROR_NONE)
  {
    bldc_last_uart_error = HAL_UART_GetError(&huart3);
  }

  if (bldcUartMutexHandle != NULL)
  {
    (void)osMutexRelease(bldcUartMutexHandle);
  }

  return rx_len;
}

void BLDC_SendCmd(uint8_t addr, uint8_t cmd, const uint8_t *data, uint8_t len)
{
  uint8_t tx_buf[20];
  uint8_t tx_len;

  if (len > 14U)
  {
    return;
  }

  tx_len = bldc_build_cmd(addr, cmd, data, len, tx_buf, sizeof(tx_buf));
  if (tx_len > 0U)
  {
    bldc_send_array(tx_buf, tx_len);
  }
}

void BLDC_Enable(uint8_t addr)
{
  BLDC_SendCmd(addr, BLDC_CMD_ENABLE, NULL, 0U);
}

void BLDC_Disable(uint8_t addr)
{
  BLDC_SendCmd(addr, BLDC_CMD_DISABLE, NULL, 0U);
}

void BLDC_SetMode(uint8_t addr, uint16_t mode)
{
  uint8_t data[2];

  data[0] = (uint8_t)((mode >> 8) & 0xFFU);
  data[1] = (uint8_t)(mode & 0xFFU);
  BLDC_SendCmd(addr, BLDC_CMD_MODE, data, sizeof(data));
}

void BLDC_SetSpeed(uint8_t addr, int16_t rpm)
{
  uint8_t data[2];
  uint16_t raw = (uint16_t)rpm;

  data[0] = (uint8_t)((raw >> 8) & 0xFFU);
  data[1] = (uint8_t)(raw & 0xFFU);
  BLDC_SendCmd(addr, BLDC_CMD_SPEED, data, sizeof(data));
}

void BLDC_SetPosition(uint8_t addr, int32_t angle_x10)
{
  uint8_t data[4];
  uint32_t raw = (uint32_t)angle_x10;

  data[0] = (uint8_t)((raw >> 24) & 0xFFU);
  data[1] = (uint8_t)((raw >> 16) & 0xFFU);
  data[2] = (uint8_t)((raw >> 8) & 0xFFU);
  data[3] = (uint8_t)(raw & 0xFFU);
  BLDC_SendCmd(addr, BLDC_CMD_POSITION, data, sizeof(data));

#if BLDC_POSITION_LEGACY_FALLBACK_ENABLE
  if ((angle_x10 >= 0) && (angle_x10 <= 3600L))
  {
    osDelay(BLDC_CMD_GAP_MS);
    bldc_send_legacy_single_angle(addr, (uint16_t)angle_x10);
  }
#endif
}

void BLDC_SetSingleAngle(uint8_t addr, uint16_t angle_x10)
{
  if (angle_x10 > 3599U)
  {
    angle_x10 = 3599U;
  }

  bldc_send_legacy_single_angle(addr, angle_x10);
}

void BLDC_SetAcc(uint8_t addr, uint16_t acc)
{
  uint8_t data[2];

  data[0] = (uint8_t)((acc >> 8) & 0xFFU);
  data[1] = (uint8_t)(acc & 0xFFU);
  BLDC_SendCmd(addr, BLDC_CMD_ACC, data, sizeof(data));
}

uint8_t BLDC_RequestFeedbackValue(uint8_t addr, uint8_t type, int32_t *value)
{
  uint8_t rx_buf[BLDC_FEEDBACK_FRAME_LEN];
  uint8_t rx_len;
  uint8_t calc_bcc;

  if (value == NULL)
  {
    return 0U;
  }

  rx_len = bldc_request_feedback_frame(addr,
                                       type,
                                       rx_buf,
                                       sizeof(rx_buf),
                                       BLDC_FEEDBACK_VALUE_FIRST_BYTE_TIMEOUT_MS,
                                       BLDC_FEEDBACK_VALUE_NEXT_BYTE_TIMEOUT_MS);
  if (rx_len != BLDC_FEEDBACK_FRAME_LEN)
  {
    if (bldc_last_uart_error == HAL_UART_ERROR_NONE)
    {
      bldc_last_uart_error = BLDC_UART_ERROR_FEEDBACK_TIMEOUT;
    }
    return 0U;
  }

  if ((rx_buf[0] != BLDC_HEADER) ||
      (rx_buf[1] != addr) ||
      (rx_buf[2] != type) ||
      (rx_buf[8] != BLDC_TAIL))
  {
    if (bldc_last_uart_error == HAL_UART_ERROR_NONE)
    {
      bldc_last_uart_error = BLDC_UART_ERROR_FEEDBACK_TIMEOUT;
    }
    return 0U;
  }

  calc_bcc = bldc_calc_bcc(rx_buf, 7U);
  if (calc_bcc != rx_buf[7])
  {
    if (bldc_last_uart_error == HAL_UART_ERROR_NONE)
    {
      bldc_last_uart_error = BLDC_UART_ERROR_FEEDBACK_TIMEOUT;
    }
    return 0U;
  }

  *value = (int32_t)(((uint32_t)rx_buf[3] << 24) |
                     ((uint32_t)rx_buf[4] << 16) |
                     ((uint32_t)rx_buf[5] << 8) |
                     (uint32_t)rx_buf[6]);

  return 1U;
}

uint32_t BLDC_GetLastUartError(void)
{
  return bldc_last_uart_error;
}

uint8_t BLDC_GetStartupAttemptCount(void)
{
  return bldc_startup_attempt_count;
}

uint8_t BLDC_GetStartupFeedbackOk(uint8_t addr)
{
  if (addr == BLDC_ADDR_YAW)
  {
    return bldc_startup_yaw_feedback_ok;
  }

  if (addr == BLDC_ADDR_PITCH)
  {
    return bldc_startup_pitch_feedback_ok;
  }

  return 0U;
}

static void BLDC_StartAxisSpeed(uint8_t addr, int16_t rpm)
{
  /* The F32C startup sequence is enable -> mode -> speed.  Sending a
   * disable first leaves the controller in the wrong state on some
   * firmware revisions. */
  BLDC_Enable(addr);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetMode(addr, BLDC_MODE_SPEED);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetSpeed(addr, rpm);
  osDelay(BLDC_CMD_GAP_MS);
}

static void BLDC_RefreshAxisSpeed(uint8_t addr, int16_t rpm)
{
  BLDC_SetSpeed(addr, rpm);
}

static void BLDC_StartAxisPosition(uint8_t addr, int32_t angle_x10, int16_t limit_rpm)
{
  /* Match the vendor example: enable before selecting the mode and sending
   * the speed/position commands. */
  BLDC_Enable(addr);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetMode(addr,
               (addr == BLDC_ADDR_PITCH) ? BLDC_MODE_SINGLE_POS_T : BLDC_MODE_MULTI_POS_T);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetSpeed(addr, limit_rpm);
  osDelay(BLDC_CMD_GAP_MS);
  if (addr == BLDC_ADDR_PITCH)
  {
    if (angle_x10 < 0)
    {
      angle_x10 = 0;
    }
    if (angle_x10 > 3599L)
    {
      angle_x10 = 3599L;
    }
    BLDC_SetSingleAngle(addr, (uint16_t)angle_x10);
  }
  else
  {
    BLDC_SetPosition(addr, angle_x10);
  }
  osDelay(BLDC_CMD_GAP_MS);
}

static void BLDC_StartAxisSingleAngle(uint8_t addr, uint16_t angle_x10, int16_t limit_rpm)
{
  if (angle_x10 > 3599U)
  {
    angle_x10 = 3599U;
  }

  BLDC_Enable(addr);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetMode(addr, BLDC_MODE_SINGLE_POS_T);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetSpeed(addr, limit_rpm);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetSingleAngle(addr, angle_x10);
  osDelay(BLDC_CMD_GAP_MS);
}

static void BLDC_ArmAxisSingleAngle(uint8_t addr,
                                    uint16_t angle_x10,
                                    int16_t limit_rpm)
{
  if (angle_x10 > 3599U)
  {
    angle_x10 = 3599U;
  }

  /* Configure position control while output is disabled.  On F32C, changing
   * an enabled axis from speed to position can activate the previously
   * latched position before the following speed/angle frames take effect. */
  BLDC_Disable(addr);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetMode(addr, BLDC_MODE_SINGLE_POS_L);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetSpeed(addr, limit_rpm);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetSingleAngle(addr, angle_x10);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_Enable(addr);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetSingleAngle(addr, angle_x10);
  osDelay(BLDC_CMD_GAP_MS);
}

static void BLDC_ArmAxisTimedSingleAngle(uint8_t addr,
                                         uint16_t angle_x10,
                                         int16_t limit_rpm)
{
  if (angle_x10 > 3599U)
  {
    angle_x10 = 3599U;
  }

  /* Pitch uses timed single-position mode.  Preload every field while
   * disabled so enabling after an MCU reset cannot execute a retained target
   * from the motor controller. */
  BLDC_Disable(addr);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetMode(addr, BLDC_MODE_SINGLE_POS_T);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetSpeed(addr, limit_rpm);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetSingleAngle(addr, angle_x10);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_Enable(addr);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetSingleAngle(addr, angle_x10);
  osDelay(BLDC_CMD_GAP_MS);
}

static void BLDC_RefreshAxisPosition(uint8_t addr, int32_t angle_x10)
{
  if (addr == BLDC_ADDR_PITCH)
  {
    if (angle_x10 < 0)
    {
      angle_x10 = 0;
    }
    if (angle_x10 > 3599L)
    {
      angle_x10 = 3599L;
    }
    BLDC_SetSingleAngle(addr, (uint16_t)angle_x10);
    return;
  }

  BLDC_SetPosition(addr, angle_x10);
}

static void bldc_configure_tracking_once(uint16_t pitch_angle_x10,
                                          int16_t pitch_limit_rpm)
{
  BLDC_Enable(BLDC_ADDR_YAW);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_Enable(BLDC_ADDR_PITCH);
  osDelay(BLDC_CMD_GAP_MS);

  BLDC_SetMode(BLDC_ADDR_YAW, BLDC_MODE_SPEED);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetMode(BLDC_ADDR_PITCH, BLDC_MODE_SINGLE_POS_T);
  osDelay(BLDC_CMD_GAP_MS);

  BLDC_SetSpeed(BLDC_ADDR_YAW, 0);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_SetSpeed(BLDC_ADDR_PITCH, pitch_limit_rpm);
  osDelay(BLDC_CMD_GAP_MS);

  BLDC_SetSingleAngle(BLDC_ADDR_PITCH, pitch_angle_x10);
  osDelay(BLDC_CMD_GAP_MS);
}

void BLDC_GimbalPowerOnWake(void)
{
  uint8_t dummy = 0U;

  /* The F32C examples send one wake byte after reset.  Keep this electrical
   * startup requirement separate from any gimbal pose or control mode. */
  bldc_send_array(&dummy, 1U);
  osDelay(BLDC_POWER_ON_DELAY_MS);
}

void BLDC_GimbalSelectTrackingMode(uint16_t pitch_angle_x10,
                                   int16_t pitch_limit_rpm)
{
  bldc_configure_tracking_once(pitch_angle_x10, pitch_limit_rpm);
}

void BLDC_GimbalStartTracking(uint16_t pitch_angle_x10, int16_t pitch_limit_rpm)
{
  uint8_t attempt;
  int32_t feedback_value;

  BLDC_GimbalPowerOnWake();

  bldc_startup_attempt_count = 0U;
  bldc_startup_yaw_feedback_ok = 0U;
  bldc_startup_pitch_feedback_ok = 0U;

  for (attempt = 0U; attempt < BLDC_STARTUP_RETRY_COUNT; attempt++)
  {
    bldc_startup_attempt_count = (uint8_t)(attempt + 1U);

    /* Follow the vendor order on every attempt.  Feedback proves that the
     * selected USART pins reach both devices; when RX is unavailable the
     * bounded retries still make the TX startup tolerant of a late motor. */
    bldc_configure_tracking_once(pitch_angle_x10, pitch_limit_rpm);

    bldc_startup_yaw_feedback_ok =
        BLDC_RequestFeedbackValue(BLDC_ADDR_YAW,
                                  BLDC_FEEDBACK_MULTI_ANGLE,
                                  &feedback_value);
    osDelay(BLDC_CMD_GAP_MS);
    bldc_startup_pitch_feedback_ok =
        BLDC_RequestFeedbackValue(BLDC_ADDR_PITCH,
                                  BLDC_FEEDBACK_MULTI_ANGLE,
                                  &feedback_value);

    if ((bldc_startup_yaw_feedback_ok != 0U) &&
        (bldc_startup_pitch_feedback_ok != 0U))
    {
      break;
    }

    if ((attempt + 1U) < BLDC_STARTUP_RETRY_COUNT)
    {
      osDelay(BLDC_STARTUP_RETRY_DELAY_MS);
    }
  }
}

void BLDC_GimbalRefreshTracking(int16_t yaw_rpm, uint16_t pitch_angle_x10)
{
  BLDC_RefreshAxisSpeed(BLDC_ADDR_YAW, yaw_rpm);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_RefreshAxisPosition(BLDC_ADDR_PITCH, (int32_t)pitch_angle_x10);
}

void BLDC_GimbalRefreshSpeedTracking(int16_t yaw_rpm, int16_t pitch_rpm)
{
  BLDC_RefreshAxisSpeed(BLDC_ADDR_YAW, yaw_rpm);
  osDelay(BLDC_CMD_GAP_MS);
  BLDC_RefreshAxisSpeed(BLDC_ADDR_PITCH, pitch_rpm);
}

void BLDC_GimbalStartYawSpeed(int16_t yaw_rpm)
{
  BLDC_StartAxisSpeed(BLDC_ADDR_YAW, yaw_rpm);
}

void BLDC_GimbalRefreshYawSpeed(int16_t yaw_rpm)
{
  BLDC_RefreshAxisSpeed(BLDC_ADDR_YAW, yaw_rpm);
}

void BLDC_GimbalStartYawPosition(int32_t yaw_angle_x10, int16_t yaw_limit_rpm)
{
  BLDC_StartAxisPosition(BLDC_ADDR_YAW, yaw_angle_x10, yaw_limit_rpm);
}

void BLDC_GimbalStartYawSingleAngle(uint16_t yaw_angle_x10, int16_t yaw_limit_rpm)
{
  BLDC_StartAxisSingleAngle(BLDC_ADDR_YAW, yaw_angle_x10, yaw_limit_rpm);
}

void BLDC_GimbalArmYawSingleAngle(uint16_t yaw_angle_x10, int16_t yaw_limit_rpm)
{
  BLDC_ArmAxisSingleAngle(BLDC_ADDR_YAW, yaw_angle_x10, yaw_limit_rpm);
}

void BLDC_GimbalRefreshYawSingleAngle(uint16_t yaw_angle_x10)
{
  BLDC_SetSingleAngle(BLDC_ADDR_YAW, yaw_angle_x10);
}

void BLDC_GimbalStartPitchPosition(uint16_t pitch_angle_x10, int16_t pitch_limit_rpm)
{
  BLDC_StartAxisPosition(BLDC_ADDR_PITCH, (int32_t)pitch_angle_x10, pitch_limit_rpm);
}

void BLDC_GimbalArmPitchPosition(uint16_t pitch_angle_x10,
                                 int16_t pitch_limit_rpm)
{
  BLDC_ArmAxisTimedSingleAngle(BLDC_ADDR_PITCH,
                               pitch_angle_x10,
                               pitch_limit_rpm);
}

void BLDC_GimbalStartPitchSpeed(int16_t pitch_rpm)
{
  BLDC_StartAxisSpeed(BLDC_ADDR_PITCH, pitch_rpm);
}

void BLDC_GimbalRefreshPitchSpeed(int16_t pitch_rpm)
{
  BLDC_RefreshAxisSpeed(BLDC_ADDR_PITCH, pitch_rpm);
}
