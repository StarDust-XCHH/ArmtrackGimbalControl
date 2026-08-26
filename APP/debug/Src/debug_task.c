#include "debug_task.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "gimbal_task.h"
#include "usart.h"

#define DEBUG_RX_RING_SIZE 128U
#define DEBUG_LINE_SIZE 96U
#define DEBUG_STATUS_PERIOD_MS 250U

extern osMutexId_t debugUartMutexHandle;

static uint8_t rx_byte;
static uint8_t rx_ring[DEBUG_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

static void uart_write(const char *text)
{
  uint16_t length = (uint16_t)strlen(text);
  if (debugUartMutexHandle != NULL) (void)osMutexAcquire(debugUartMutexHandle, osWaitForever);
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)text, length, 100U);
  if (debugUartMutexHandle != NULL) (void)osMutexRelease(debugUartMutexHandle);
}

static void push_rx(uint8_t value)
{
  uint16_t next = (uint16_t)((rx_head + 1U) % DEBUG_RX_RING_SIZE);
  if (next != rx_tail) { rx_ring[rx_head] = value; rx_head = next; }
}

static uint8_t pop_rx(uint8_t *value)
{
  if (rx_tail == rx_head) return 0U;
  *value = rx_ring[rx_tail];
  rx_tail = (uint16_t)((rx_tail + 1U) % DEBUG_RX_RING_SIZE);
  return 1U;
}

static uint8_t handle_tick_line(const char *line, char *response,
                                uint16_t response_size)
{
  const char *cursor;
  uint32_t sequence = 0U;

  if ((line == NULL) || (response == NULL) || (response_size == 0U) ||
      (strncmp(line, "tick ", 5U) != 0))
  {
    return 0U;
  }
  cursor = line + 5;
  if (*cursor < '0' || *cursor > '9') return 0U;
  while (*cursor >= '0' && *cursor <= '9')
  {
    uint32_t digit = (uint32_t)(*cursor - '0');
    if (sequence > ((0xFFFFFFFFUL - digit) / 10U)) return 0U;
    sequence = sequence * 10U + digit;
    cursor++;
  }
  if (*cursor != '\0') return 0U;
  (void)snprintf(response, response_size, "TOCK %lu %lu\r\n",
                 (unsigned long)sequence, (unsigned long)HAL_GetTick());
  return 1U;
}

static void format_angle(char *text, uint16_t size, int32_t angle_x10,
                         uint8_t valid)
{
  int32_t absolute;

  if (valid == 0U)
  {
    (void)snprintf(text, size, "NA");
    return;
  }
  absolute = angle_x10 < 0 ? -angle_x10 : angle_x10;
  (void)snprintf(text, size, "%s%ld.%ld",
                 angle_x10 < 0 ? "-" : "",
                 (long)(absolute / 10L), (long)(absolute % 10L));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle)
{
  if (handle == NULL || handle->Instance != USART2) return;
  push_rx(rx_byte);
  (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle)
{
  if (handle != NULL && handle->Instance == USART2)
    (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
}

static void handle_line(const char *line)
{
  char response[160];
  if (handle_tick_line(line, response, sizeof(response)) != 0U)
  {
    uart_write(response);
    return;
  }
  if (Gimbal_HandleControlCommandLine(line, response, sizeof(response)) != 0U)
  {
    uart_write(response);
    return;
  }
  uart_write("ERR commands: tick home stop yaw yawpos pitch pitchspd track pose\r\n");
}

void StartDebugTask(void *argument)
{
  char line[DEBUG_LINE_SIZE];
  uint16_t line_length = 0U;
  uint8_t value;
  uint32_t last_status = 0U;
  (void)argument;
  (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
  for (;;)
  {
    while (pop_rx(&value) != 0U)
    {
      if (value == '\r' || value == '\n')
      {
        if (line_length != 0U)
        {
          line[line_length] = '\0';
          handle_line(line);
          line_length = 0U;
        }
      }
      else if (line_length + 1U < DEBUG_LINE_SIZE)
      {
        line[line_length++] = (char)value;
      }
      else
      {
        line_length = 0U;
        uart_write("ERR line too long\r\n");
      }
    }
    if (HAL_GetTick() - last_status >= DEBUG_STATUS_PERIOD_MS)
    {
      GimbalStatus_t status;
      last_status = HAL_GetTick();
      if (Gimbal_GetStatus(&status) != 0U)
      {
        char status_line[180];
        char yaw_current[20];
        char yaw_target[20];
        char pitch_current[20];
        char pitch_target[20];
        format_angle(yaw_current, sizeof(yaw_current), status.yaw_current_x10,
                     status.yaw_feedback_valid);
        format_angle(yaw_target, sizeof(yaw_target), status.yaw_target_x10,
                     status.yaw_target_valid);
        format_angle(pitch_current, sizeof(pitch_current),
                     status.pitch_current_x10, status.pitch_feedback_valid);
        format_angle(pitch_target, sizeof(pitch_target),
                     status.pitch_target_x10, 1U);
        snprintf(status_line, sizeof(status_line),
                 "yaw_cur:%s,yaw_tgt:%s,pitch_cur:%s,pitch_tgt:%s,homed:%u,home_fault:%u,state:%u\r\n",
                 yaw_current, yaw_target, pitch_current, pitch_target,
                 status.homed, status.home_fault, status.run_state);
        uart_write(status_line);
      }
    }
    osDelay(10U);
  }
}
