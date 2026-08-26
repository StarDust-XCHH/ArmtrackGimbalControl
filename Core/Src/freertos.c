#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128U * 4U,
  .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t GimbalTaskHandle;
const osThreadAttr_t GimbalTask_attributes = {
  .name = "GimbalTask",
  .stack_size = 1024U * 4U,
  .priority = (osPriority_t)osPriorityHigh,
};

osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = {
  .name = "DebugTask",
  .stack_size = 512U * 4U,
  .priority = (osPriority_t)osPriorityLow,
};

osMutexId_t bldcUartMutexHandle;
const osMutexAttr_t bldcUartMutex_attributes = { .name = "bldcUartMutex" };

osMutexId_t debugUartMutexHandle;
const osMutexAttr_t debugUartMutex_attributes = { .name = "debugUartMutex" };

extern void StartGimbalTask(void *argument);
extern void StartDebugTask(void *argument);

static void StartDefaultTask(void *argument)
{
  (void)argument;
  for (;;)
  {
    osDelay(1U);
  }
}

void MX_FREERTOS_Init(void)
{
  bldcUartMutexHandle = osMutexNew(&bldcUartMutex_attributes);
  debugUartMutexHandle = osMutexNew(&debugUartMutex_attributes);

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
                                  &defaultTask_attributes);
  GimbalTaskHandle = osThreadNew(StartGimbalTask, NULL,
                                 &GimbalTask_attributes);
  DebugTaskHandle = osThreadNew(StartDebugTask, NULL,
                                &DebugTask_attributes);
}
