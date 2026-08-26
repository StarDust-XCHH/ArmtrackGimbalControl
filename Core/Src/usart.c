#include "usart.h"

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

static void uart_common_init(UART_HandleTypeDef *handle,
                             USART_TypeDef *instance)
{
  handle->Instance = instance;
  handle->Init.BaudRate = 115200U;
  handle->Init.WordLength = UART_WORDLENGTH_8B;
  handle->Init.StopBits = UART_STOPBITS_1;
  handle->Init.Parity = UART_PARITY_NONE;
  handle->Init.Mode = UART_MODE_TX_RX;
  handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  handle->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(handle) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_USART2_UART_Init(void)
{
  uart_common_init(&huart2, USART2);
}

void MX_USART3_UART_Init(void)
{
  uart_common_init(&huart3, USART3);
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
  GPIO_InitTypeDef gpio = {0};

  if (uartHandle->Instance == USART2)
  {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = USART_TX_Pin | USART_RX_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
  else if (uartHandle->Instance == USART3)
  {
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin = F32C_RX_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(F32C_RX_GPIO_Port, &gpio);

    gpio.Pin = F32C_TX_Pin;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(F32C_TX_GPIO_Port, &gpio);

    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{
  if (uartHandle->Instance == USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, USART_TX_Pin | USART_RX_Pin);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
  else if (uartHandle->Instance == USART3)
  {
    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_GPIO_DeInit(F32C_RX_GPIO_Port, F32C_RX_Pin);
    HAL_GPIO_DeInit(F32C_TX_GPIO_Port, F32C_TX_Pin);
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  }
}
