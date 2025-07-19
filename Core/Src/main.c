/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for UARTTask */
osThreadId_t UARTTaskHandle;
const osThreadAttr_t UARTTask_attributes = {
  .name = "UARTTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RedLEDTask */
osThreadId_t RedLEDTaskHandle;
const osThreadAttr_t RedLEDTask_attributes = {
  .name = "RedLEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for YellowLEDTask */
osThreadId_t YellowLEDTaskHandle;
const osThreadAttr_t YellowLEDTask_attributes = {
  .name = "YellowLEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GreenLEDTask */
osThreadId_t GreenLEDTaskHandle;
const osThreadAttr_t GreenLEDTask_attributes = {
  .name = "GreenLEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BlinkRedLEDTask */
osThreadId_t BlinkRedLEDTaskHandle;
const osThreadAttr_t BlinkRedLEDTask_attributes = {
  .name = "BlinkRedLEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCDNotification */
osThreadId_t LCDNotificationHandle;
const osThreadAttr_t LCDNotification_attributes = {
  .name = "LCDNotification",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myQueue */
osMessageQueueId_t myQueueHandle;
const osMessageQueueAttr_t myQueue_attributes = {
  .name = "myQueue"
};
/* USER CODE BEGIN PV */
uint8_t tx_data[]="Hello, are you ready to start the capture? (y/n) \r\n";
uint8_t rx_data[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartUARTtask(void *argument);
void StartRedLEDTask(void *argument);
void StartYellowLEDTask(void *argument);
void StartGreenLEDTask(void *argument);
void StartBlinkRedLEDTask(void *argument);
void StartLCDNotificationTask(void *argument);

/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct{
	uint8_t id;
}QUEUE_t;

typedef enum{
	RED_ON,
	RED_BLINK_ON,
	YELLOW_ON,
	GREEN_ON
}led_state;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue */
  myQueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &myQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UARTTask */
  UARTTaskHandle = osThreadNew(StartUARTtask, NULL, &UARTTask_attributes);

  /* creation of RedLEDTask */
  RedLEDTaskHandle = osThreadNew(StartRedLEDTask, NULL, &RedLEDTask_attributes);

  /* creation of YellowLEDTask */
  YellowLEDTaskHandle = osThreadNew(StartYellowLEDTask, NULL, &YellowLEDTask_attributes);

  /* creation of GreenLEDTask */
  GreenLEDTaskHandle = osThreadNew(StartGreenLEDTask, NULL, &GreenLEDTask_attributes);

  /* creation of BlinkRedLEDTask */
  BlinkRedLEDTaskHandle = osThreadNew(StartBlinkRedLEDTask, NULL, &BlinkRedLEDTask_attributes);

  /* creation of LCDNotification */
  LCDNotificationHandle = osThreadNew(StartLCDNotificationTask, NULL, &LCDNotification_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


//	  HAL_Delay(400);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RED_LED_Pin|DEFLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PSH_BTN_Pin */
  GPIO_InitStruct.Pin = PSH_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PSH_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_LED_Pin DEFLED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin|DEFLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : YELLOW_LED_Pin */
  GPIO_InitStruct.Pin = YELLOW_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(YELLOW_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUARTtask */
/**
  * @brief  Function implementing the UARTTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUARTtask */
void StartUARTtask(void *argument)
{
  /* USER CODE BEGIN 5 */

	uint32_t ulNotifiedValue;

  /* Infinite loop */
  for(;;)
  {

	  xTaskNotify(RedLEDTaskHandle,0,eNoAction);
	  HAL_UART_Receive(&huart2, rx_data, sizeof(rx_data), HAL_MAX_DELAY);
	  HAL_UART_Transmit(&huart2, rx_data, 1, HAL_MAX_DELAY);
	  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

	  if(rx_data[0]=='y'){

//		  HAL_UART_Transmit(&huart2, (uint8_t*)"SendingBlinkRedLEDTask\r\n", sizeof("SendingBlinkRedLEDTask\r\n"), HAL_MAX_DELAY);
		  xTaskNotify(BlinkRedLEDTaskHandle,0,eNoAction);
		  if(xTaskNotifyWait(0, 0, &ulNotifiedValue , portMAX_DELAY)==pdTRUE){
			  HAL_UART_Transmit(&huart2, (uint8_t*)"it was a success!!!\r\n", sizeof("it was a success!!!\r\n"), HAL_MAX_DELAY);
			  //clear receive buffer

			  HAL_UART_Receive(&huart2, rx_data, sizeof(rx_data), 200);
		  }

	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartRedLEDTask */
/**
* @brief Function implementing the RedLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRedLEDTask */
void StartRedLEDTask(void *argument)
{
  /* USER CODE BEGIN StartRedLEDTask */
	uint32_t ulNotifiedValue;
	uint8_t msgToSend=0;
  /* Infinite loop */
  for(;;)
  {
	  if(xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY)){
		  osMessageQueuePut(myQueueHandle, &msgToSend, 0U, 0U);
		  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
		  HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, 0);
		  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
	  }
  }
  /* USER CODE END StartRedLEDTask */
}

/* USER CODE BEGIN Header_StartYellowLEDTask */
/**
* @brief Function implementing the YellowLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartYellowLEDTask */
void StartYellowLEDTask(void *argument)
{
  /* USER CODE BEGIN StartYellowLEDTask */
	uint32_t ulNotifiedValue;
	uint8_t msgToSend=2;
  /* Infinite loop */
  for(;;)
  {
	  if(xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY)){
		  osMessageQueuePut(myQueueHandle, &msgToSend, 0U, 0U);
//		  osMessagePut(myQueueHandle, (uint32_t)"WE ARE IN YELLOW TASK!", HAL_MAX_DELAY);
		  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
		  HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, 1);
		  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
		  osDelay(5000);
		  xTaskNotify(GreenLEDTaskHandle,0,eNoAction);
	  }
    osDelay(1);
  }
  /* USER CODE END StartYellowLEDTask */
}

/* USER CODE BEGIN Header_StartGreenLEDTask */
/**
* @brief Function implementing the GreenLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGreenLEDTask */
void StartGreenLEDTask(void *argument)
{
  /* USER CODE BEGIN StartGreenLEDTask */
	uint32_t ulNotifiedValue;
	uint8_t msgToSend=3;
  /* Infinite loop */
  for(;;)
  {
	  if(xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY)){
		  osMessageQueuePut(myQueueHandle, &msgToSend, 0U, 0U);
		  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
		  HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, 0);
		  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
		  osDelay(3000);
		  HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
		  xTaskNotify(UARTTaskHandle,0,eNoAction);
	  }
  }
  /* USER CODE END StartGreenLEDTask */
}

/* USER CODE BEGIN Header_StartBlinkRedLEDTask */
/**
* @brief Function implementing the BlinkRedLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlinkRedLEDTask */
void StartBlinkRedLEDTask(void *argument)
{
  /* USER CODE BEGIN StartBlinkRedLEDTask */
	uint32_t ulNotifiedValue;
	uint8_t msgToSend=1;

  /* Infinite loop */
  for(;;)
  {
	  if(xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY)){
//	  if(ulTaskNotifyTake(pdFALSE, portMAX_DELAY)){
		  HAL_UART_Transmit(&huart2, (uint8_t*)"INBLINKLEDDDDD\r\n", sizeof("INBLINKLEDDDDD\r\n"), HAL_MAX_DELAY);
		  osMessageQueuePut(myQueueHandle, &msgToSend, 0U, 0U);
//		  osMessagePut(myQueueHandle,(uint32_t)"WE ARE IN rEDBLINKINGGGGG TASK!", HAL_MAX_DELAY);
		  int cnt=0;
		  while(cnt<6){
			  HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
			  osDelay(500);
			  cnt++;
		  }
		  osDelay(1000);

		  xTaskNotify(YellowLEDTaskHandle,0,eNoAction);
	  }
    osDelay(1);
  }
  /* USER CODE END StartBlinkRedLEDTask */
}

/* USER CODE BEGIN Header_StartLCDNotificationTask */
/**
* @brief Function implementing the LCDNotification thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCDNotificationTask */
void StartLCDNotificationTask(void *argument)
{
  /* USER CODE BEGIN StartLCDNotificationTask */
//	osEvent taskMsg;
	QUEUE_t msg;

  HD44780_Init(2);
  HD44780_Clear();
  HD44780_Backlight();

  /* Infinite loop */
  for(;;)
  {


//	  HAL_UART_Transmit(&huart2, (uint8_t*)"INLCDTAKS\r\n", sizeof("INLCDTAKS\r\n"), osWaitForever);
	  if(osMessageQueueGet(myQueueHandle, &msg, NULL, HAL_MAX_DELAY) == osOK){

		  if(msg.id==RED_ON){
//			  HD44780_Init(2);
//			  HD44780_Clear();
//			  HD44780_Backlight();
			  HD44780_SetCursor(0,0);
			  HD44780_PrintStr("Still waiting...");
			  HD44780_SetCursor(0,1);
			  HD44780_PrintStr(" :(             ");

		  }
		  else if(msg.id==RED_BLINK_ON){
//			  HD44780_Init(2);
			  HD44780_Clear();
//			  HD44780_Backlight();
			  HD44780_SetCursor(0,0);
			  HD44780_PrintStr("Ok, we will");
			  HD44780_SetCursor(0,1);
			  HD44780_PrintStr("start soon :)");

		  }
		  else if(msg.id==YELLOW_ON){
//			  HD44780_Init(2);
			  HD44780_Clear();
//			  HD44780_Backlight();
			  HD44780_SetCursor(0,0);
			  HD44780_PrintStr("Working on it!");
			  HD44780_SetCursor(0,1);
			  HD44780_PrintStr(" :D            ");

		  }
		  else if(msg.id==GREEN_ON){
//			  HD44780_Init(2);
			  HD44780_Clear();
//			  HD44780_Backlight();
			  HD44780_SetCursor(0,0);
			  HD44780_PrintStr("All done,     ");
			  HD44780_SetCursor(0,1);
			  HD44780_PrintStr("have a good day!");
		  }
		  //envoie un uart apres le if du msg.id pour verifier!!!!

	  }
//	  taskMsg=osMessageGet(myQueueHandle, HAL_MAX_DELAY);
  }
  /* USER CODE END StartLCDNotificationTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
