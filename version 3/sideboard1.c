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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "i2c-lcd.h"
#include "stdio.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_TIME 150
#define PULSE_DELAY_MS 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for pump1 */
osThreadId_t pump1Handle;
const osThreadAttr_t pump1_attributes = {
  .name = "pump1",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pump2 */
osThreadId_t pump2Handle;
const osThreadAttr_t pump2_attributes = {
  .name = "pump2",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCD */
osThreadId_t LCDHandle;
const osThreadAttr_t LCD_attributes = {
  .name = "LCD",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for pumpSemaphore */
osSemaphoreId_t pumpSemaphoreHandle;
const osSemaphoreAttr_t pumpSemaphore_attributes = {
  .name = "pumpSemaphore"
};
/* USER CODE BEGIN PV */
//petrol pumped for each pumper
volatile uint32_t pump1_volume = 0;
volatile uint32_t pump2_volume = 0;

//activation status of each pump
volatile static bool pump1_status = false;
volatile static bool pump2_status = false;

//timing analysis
uint32_t timer_start = 0;
uint32_t timer_end = 0;
uint32_t load_value = 0;

//debouncing time
TickType_t lastDebounceTime = 0;

//lcd variables
char message[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void func_pump1(void *argument);
void func_pump2(void *argument);
void updateLCD(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	TickType_t currentTime = xTaskGetTickCount();


		if(GPIO_Pin == stop_board1_Pin ){
			pump1_status = false;
			pump2_status = false;
		}

		if(GPIO_Pin == start_stop_pump1_button_Pin){

			if((currentTime - lastDebounceTime)> pdMS_TO_TICKS(DEBOUNCE_TIME)){
				lastDebounceTime = currentTime;
				pump1_status = !pump1_status;
				if (HAL_GPIO_ReadPin(stop_board1_GPIO_Port, stop_board1_Pin) == GPIO_PIN_RESET) {
					pump1_status = false; // Force stop if mainboard indicates empty
				}
			}
		}else if(GPIO_Pin == start_stop_pump2_button_Pin){
			if((currentTime - lastDebounceTime)> pdMS_TO_TICKS(DEBOUNCE_TIME)){
				lastDebounceTime = currentTime;
				pump2_status = !pump2_status;
				if (HAL_GPIO_ReadPin(stop_board1_GPIO_Port, stop_board1_Pin) == GPIO_PIN_RESET) {
				pump2_status = false; // Force stop if mainboard indicates empty
			}
		}
	}

	//}
}

void scan_i2c_devices(void) {
    HAL_StatusTypeDef result;
    uint8_t i;
    char buffer[64];
    unsigned int x;
    for (i = 1; i < 128; i++) {
        /*
         * HAL_I2C_IsDeviceReady returns:
         * HAL_OK if ACK received
         * HAL_ERROR or HAL_BUSY otherwise
         */
        result = HAL_I2C_IsDeviceReady(&hi2c1, (i << 1), 1, 10);
        if (result == HAL_OK) {
           x = sprintf(buffer, "I2C device found at address 0x%X\r\n", i);
            for(int i = 0; i < x ; i++){
            	ITM_SendChar(buffer[i]);
            }
        } else {
            continue;
        }

    }
}


void CPU_Burst(uint32_t delay_ms){
	for (int x = 0 ; x < delay_ms; x++){
		__NOP();
	}
}
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
  lcd_init();
  scan_i2c_devices();
  lcd_put_cur(0, 0);
  lcd_send_string("Init OK");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of pumpSemaphore */
  pumpSemaphoreHandle = osSemaphoreNew(1, 1, &pumpSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of pump1 */
  pump1Handle = osThreadNew(func_pump1, NULL, &pump1_attributes);

  /* creation of pump2 */
  pump2Handle = osThreadNew(func_pump2, NULL, &pump2_attributes);

  /* creation of LCD */
  LCDHandle = osThreadNew(updateLCD, NULL, &LCD_attributes);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOC, pump2_petrol_signal_Pin|pump1_petrol_signal_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : stop_board1_Pin */
  GPIO_InitStruct.Pin = stop_board1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(stop_board1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : pump1_signal_inc_Pin pump2_signal_inc_Pin */
  GPIO_InitStruct.Pin = pump1_signal_inc_Pin|pump2_signal_inc_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : pump2_petrol_signal_Pin pump1_petrol_signal_Pin */
  GPIO_InitStruct.Pin = pump2_petrol_signal_Pin|pump1_petrol_signal_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : start_stop_pump1_button_Pin start_stop_pump2_button_Pin */
  GPIO_InitStruct.Pin = start_stop_pump1_button_Pin|start_stop_pump2_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_func_pump1 */
/**
  * @brief  Function implementing the pump1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_func_pump1 */
void func_pump1(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if(pump1_status){
		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);

		  pump1_volume++;
		  //osDelay(2);
		  CPU_Burst(189);
    }else{
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_func_pump2 */
/**
* @brief Function implementing the pump2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_pump2 */
void func_pump2(void *argument)
{
  /* USER CODE BEGIN func_pump2 */
  /* Infinite loop */
  for(;;)
  {
	  if(pump2_status){
		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);   // Pulse HIGH (idle state)

			   pump2_volume++;
			   //osDelay(2);
			   CPU_Burst(189);
	   } else {
		   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
	   }
  }
  /* USER CODE END func_pump2 */
}

/* USER CODE BEGIN Header_updateLCD */
/**
* @brief Function implementing the LCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_updateLCD */
void updateLCD(void *argument)
{
  /* USER CODE BEGIN updateLCD */
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreAcquire(pumpSemaphoreHandle, osWaitForever) == osOK){ // Acquire semaphore
	        // Format and display messages
		  	  lcd_clear();
		      lcd_put_cur(0, 0);
		      sprintf(message, "Pump1: %lu L", pump1_volume);
		      lcd_send_string(message);

		      lcd_put_cur(1, 0);
		      sprintf(message, "Pump2: %lu L", pump2_volume);
		      lcd_send_string(message);
		      osDelay(1);
	        osSemaphoreRelease(pumpSemaphoreHandle); // Release semaphore
	    }
  }
  /* USER CODE END updateLCD */
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

#ifdef  USE_FULL_ASSERT
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
