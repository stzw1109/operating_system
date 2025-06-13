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
#include <stdio.h>
#include "wizchip_conf.h"
#include <string.h>
//#include "MQTTClient.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_TIME 100

//for W5500
#define W5500_CS_PORT GPIOC
#define W5500_CS_PIN GPIO_PIN_0
#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define HTTP_SOCKET     2
#define SOCK_TCPS       0
#define SOCK_UDPS       1
#define PORT_TCPS       5000
#define PORT_UDPS       3000
#define MAX_HTTPSOCK    6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* Definitions for updateLCD */
osThreadId_t updateLCDHandle;
const osThreadAttr_t updateLCD_attributes = {
  .name = "updateLCD",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for updateCloud */
osThreadId_t updateCloudHandle;
const osThreadAttr_t updateCloud_attributes = {
  .name = "updateCloud",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for pumpEventHandle */
osThreadId_t pumpEventHandleHandle;
const osThreadAttr_t pumpEventHandle_attributes = {
  .name = "pumpEventHandle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for pumpVolumeQueue */
osMessageQueueId_t pumpVolumeQueueHandle;
const osMessageQueueAttr_t pumpVolumeQueue_attributes = {
  .name = "pumpVolumeQueue"
};
/* Definitions for mySemaphore01 */
osSemaphoreId_t mySemaphore01Handle;
const osSemaphoreAttr_t mySemaphore01_attributes = {
  .name = "mySemaphore01"
};
/* Definitions for myBinarySem02 */
osSemaphoreId_t myBinarySem02Handle;
const osSemaphoreAttr_t myBinarySem02_attributes = {
  .name = "myBinarySem02"
};
/* USER CODE BEGIN PV */
//petrol tank volume
uint32_t petrol_tank_volume = 1000000;

//petrol pumped value for each pump
uint32_t pump1_volume = 0;
uint32_t pump2_volume = 0;
uint32_t pump3_volume = 0;
uint32_t pump4_volume = 0;

//petrol pump activation status
static bool pump1_status = false;
static bool pump2_status = false;
static bool pump3_status = false;
static bool pump4_status = false;

//petrol tank status
static bool petrol_sufficient = true;

//timing analysis
uint32_t timer_start = 0;
uint32_t timer_end = 0;

//for SPI
uint8_t txsize[8] = {2,2,2,2,2,2,2,2}; // Socket TX buffer
uint8_t rxsize[8] = {2,2,2,2,2,2,2,2}; // Socket RX buffer
volatile bool ip_assigned = false;
uint8_t dhcp_buffer[1024];
uint8_t dns_buffer[1024];
uint8_t socknumlist[] = {2, 3, 4, 5, 6, 7};
uint8_t RX_BUF[1024];
uint8_t TX_BUF[1024];

wiz_NetInfo net_info = {
    .mac  = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED },
    .dhcp = NETINFO_DHCP
};

char message[100];
char charData[200]; // Data holder
unsigned int buffer;

//MQTT stuff
/* USER CODE BEGIN PV */
//Network network;
//MQTTClient client;
unsigned char sendbuf[100], readbuf[100];
//MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;

//message queue stuff
uint16_t pump_event = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
void func_updateLCD(void *argument);
void func_updateCloud(void *argument);
void func_pumpEventHandle(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(petrol_tank_volume != 0){
		if (GPIO_Pin == pump_1_info_Pin) {
//			pump_event = 1;
//			osMessageQueuePut(pumpleHandleQueueHandle, &pump_event, 0, 0);
			pump1_status = !pump1_status;
		}
		if (GPIO_Pin == pump_2_info_Pin) {
//			pump_event = 2;
//			osMessageQueuePut(pumpleHandleQueueHandle, &pump_event, 0, 0);
			pump2_status = !pump2_status;
		}
		if (GPIO_Pin == pump_3_info_Pin) {
//			pump_event = 3;
//			osMessageQueuePut(pumpleHandleQueueHandle, &pump_event, 0, 0);
			pump3_status = !pump3_status;
		}
		if (GPIO_Pin == pump_4_info_Pin) {
//			pump_event = 4;
//			osMessageQueuePut(pumpleHandleQueueHandle, &pump_event, 0, 0);
			pump4_status = !pump4_status;
		}

	}else{
		HAL_GPIO_WritePin(GPIOA,stop_Board1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,stop_Board1_Pin,GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOA,stop_Board2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,stop_Board2_Pin,GPIO_PIN_SET);
	}
}


void wizchip_select(void) {
	HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_RESET);
}

void wizchip_deselect(void)  {
	HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_SET);
}

void wizchipWriteBurst(uint8_t* buff, uint16_t len) {
    HAL_SPI_Transmit(&hspi2, buff, len, HAL_MAX_DELAY);
}

void wizchipReadBurst(uint8_t* buff, uint16_t len) {
    HAL_SPI_Receive(&hspi2, buff, len, HAL_MAX_DELAY);
}

uint8_t wizchipReadByte(void) {
    uint8_t byte;
    wizchipReadBurst(&byte, sizeof(byte));
    return byte;
}

void wizchipWriteByte(uint8_t byte) {
    wizchipWriteBurst(&byte, sizeof(byte));
}

void Callback_IPAssigned(void) {
    ip_assigned = true;
}

void Callback_IPConflict(void) {
    ip_assigned = false;
}

void W5500Init() {
    // Register W5500 callbacks
    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
    reg_wizchip_spi_cbfunc(wizchipReadByte, wizchipWriteByte);
    reg_wizchip_spiburst_cbfunc(wizchipReadBurst, wizchipWriteBurst);

    uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
    wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);

    // set MAC address before using DHCP
    setSHAR(net_info.mac);
    DHCP_init(DHCP_SOCKET, dhcp_buffer);

    reg_dhcp_cbfunc(
        Callback_IPAssigned,
        Callback_IPAssigned,
        Callback_IPConflict
    );

 uint32_t ctr = 10000;
 while((!ip_assigned) && (ctr > 0)) {
	 DHCP_run();
	 ctr--;
}
 if(!ip_assigned) {
	 return;
}

	getIPfromDHCP(net_info.ip);
	getGWfromDHCP(net_info.gw);
	getSNfromDHCP(net_info.sn);


    buffer = sprintf(charData,"IP:  %d.%d.%d.%d\r\nGW:  %d.%d.%d.%d\r\nNet: %d.%d.%d.%d\r\n",
        net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3],
        net_info.gw[0], net_info.gw[1], net_info.gw[2], net_info.gw[3],
        net_info.sn[0], net_info.sn[1], net_info.sn[2], net_info.sn[3]
    );
    for(int i = 0; i < buffer ; i++){
    	ITM_SendChar(charData[i]);
    }

	wizchip_setnetinfo(&net_info);
}

void update_Board1(){
	if(pump1_status){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
	}
	if(pump2_status){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
	}
}

void update_Board2(){
	if(pump3_status){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
	}
	if(pump4_status){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  //w5500 section
  //W5500Init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of mySemaphore01 */
  mySemaphore01Handle = osSemaphoreNew(1, 1, &mySemaphore01_attributes);

  /* creation of myBinarySem02 */
  myBinarySem02Handle = osSemaphoreNew(1, 1, &myBinarySem02_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of pumpVolumeQueue */
  pumpVolumeQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &pumpVolumeQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  osMessageQueuePut(pumpVolumeQueueHandle,&petrol_tank_volume,0,0);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of updateLCD */
  updateLCDHandle = osThreadNew(func_updateLCD, NULL, &updateLCD_attributes);

  /* creation of updateCloud */
  updateCloudHandle = osThreadNew(func_updateCloud, NULL, &updateCloud_attributes);

  /* creation of pumpEventHandle */
  pumpEventHandleHandle = osThreadNew(func_pumpEventHandle, NULL, &pumpEventHandle_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  //pumpEventQueueHandle = osMessageQueueNew(10, sizeof(uint16_t), NULL);

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  HAL_GPIO_WritePin(SPI_Chip_Select_GPIO_Port, SPI_Chip_Select_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, pump1_volume_inc_Pin|pump2_volume_inc_Pin|stop_Board1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(stop_Board2_GPIO_Port, stop_Board2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, pump3_volume_inc_Pin|pump4_volume_inc_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SPI_Chip_Select_Pin */
  GPIO_InitStruct.Pin = SPI_Chip_Select_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_Chip_Select_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : pump1_volume_inc_Pin pump2_volume_inc_Pin stop_Board1_Pin */
  GPIO_InitStruct.Pin = pump1_volume_inc_Pin|pump2_volume_inc_Pin|stop_Board1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : stop_Board2_Pin */
  GPIO_InitStruct.Pin = stop_Board2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(stop_Board2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : pump3_volume_inc_Pin pump4_volume_inc_Pin */
  GPIO_InitStruct.Pin = pump3_volume_inc_Pin|pump4_volume_inc_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : pump_4_info_Pin pump_3_info_Pin pump_1_info_Pin pump_2_info_Pin */
  GPIO_InitStruct.Pin = pump_4_info_Pin|pump_3_info_Pin|pump_1_info_Pin|pump_2_info_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_func_updateLCD */
/**
  * @brief  Function implementing the updateLCD thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_func_updateLCD */
void func_updateLCD(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	 if(osSemaphoreAcquire(pumpSemaphoreHandle, osWaitForever) == osOK){ // Acquire semaphore
//		// Format and display messages
//		  lcd_clear();
//		  lcd_put_cur(0, 0);
//		  sprintf(message, "Petrol Fuel Volume: %lu L", petrol_tank_volume);
//		  lcd_send_string(message);
//		  osSemaphoreRelease(pumpSemaphoreHandle); // Release semaphore
//		}
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_func_updateCloud */
/**
* @brief Function implementing the updateCloud thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_updateCloud */
void func_updateCloud(void *argument)
{
  /* USER CODE BEGIN func_updateCloud */
  /* Infinite loop */
  for(;;)
  {
//	  mqttnetwork_connect(&network, "your-cluster-id.s1.eu.hivemq.cloud", 8883); // Use 8883 for TLS or 1883 for no TLS
//
//	   MQTTClientInit(&client, &network, 5000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));
//
//	   connectData.MQTTVersion = 3;
//	   connectData.clientID.cstring = "STM32Client";
//	   connectData.username.cstring = "b022210152";  // Replace with HiveMQ username
//	   connectData.password.cstring = "b022210152UTEM!";  // Replace with HiveMQ password
//
//	   if (MQTTConnect(&client, &connectData) != SUCCESS)
//	   {
//	     ITM_SendChar('F');  // connection failed
//	     vTaskDelete(NULL);
//	   }
//
//	   MQTTMessage message;
//	   message.qos = QOS0;
//	   message.retained = 0;
//	   message.payload = sendbuf;
//
//	   while (1)
//	   {
//	     snprintf((char *)sendbuf, sizeof(sendbuf), "Petrol Volume: %lu", petrol_tank_volume);
//	     message.payloadlen = strlen((char *)sendbuf);
//
//	     MQTTPublish(&client, "stm32/petrol", &message);
//
//	     osDelay(5000); // Wait 5 seconds
//	   }
  }
  /* USER CODE END func_updateCloud */
}

/* USER CODE BEGIN Header_func_pumpEventHandle */
/**
* @brief Function implementing the pumpEventHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_pumpEventHandle */
void func_pumpEventHandle(void *argument)
{
  /* USER CODE BEGIN func_pumpEventHandle */
  /* Infinite loop */
  unsigned int x;
  for(;;)
  {
	 if(petrol_tank_volume == 0){
		 sprintf(message, "Petrol Fuel Tank is Empty");
		 ITM_SendChar(message);
		 osDelay(osWaitForever);
	 }else{
		 if(pump1_status){
			 osMessageQueueGet(pumpVolumeQueueHandle,&petrol_tank_volume,0,osWaitForever);
			 petrol_tank_volume--;
			 pump1_volume++;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
			 osMessageQueuePut(pumpVolumeQueueHandle,&petrol_tank_volume,0,osWaitForever);

		 }else if(pump2_status){
			 osMessageQueueGet(pumpVolumeQueueHandle,&petrol_tank_volume,0,osWaitForever);
			 petrol_tank_volume--;
			 pump2_volume++;
			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
			 osMessageQueuePut(pumpVolumeQueueHandle,&petrol_tank_volume,0,osWaitForever);

		 }else if(pump3_status){
			 osMessageQueueGet(pumpVolumeQueueHandle,&petrol_tank_volume,0,osWaitForever);
			 petrol_tank_volume--;
			 pump3_volume++;
			 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
			 osMessageQueuePut(pumpVolumeQueueHandle,&petrol_tank_volume,0,osWaitForever);

		 }else if(pump4_status){
			 osMessageQueueGet(pumpVolumeQueueHandle,&petrol_tank_volume,0,osWaitForever);
			 petrol_tank_volume--;
			 pump4_volume++;
			 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
			 osMessageQueuePut(pumpVolumeQueueHandle,&petrol_tank_volume,0,osWaitForever);
		 }


	 }
  }
  /* USER CODE END func_pumpEventHandle */
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
