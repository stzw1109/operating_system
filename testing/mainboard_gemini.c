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
#include "MQTTClient.h" // Uncommented: MQTTClient library
#include "MQTTNetwork.h" // Uncommented: Your custom MQTT network layer
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_TIME 100

//for W5500
#define W5500_CS_PORT GPIOC // Assuming SPI_Chip_Select_GPIO_Port is GPIOC
#define W5500_CS_PIN GPIO_PIN_0 // Assuming SPI_Chip_Select_Pin is GPIO_PIN_0
#define DHCP_SOCKET     0
#define DNS_SOCKET      1
// #define HTTP_SOCKET     2 // Not used directly in this context
// #define SOCK_TCPS       0 // Redefined in MQTTNetwork.c for clarity
// #define SOCK_UDPS       1 // Not used in this context
// #define PORT_TCPS       5000 // Not used directly, defined locally in MQTTNetwork.c
// #define PORT_UDPS       3000 // Not used in this context
// #define MAX_HTTPSOCK    6 // Not used in this context
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
  .stack_size = 256 * 4, // Increased stack size
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for updateCloud */
osThreadId_t updateCloudHandle;
const osThreadAttr_t updateCloud_attributes = {
  .name = "updateCloud",
  .stack_size = 512 * 4, // Increased stack size for MQTT operations
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pumpEventHandle */
osThreadId_t pumpEventHandleHandle;
const osThreadAttr_t pumpEventHandle_attributes = {
  .name = "pumpEventHandle",
  .stack_size = 256 * 4, // Increased stack size to handle logic and UART printing
  .priority = (osPriority_t) osPriorityHigh, // Set higher priority as it processes critical events
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
// myBinarySem02Handle is not used in the updated logic, consider removing if not needed elsewhere
osSemaphoreId_t myBinarySem02Handle;
const osSemaphoreAttr_t myBinarySem02_attributes = {
  .name = "myBinarySem02"
};
/* USER CODE BEGIN PV */
//petrol tank volume
volatile uint32_t petrol_tank_volume = 1000000; // Make volatile

//petrol pumped value for each pump
volatile uint32_t pump1_volume = 0; // Make volatile
volatile uint32_t pump2_volume = 0; // Make volatile
volatile uint32_t pump3_volume = 0; // Make volatile
volatile uint32_t pump4_volume = 0; // Make volatile

//petrol pump activation status - These variables are no longer used for counting logic
// static bool pump1_status = false;
// static bool pump2_status = false;
// static bool pump3_status = false;
// static bool pump4_status = false;

//petrol tank status - make volatile as it's modified in pumpEventHandle and checked elsewhere
volatile bool petrol_sufficient = true;

//timing analysis (if used, make volatile if modified by ISRs/multiple tasks)
uint32_t timer_start = 0;
uint32_t timer_end = 0;

//for SPI (W5500)
uint8_t txsize[8] = {2,2,2,2,2,2,2,2}; // Socket TX buffer
uint8_t rxsize[8] = {2,2,2,2,2,2,2,2}; // Socket RX buffer
volatile bool ip_assigned = false; // Make volatile
uint8_t dhcp_buffer[1024];
uint8_t dns_buffer[1024]; // Used if DNS is implemented
uint8_t socknumlist[] = {2, 3, 4, 5, 6, 7}; // List of available sockets
uint8_t RX_BUF[1024]; // Generic RX buffer (if used by W5500 driver directly)
uint8_t TX_BUF[1024]; // Generic TX buffer (if used by W5500 driver directly)

wiz_NetInfo net_info = {
    .mac  = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED },
    .dhcp = NETINFO_DHCP
};

char message_buffer[100]; // Renamed 'message' to 'message_buffer' to avoid conflict if 'message' is MQTT struct
char charData[200]; // Data holder, used in W5500Init for debug print
unsigned int buffer; // Used for sprintf return value

//MQTT stuff - Uncommented and declared
Network network;
MQTTClient client;
unsigned char sendbuf[256], readbuf[256]; // Increased buffer size for MQTT
MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;


//message queue stuff for pump events
uint16_t pump_event_id = 0; // Renamed 'pump_event' to 'pump_event_id' for clarity
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
// W5500 SPI Chip Select functions
void wizchip_select(void);
void wizchip_deselect(void);
void wizchipWriteBurst(uint8_t* buff, uint16_t len);
void wizchipReadBurst(uint8_t* buff, uint16_t len);
uint8_t wizchipReadByte(void);
void wizchipWriteByte(uint8_t byte);

// DHCP Callbacks
void Callback_IPAssigned(void);
void Callback_IPConflict(void);

// W5500 Initialization
void W5500Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief EXTI line detection callbacks.
 * This function is called by the HAL when an external interrupt occurs.
 * It should ONLY put the pump event ID into the queue.
 * @param GPIO_Pin: Specifies the pins connected to EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    // In ISR, only put event into queue. Complex logic or GPIO writes should be in tasks.
    // The check for petrol_tank_volume == 0 and stopping boards will be done in func_pumpEventHandle.

    uint16_t current_pump_event_id = 0; // Local variable for the event ID

    if (GPIO_Pin == pump_1_info_Pin) {
        current_pump_event_id = 1;
    } else if (GPIO_Pin == pump_2_info_Pin) {
        current_pump_event_id = 2;
    } else if (GPIO_Pin == pump_3_info_Pin) {
        current_pump_event_id = 3;
    } else if (GPIO_Pin == pump_4_info_Pin) {
        current_pump_event_id = 4;
    }

    // Only put a valid event into the queue if a known pump pin triggered it.
    // Use osWaitNone (0) as timeout in ISR context to avoid blocking.
    if (current_pump_event_id != 0) {
        osMessageQueuePut(pumpVolumeQueueHandle, &current_pump_event_id, 0, 0);
    }
}


// W5500 SPI Chip Select functions
void wizchip_select(void) {
	HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_RESET); // Assert CS (active low)
}

void wizchip_deselect(void)  {
	HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_SET);   // De-assert CS
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

// DHCP Callbacks
void Callback_IPAssigned(void) {
    ip_assigned = true;
    sprintf(message_buffer, "IP Assigned!\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
}

void Callback_IPConflict(void) {
    ip_assigned = false;
    sprintf(message_buffer, "IP Conflict!\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
}

/**
 * @brief Initializes the W5500 Ethernet module and configures network settings.
 * @retval None
 */
void W5500Init() {
    // Register W5500 callbacks for SPI communication and chip select
    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
    reg_wizchip_spi_cbfunc(wizchipReadByte, wizchipWriteByte);
    reg_wizchip_spiburst_cbfunc(wizchipReadBurst, wizchipWriteBurst);

    // Set W5500 RX/TX buffer sizes for each socket (Each '2' means 4KB)
    // Correct configuration if total TX/RX sum must not exceed 16KB each.
    // For 8 sockets, {1,1,1,1,1,1,1,1} for 2KB per socket is typical.
    // If you use {2,2,2,2,2,2,2,2}, it implies 4KB per socket, totaling 32KB for TX and 32KB for RX,
    // which EXCEEDS the W5500's 16KB TX / 16KB RX memory.
    // Please verify your intended buffer sizes. Using {1,1,1,1,1,1,1,1} is safer for 8 sockets.
    uint8_t rx_tx_buff_sizes[] = {1, 1, 1, 1, 1, 1, 1, 1}; // Changed to 2KB per socket (2^1 KB)
    wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);

    // Set MAC address before using DHCP
    wiz_NetInfo net_info_local; // Use a local variable for net_info
    net_info_local.mac[0] = 0xDE;
    net_info_local.mac[1] = 0xAD;
    net_info_local.mac[2] = 0xBE;
    net_info_local.mac[3] = 0xEF;
    net_info_local.mac[4] = 0xFE;
    net_info_local.mac[5] = 0xED;
    setSHAR(net_info_local.mac);

    // Initialize DHCP client
    DHCP_init(DHCP_SOCKET, dhcp_buffer);

    // Register DHCP callback functions
    reg_dhcp_cbfunc(
        Callback_IPAssigned,
        Callback_IPAssigned, // Same callback for assigned and lease_renewed
        Callback_IPConflict
    );

    // Run DHCP until IP is assigned or timeout
    uint32_t dhcp_timeout_ctr = 10000; // Adjust as needed
    ip_assigned = false; // Reset flag
    sprintf(message_buffer, "Getting IP via DHCP...\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);

    while((!ip_assigned) && (dhcp_timeout_ctr > 0)) {
        DHCP_run();
        dhcp_timeout_ctr--;
        HAL_Delay(10); // Small delay to prevent tight loop in pre-scheduler init
    }

    if(!ip_assigned) {
        sprintf(message_buffer, "DHCP Failed! Using static IP (fallback).\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
        // Fallback to static IP if DHCP fails (example static IP)
        net_info_local.dhcp = NETINFO_STATIC;
        net_info_local.ip[0] = 192; net_info_local.ip[1] = 168; net_info_local.ip[2] = 1; net_info_local.ip[3] = 10;
        net_info_local.gw[0] = 192; net_info_local.gw[1] = 168; net_info_local.gw[2] = 1; net_info_local.gw[3] = 1;
        net_info_local.sn[0] = 255; net_info_local.sn[1] = 255; net_info_local.sn[2] = 255; net_info_local.sn[3] = 0;
    } else {
        // Get assigned IP info from DHCP
        getIPfromDHCP(net_info_local.ip);
        getGWfromDHCP(net_info_local.gw);
        getSNfromDHCP(net_info_local.sn);
        net_info_local.dhcp = NETINFO_DHCP; // Ensure status is DHCP
    }

    // Set the network info for the W5500
    wizchip_setnetinfo(&net_info_local);

    // Print network info to ITM/UART for debugging
    sprintf(charData, "IP:  %d.%d.%d.%d\r\nGW:  %d.%d.%d.%d\r\nSN:  %d.%d.%d.%d\r\n",
        net_info_local.ip[0], net_info_local.ip[1], net_info_local.ip[2], net_info_local.ip[3],
        net_info_local.gw[0], net_info_local.gw[1], net_info_local.gw[2], net_info_local.gw[3],
        net_info_local.sn[0], net_info_local.sn[1], net_info_local.sn[2], net_info_local.sn[3]
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)charData, strlen(charData), HAL_MAX_DELAY);
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
  // lcd_init(); // Uncomment if you are using an LCD
  W5500Init(); // Uncommented: W5500 initialization
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
  myBinarySem02Handle = osSemaphoreNew(1, 1, &myBinarySem02_attributes); // MyBinarySem02 is unused in new logic

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of pumpVolumeQueue */
  // Queue size 16 for uint16_t is good for pump events
  pumpVolumeQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &pumpVolumeQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  // osMessageQueuePut(pumpVolumeQueueHandle,&petrol_tank_volume,0,0); // REMOVE: Don't put tank volume in this queue
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of updateLCD */
  updateLCDHandle = osThreadNew(func_updateLCD, NULL, &updateLCD_attributes);

  /* creation of updateCloud */
  updateCloudHandle = osThreadNew(func_updateCloud, NULL, &updateCloud_attributes);

  /* creation of pumpEventHandle */
  pumpEventHandleHandle = osThreadNew(func_pumpEventHandle, NULL, &pumpEventHandle_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* Remove these as func_pumpEventHandle centralizes volume updates */
  // updatePetrol_P1Handle = osThreadNew(func_updatePetrol_pump1, NULL, &updatePetrol_P1_attributes);
  // updatePetrol_P2Handle = osThreadNew(func_updatePetrol_pump2, NULL, &updatePetrol_P2_attributes);
  // updatePetrol_P3Handle = osThreadNew(func_updatePetrol_pump3, NULL, &updatePetrol_P3_attributes);
  // updatePetrol_P4Handle = osThreadNew(func_updatePetrol_pump4, NULL, &updatePetrol_P4_attributes);
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
  HAL_GPIO_WritePin(SPI_Chip_Select_GPIO_Port, SPI_Chip_Select_Pin, GPIO_PIN_SET); // Set CS high initially

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, pump2_volume_inc_Pin|pump1_volume_inc_Pin|stop_Board1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(stop_Board2_GPIO_Port, stop_Board2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, pump3_volume_inc_Pin|pump4_volume_inc_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SPI_Chip_Select_Pin */
  GPIO_InitStruct.Pin = SPI_Chip_Select_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Changed to PULLUP for SPI CS, active low
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_Chip_Select_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : pump2_volume_inc_Pin pump1_volume_inc_Pin stop_Board1_Pin */
  GPIO_InitStruct.Pin = pump2_volume_inc_Pin|pump1_volume_inc_Pin|stop_Board1_Pin;
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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // Changed to FALLING for pump info pulses
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Pull-up to ensure HIGH when idle
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
// Placeholder for LCD update function (needs actual LCD library calls)
void func_updateLCD(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    // Acquire semaphore for consistent read if variables are updated by other tasks/ISRs.
    // 'mySemaphore01Handle' is used for volume updates.
    if(osSemaphoreAcquire(mySemaphore01Handle, osWaitForever) == osOK){
        // Format and display messages
        // lcd_clear(); // Uncomment if you have this function
        // lcd_put_cur(0, 0); // Uncomment if you have this function
        sprintf(message_buffer, "Tank: %lu L", petrol_tank_volume);
        // lcd_send_string(message_buffer); // Uncomment if you have this function

        // Example for showing individual pump volumes on a second line
        // lcd_put_cur(1, 0); // For 2nd line
        sprintf(message_buffer, "P1:%lu P2:%lu", pump1_volume, pump2_volume);
        // lcd_send_string(message_buffer); // Uncomment if you have this function

        // Consider updating a third line for P3/P4 if your LCD supports it
        // lcd_put_cur(2, 0); // For 3rd line
        // sprintf(message_buffer, "P3:%lu P4:%lu", pump3_volume, pump4_volume);
        // lcd_send_string(message_buffer);

        osSemaphoreRelease(mySemaphore01Handle); // Release semaphore
    } else {
        // Log semaphore acquisition failure, if necessary
        sprintf(message_buffer, "LCD: Failed to acquire semaphore!\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
    }
    osDelay(pdMS_TO_TICKS(500)); // Update LCD every 500ms
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
  // --- MQTT Connection and Initialization Setup for HiveMQ Cluster ---
  //
  // IMPORTANT: HiveMQ Cloud typically enforces TLS on port 8883.
  // Your current W5500 and Paho MQTT client setup DOES NOT include a TLS/SSL stack.
  // Attempting to connect to port 8883 WITHOUT TLS will FAIL at runtime.
  // To connect securely to HiveMQ Cloud, you WILL NEED to integrate a TLS library
  // (e.g., MbedTLS) into your project and modify your MQTTNetwork.c to handle TLS handshakes.
  //
  // IMPORTANT: The `mqttnetwork_connect` function (in MQTTNetwork.c) currently expects an IP address (uint8_t[4]).
  // If you want to use your HiveMQ cluster's HOSTNAME (e.g., 'your-cluster-id.s1.eu.hivemq.cloud'),
  // you MUST implement DNS resolution (using W5500's DNS client capabilities) to resolve the hostname to an IP address
  // BEFORE calling `mqttnetwork_connect`. Your current setup does not have DNS client functionality.
  //
  // For now, you MUST provide the IP address of your HiveMQ cluster.
  // You can get this by pinging your HiveMQ Cloud hostname from your PC.
  //
  // **REPLACE THIS WITH THE ACTUAL IP ADDRESS OF YOUR HIVEHQ CLUSTER.**
  // Example for a HiveMQ Cloud cluster (this is a placeholder, get your actual IP):
  uint8_t broker_ip[4] = {123, 45, 67, 89}; // Placeholder: REPLACE with your actual HiveMQ cluster IP!
  int broker_port = 8883; // As requested, using port 8883 (requires TLS on HiveMQ Cloud)

  // A placeholder for the resolved IP address. You need to implement DNS to fill this.
  // For testing WITHOUT DNS: Manually find your cluster's IP (e.g., ping broker_hostname)
  // and uncomment/set this:
  // uint8_t resolved_broker_ip[4] = {123, 45, 67, 89}; // Example: IP of your HiveMQ cluster.
  uint8_t resolved_broker_ip[4] = {0, 0, 0, 0}; // Initialize to 0, DNS should fill this

  bool connected_to_mqtt = false;

  // Connection and initialization loop
  while (!connected_to_mqtt) {
      sprintf(message_buffer, "Cloud: Connecting to network...\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);

      // --- DNS RESOLUTION (PLACEHOLDER - YOU NEED TO IMPLEMENT THIS) ---
      // If you are using a hostname, you need to implement DNS resolution here
      // using the W5500's DNS client functions (e.g., DNS_run()).
      // For now, `resolved_broker_ip` must be manually set or filled by your DNS implementation.
      // If DNS is not implemented, the `mqttnetwork_connect` will fail if `resolved_broker_ip` is {0,0,0,0}.
      // Consider setting `resolved_broker_ip` manually for initial testing if DNS is too complex for now.
      // Example for manual IP if DNS not implemented:
      // resolved_broker_ip[0] = 123; resolved_broker_ip[1] = 45; resolved_broker_ip[2] = 67; resolved_broker_ip[3] = 89;


      // Attempt network connection using the (resolved) IP address
      if (mqttnetwork_connect(&network, resolved_broker_ip, broker_port) != 0) {
          sprintf(message_buffer, "Cloud: Network connection failed. Retrying...\r\n");
          HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
          osDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds before retrying network connection
          continue; // Skip to next iteration to retry network connection
      }

      sprintf(message_buffer, "Cloud: Network connected. Initializing MQTT client...\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);

      // Initialize MQTT client with network interface and buffers
      MQTTClientInit(&client, &network, 5000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));

      // Setup MQTT connection data
      connectData.MQTTVersion = 3; // MQTT v3.1.1
      connectData.clientID.cstring = "STM32MainBoardClient"; // Ensure this is a unique client ID for your cluster
      connectData.username.cstring = "b022210152";  // Your HiveMQ username
      connectData.password.cstring = "b022210152UTEM!";  // Your HiveMQ password

      sprintf(message_buffer, "Cloud: Connecting to MQTT broker...\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);

      // Attempt MQTT connection
      // NOTE: This will fail if TLS is required by broker_port (8883) and not implemented in MQTTNetwork.c
      if (MQTTConnect(&client, &connectData) != SUCCESS) {
          sprintf(message_buffer, "Cloud: MQTT connection failed. Retrying...\r\n");
          HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
          mqttnetwork_disconnect(&network); // Disconnect network if MQTT connection fails
          osDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds before retrying MQTT connection
      } else {
          sprintf(message_buffer, "Cloud: MQTT Connected!\r\n");
          HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
          connected_to_mqtt = true; // Mark as connected, exit this loop
      }
  }

  // --- MQTT Publishing Loop (Runs once connected) ---
  /* Infinite loop for publishing */
  for(;;)
  {
    // Check if the MQTT client is still connected.
    // MQTTYield() helps update client.isconnected state by processing network traffic.
    if (!client.isconnected) {
        sprintf(message_buffer, "Cloud: MQTT disconnected. Attempting to reconnect...\r\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);

        // Attempt network reconnection
        // Requires re-running DNS resolution if using hostname
        if (mqttnetwork_connect(&network, resolved_broker_ip, broker_port) != 0) {
            osDelay(pdMS_TO_TICKS(5000)); // Wait before next retry
            continue; // Skip publishing in this iteration, retry connection in next
        }
        // Attempt MQTT reconnection
        if (MQTTConnect(&client, &connectData) != SUCCESS) {
            mqttnetwork_disconnect(&network); // Disconnect network if MQTT fails
            osDelay(pdMS_TO_TICKS(5000)); // Wait before next retry
            continue; // Skip publishing in this iteration, retry connection in next
        }
        sprintf(message_buffer, "Cloud: MQTT Reconnected!\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
    }

    // Acquire semaphore to protect shared volume data for consistent read
    // This is important as ISRs modify these variables.
    if (osSemaphoreAcquire(mySemaphore01Handle, osWaitForever) == osOK) {
        snprintf((char *)sendbuf, sizeof(sendbuf),
                 "Tank:%lu,P1:%lu,P2:%lu,P3:%lu,P4:%lu,PetrolOk:%d",
                 petrol_tank_volume, pump1_volume, pump2_volume, pump3_volume, pump4_volume, petrol_sufficient);
        osSemaphoreRelease(mySemaphore01Handle); // Release semaphore immediately after reading
    } else {
        // Handle semaphore acquisition failure, e.g., log an error
        sprintf(message_buffer, "Cloud: Failed to acquire semaphore!\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
        osDelay(pdMS_TO_TICKS(100)); // Small delay to avoid busy-waiting on semaphore
        continue; // Skip publishing in this iteration
    }


    MQTTMessage message;
    message.qos = QOS0; // Quality of Service 0 for simple fire-and-forget (no acknowledgments)
    message.retained = 0; // Message is not retained by the broker
    message.payload = sendbuf;
    message.payloadlen = strlen((char *)sendbuf);

    if (MQTTPublish(&client, "stm32/petrol_data", &message) != SUCCESS) { // Use a specific topic for your data
        sprintf(message_buffer, "Cloud: MQTT Publish Failed!\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
        // A publish failure might indicate a disconnect; the next loop iteration's check will handle it.
    } else {
        sprintf(message_buffer, "Cloud: Published: %s\r\n", (char *)sendbuf);
        HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
    }

    // This call is CRITICAL for the Paho MQTT client. It processes incoming
    // MQTT packets (like PINGRESP, CONNACK, etc.) and keeps the connection alive.
    // It also updates `client.isconnected`.
    MQTTYield(&client, 100); // Yield for 100ms to allow network activity

    osDelay(pdMS_TO_TICKS(5000)); // Publish every 5 seconds (this will be 5000ms + MQTTYield time)
  }
  /* USER CODE END func_updateCloud */
}

/* USER CODE BEGIN Header_func_pumpEventHandle */
/**
* @brief Function implementing the pumpEventHandle thread.
* This task processes pump events received from the EXTI ISR via a message queue.
* It safely updates global volume counters and controls pump stop signals.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_pumpEventHandle */
void func_pumpEventHandle(void *argument)
{
  uint16_t received_pump_event_id; // Variable to hold the pump event ID
  char debug_msg[100]; // Buffer for debug messages (increased size)

  /* Infinite loop */
  for(;;)
  {
    // Wait indefinitely to receive a message from the pumpVolumeQueueHandle.
    // This task will block here until an EXTI interrupt puts an event into the queue.
    if (osMessageQueueGet(pumpVolumeQueueHandle, &received_pump_event_id, NULL, osWaitForever) == osOK) {
        // Message received, now safely update shared global variables.
        // Acquire the semaphore to protect 'petrol_tank_volume' and 'pumpX_volume'.
        if (osSemaphoreAcquire(mySemaphore01Handle, osWaitForever) == osOK) {
            // Check if there's petrol left before decrementing the tank volume.
            if (petrol_tank_volume > 0) {
                petrol_tank_volume--; // Decrement the main tank volume

                // Increment the volume for the specific pump that triggered the event.
                switch (received_pump_event_id) {
                    case 1:
                        pump1_volume++;
                        sprintf(debug_msg, "Pump 1 event. P1:%lu, Tank:%lu\r\n", pump1_volume, petrol_tank_volume);
                        // These GPIO toggles are for the mainboard's *output* control signals to the pumps.
                        // They typically would be for controlling the physical pump motor or dispensing mechanism.
                        // If they are meant as feedback pulses, ensure the sidepump expects this.
                        HAL_GPIO_WritePin(GPIOA,pump1_volume_inc_Pin,GPIO_PIN_RESET); // Assert signal (LOW)
                        HAL_GPIO_WritePin(GPIOA,pump1_volume_inc_Pin,GPIO_PIN_SET);   // De-assert signal (HIGH)
                        break;
                    case 2:
                        pump2_volume++;
                        sprintf(debug_msg, "Pump 2 event. P2:%lu, Tank:%lu\r\n", pump2_volume, petrol_tank_volume);
                        HAL_GPIO_WritePin(GPIOA,pump2_volume_inc_Pin,GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(GPIOA,pump2_volume_inc_Pin,GPIO_PIN_SET);
                        break;
                    case 3:
                        pump3_volume++;
                        sprintf(debug_msg, "Pump 3 event. P3:%lu, Tank:%lu\r\n", pump3_volume, petrol_tank_volume);
                        HAL_GPIO_WritePin(GPIOB,pump3_volume_inc_Pin,GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(GPIOB,pump3_volume_inc_Pin,GPIO_PIN_SET);
                        break;
                    case 4:
                        pump4_volume++;
                        sprintf(debug_msg, "Pump 4 event. P4:%lu, Tank:%lu\r\n", pump4_volume, petrol_tank_volume);
                        HAL_GPIO_WritePin(GPIOB,pump4_volume_inc_Pin,GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(GPIOB,pump4_volume_inc_Pin,GPIO_PIN_SET);
                        break;
                    default:
                        sprintf(debug_msg, "Unknown pump event ID: %u\r\n", received_pump_event_id);
                        break;
                }
                HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);

                // After updating, check if the tank has now become empty.
                if (petrol_tank_volume == 0) {
                    if (petrol_sufficient) { // Only change state and assert pins once
                        petrol_sufficient = false; // Mark petrol as insufficient
                        // Assert stop signals (drive LOW) to side boards
                        HAL_GPIO_WritePin(stop_Board1_GPIO_Port, stop_Board1_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(stop_Board2_GPIO_Port, stop_Board2_Pin, GPIO_PIN_RESET);
                        sprintf(debug_msg, "Tank empty! Pumps stopped.\r\n");
                        HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
                    }
                } else {
                    // If petrol was previously insufficient but now has some (e.g., refilled manually)
                    if (!petrol_sufficient) {
                        petrol_sufficient = true; // Mark petrol as sufficient
                        // De-assert stop signals (drive HIGH) to allow pumps to start again
                        HAL_GPIO_WritePin(stop_Board1_GPIO_Port, stop_Board1_Pin, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(stop_Board2_GPIO_Port, stop_Board2_Pin, GPIO_PIN_SET);
                        sprintf(debug_msg, "Tank refilled! Pumps enabled.\r\n");
                        HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
                    }
                }
            } else {
                // Log semaphore acquisition failure, this shouldn't happen with osWaitForever unless kernel is faulty
                sprintf(debug_msg, "PumpEventHandle: Failed to acquire semaphore!\r\n");
                HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
            }
            osSemaphoreRelease(mySemaphore01Handle); // Ensure semaphore is always released
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
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Indicate error with LED toggle
      HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * line: assert_param error line source number
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
