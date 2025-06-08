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
#include <stdio.h> // For sprintf
#include "wizchip_conf.h" // For W5500 functions
#include "MQTTClient.h"   // Paho MQTT Client library
#include "MQTTNetwork.h"  // Your custom MQTT network layer
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_TIME 100

// For W5500 Chip Select
#define W5500_CS_PORT SPI_Chip_Select_GPIO_Port // Assuming this maps to GPIOC
#define W5500_CS_PIN  SPI_Chip_Select_Pin       // Assuming this maps to GPIO_PIN_0

// W5500 DHCP/DNS/Socket definitions (from wizchip_conf.h/socket.h)
#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define HTTP_SOCKET     2 // Not explicitly used but good to keep if needed
#define SOCK_TCPS       0 // Socket type for TCP (W5500's socket index 0)
#define MQTT_LOCAL_PORT 5000 // Local port for MQTT client socket

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
  .stack_size = 256 * 4, // Increased stack size, especially if using printf or I2C LCD
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for updateCloud */
osThreadId_t updateCloudHandle;
const osThreadAttr_t updateCloud_attributes = {
  .name = "updateCloud",
  .stack_size = 512 * 4, // Increased stack size for MQTT operations (network, buffers)
  .priority = (osPriority_t) osPriorityNormal, // Higher priority to ensure network operations
};
/* Definitions for updatePetrol */
osThreadId_t updatePetrolHandle;
const osThreadAttr_t updatePetrol_attributes = {
  .name = "updatePetrol",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mySemaphore01 */
osSemaphoreId_t mySemaphore01Handle;
const osSemaphoreAttr_t mySemaphore01_attributes = {
  .name = "mySemaphore01"
};
/* USER CODE BEGIN PV */
// Declare shared variables as volatile
volatile uint32_t petrol_tank_volume = 1000000;

// petrol pumped value for each pump - These are directly incremented by EXTI ISRs
volatile uint32_t pump1_volume = 0;
volatile uint32_t pump2_volume = 0;
volatile uint32_t pump3_volume = 0;
volatile uint32_t pump4_volume = 0;

// petrol tank status - managed by updatePetrol thread
volatile bool petrol_sufficient = true;

// W5500 network buffers and info
uint8_t dhcp_buffer[1024];
uint8_t dns_buffer[1024]; // If DNS was implemented
volatile bool ip_assigned = false; // Flag to indicate if IP is assigned via DHCP

// MQTT variables
Network network;
MQTTClient client;
unsigned char sendbuf[256], readbuf[256]; // Increased buffer size for MQTT messages
MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;

// General purpose buffer for ITM_SendChar or LCD
char message_buffer[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
void func_updateLCD(void *argument);
void func_updateCloud(void *argument);
void func_updatePetrol(void *argument);

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
 * It should only increment the volume counters as pulses arrive from flow sensors.
 * @param GPIO_Pin: Specifies the pins connected to EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Disable interrupts briefly to ensure atomic updates of shared variables (uint32_t)
    // This is safer than semaphores in ISRs for simple, short critical sections.
    __disable_irq();

    if (petrol_tank_volume > 0) { // Only decrement if there's petrol left
        if (GPIO_Pin == pump_1_info_Pin) {
            petrol_tank_volume--;
            pump1_volume++;
        } else if (GPIO_Pin == pump_2_info_Pin) {
            petrol_tank_volume--;
            pump2_volume++;
        } else if (GPIO_Pin == pump_3_info_Pin) {
            petrol_tank_volume--;
            pump3_volume++;
        } else if (GPIO_Pin == pump_4_info_Pin) {
            petrol_tank_volume--;
            pump4_volume++;
        }
    }
    // Re-enable interrupts
    __enable_irq();

    // The logic to stop boards (HAL_GPIO_WritePin) should be in the updatePetrol task,
    // not in the ISR, as it's not time-critical and involves GPIO writes.
}

// W5500 SPI Chip Select functions
void wizchip_select(void) {
	HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_RESET); // Assert CS (active low)
}

void wizchip_deselect(void) {
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

    // Set W5500 RX/TX buffer sizes for each socket
    uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2}; // 2KB for each of 8 sockets
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
        // In a real RTOS, you'd use osDelay(1) here if W5500Init was in a task
        // but since it's in main() before scheduler start, busy-waiting is okay for setup.
        // For debugging, consider a small HAL_Delay or ITM_SendChar to see progress.
        HAL_Delay(10); // Small delay to prevent tight loop, though scheduler isn't running
    }

    if(!ip_assigned) {
        sprintf(message_buffer, "DHCP Failed! Using static IP (if configured).\r\n");
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
    sprintf(message_buffer, "IP:  %d.%d.%d.%d\r\nGW:  %d.%d.%d.%d\r\nSN:  %d.%d.%d.%d\r\n",
        net_info_local.ip[0], net_info_local.ip[1], net_info_local.ip[2], net_info_local.ip[3],
        net_info_local.gw[0], net_info_local.gw[1], net_info_local.gw[2], net_info_local.gw[3],
        net_info_local.sn[0], net_info_local.sn[1], net_info_local.sn[2], net_info_local.sn[3]
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
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
  MX_USART2_UART_Init(); // Initialize UART for debug messages
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  // W5500 initialization - run before RTOS starts for network setup
  W5500Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of mySemaphore01 */
  // This semaphore can be used to protect multi-byte reads/writes to shared variables
  // if not using __disable_irq() / __enable_irq().
  // For ISRs, direct interrupt disabling is safer.
  mySemaphore01Handle = osSemaphoreNew(1, 1, &mySemaphore01_attributes);

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
  /* creation of updateLCD */
  updateLCDHandle = osThreadNew(func_updateLCD, NULL, &updateLCD_attributes);

  /* creation of updateCloud */
  updateCloudHandle = osThreadNew(func_updateCloud, NULL, &updateCloud_attributes);

  /* creation of updatePetrol */
  updatePetrolHandle = osThreadNew(func_updatePetrol, NULL, &updatePetrol_attributes);

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
  // Initial state for W5500 Chip Select: HIGH (de-asserted)
  HAL_GPIO_WritePin(SPI_Chip_Select_GPIO_Port, SPI_Chip_Select_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(stop_Board2_GPIO_Port, stop_Board2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(stop_Board1_GPIO_Port, stop_Board1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_Chip_Select_Pin */
  GPIO_InitStruct.Pin = SPI_Chip_Select_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // It's usually good to keep CS pin pulled high when not active
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Changed from PULLDOWN to PULLUP
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_Chip_Select_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : stop_Board2_Pin */
  GPIO_InitStruct.Pin = stop_Board2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(stop_Board2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : pump_4_info_Pin pump_3_info_Pin pump_1_info_Pin pump_2_info_Pin */
  GPIO_InitStruct.Pin = pump_4_info_Pin|pump_3_info_Pin|pump_1_info_Pin|pump_2_info_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // Falling edge triggered interrupt
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Pull-up to ensure HIGH when idle
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : stop_Board1_Pin */
  GPIO_InitStruct.Pin = stop_Board1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(stop_Board1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  // Ensure the interrupt lines for the pump info pins are correctly enabled
  // For pump_1_info_Pin and pump_2_info_Pin and pump_3_info_Pin and pump_4_info_Pin,
  // these are likely on EXTI lines within 5-9 and 10-15.
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0); // Check your specific pin mappings
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0); // Check your specific pin mappings
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
    // Acquire semaphore if multiple tasks access pump volumes and petrol_tank_volume
    // or ensure atomic reads (e.g., disable interrupts for multi-byte reads).
    // For simple display, direct reads might be fine if variables are volatile.
    // Example: Display petrol tank volume and individual pump volumes
    sprintf(message_buffer, "Tank: %lu", petrol_tank_volume);
    // Assuming i2c-lcd.h provides lcd_put_cur and lcd_send_string
    // lcd_put_cur(0, 0);
    // lcd_send_string(message_buffer);

    sprintf(message_buffer, "P1:%lu P2:%lu", pump1_volume, pump2_volume);
    // lcd_put_cur(1, 0); // For 2nd line
    // lcd_send_string(message_buffer);

    // You might want to scroll or show other pumps on another line/screen
    // sprintf(message_buffer, "P3:%lu P4:%lu", pump3_volume, pump4_volume);
    // lcd_put_cur(2, 0); // For 3rd line if available
    // lcd_send_string(message_buffer);


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
  // --- MQTT Connection and Initialization (Moved outside the loop) ---
  // NOTE ON TLS (Port 8883): Your current W5500 and Paho MQTT client setup
  // does NOT include TLS/SSL. Connecting to port 8883 will fail.
  // Use port 1883 for unencrypted MQTT.
  //
  // NOTE ON HOSTNAMES: The `connect` function in socket.h typically expects an IP address (uint8_t[4]).
  // Your `MQTTNetwork.c` uses a hardcoded IP or expects a uint8_t array.
  // If you want to connect to a hostname like `4c5e19745ee742f4aa5c4a42bf15d3a8.s1.eu.hivemq.cloud`,
  // you MUST implement DNS resolution using W5500's DNS client features first.
  //
  // For demonstration, using a public test broker's IP and port 1883.
  // REPLACE THIS WITH YOUR ACTUAL BROKER IP and PORT (1883 for non-TLS)
  uint8_t broker_ip[4] = {18, 196, 172, 192}; // Example: IP of broker.hivemq.com (subject to change)
  int broker_port = 1883; // Use 1883 for unencrypted MQTT

  bool connected_to_mqtt = false;

  while (!connected_to_mqtt) {
      sprintf(message_buffer, "Connecting to network...\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);

      if (mqttnetwork_connect(&network, broker_ip, broker_port) != 0) {
          sprintf(message_buffer, "Network connection failed. Retrying...\r\n");
          HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
          osDelay(pdMS_TO_TICKS(5000)); // Wait and retry
          continue; // Go back to top of loop and retry network connect
      }

      sprintf(message_buffer, "Network connected. Initializing MQTT client...\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
      MQTTClientInit(&client, &network, 5000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));

      connectData.MQTTVersion = 3;
      connectData.clientID.cstring = "STM32MainBoard"; // Unique client ID
      connectData.username.cstring = "b022210152";  // Replace with HiveMQ username
      connectData.password.cstring = "b022210152UTEM!";  // Replace with HiveMQ password

      sprintf(message_buffer, "Connecting to MQTT broker...\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
      if (MQTTConnect(&client, &connectData) != SUCCESS) {
          sprintf(message_buffer, "MQTT connection failed. Retrying...\r\n");
          HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
          mqttnetwork_disconnect(&network); // Disconnect network if MQTT fails
          osDelay(pdMS_TO_TICKS(5000)); // Wait and retry
      } else {
          sprintf(message_buffer, "MQTT Connected!\r\n");
          HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
          connected_to_mqtt = true;
      }
  }

  /* Infinite loop for publishing */
  for(;;)
  {
    // Always check if client is connected before publishing
    if (!client.isconnected) {
        sprintf(message_buffer, "MQTT disconnected. Reconnecting...\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
        connected_to_mqtt = false; // Reset flag to re-enter connection logic
        // Break out of this loop to re-attempt full connection sequence at the top
        // (This assumes you'll restructure the infinite loop for connection/publish logic)
        // For simplicity now, let's just retry connect here
        if (mqttnetwork_connect(&network, broker_ip, broker_port) != 0) {
            osDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        if (MQTTConnect(&client, &connectData) != SUCCESS) {
            osDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        connected_to_mqtt = true;
        sprintf(message_buffer, "MQTT Reconnected!\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
    }

    // Use semaphore if these variables are also written by other threads (e.g., LCD thread if writing to them)
    // If only ISR writes and this thread reads, volatile is sufficient.
    // However, if LCD thread also reads and you want a consistent snapshot, a semaphore around this snprintf is good.
    osSemaphoreAcquire(mySemaphore01Handle, osWaitForever); // Protect shared volume data
    snprintf((char *)sendbuf, sizeof(sendbuf),
             "Tank:%lu,P1:%lu,P2:%lu,P3:%lu,P4:%lu,PetrolOk:%d",
             petrol_tank_volume, pump1_volume, pump2_volume, pump3_volume, pump4_volume, petrol_sufficient);
    osSemaphoreRelease(mySemaphore01Handle);

    message.payloadlen = strlen((char *)sendbuf);
    message.qos = QOS0; // Quality of Service 0 for simple fire-and-forget
    message.retained = 0;
    message.payload = sendbuf;

    if (MQTTPublish(&client, "stm32/petrol_data", &message) != SUCCESS) { // Use a specific topic
        sprintf(message_buffer, "MQTT Publish Failed!\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
        // On publish failure, client might be disconnected.
        // The !client.isconnected check at the top of loop will handle reconnect.
    } else {
        sprintf(message_buffer, "Published: %s\r\n", (char *)sendbuf);
        HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
    }

    osDelay(pdMS_TO_TICKS(5000)); // Publish every 5 seconds
  }
  /* USER CODE END func_updateCloud */
}

/* USER CODE BEGIN Header_func_updatePetrol */
/**
* @brief Function implementing the updatePetrol thread.
* This thread monitors the petrol tank volume and controls the stop signals.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_updatePetrol */
void func_updatePetrol(void *argument)
{
  /* USER CODE BEGIN func_updatePetrol */
  /* Infinite loop */
  for(;;)
  {
    // Use semaphore to safely read petrol_tank_volume if other tasks also write to it
    // (ISR writes, but this task also checks the flag, so it's okay)
    osSemaphoreAcquire(mySemaphore01Handle, osWaitForever); // Acquire semaphore before checking volume
    if (petrol_tank_volume == 0) {
        if (petrol_sufficient) { // Only change state and assert pins once
            petrol_sufficient = false; // Mark petrol as insufficient
            // Assert stop signals (drive LOW) to side boards
            HAL_GPIO_WritePin(stop_Board1_GPIO_Port, stop_Board1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(stop_Board2_GPIO_Port, stop_Board2_Pin, GPIO_PIN_RESET);
            sprintf(message_buffer, "PETROL TANK EMPTY! Pumps stopped.\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
        }
    } else { // Tank has petrol
        if (!petrol_sufficient) { // If it was previously insufficient but now has petrol (refilled)
            petrol_sufficient = true;
            // De-assert stop signals (drive HIGH) to allow pumps to start again
            HAL_GPIO_WritePin(stop_Board1_GPIO_Port, stop_Board1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(stop_Board2_GPIO_Port, stop_Board2_Pin, GPIO_PIN_SET);
            sprintf(message_buffer, "Petrol tank refilled. Pumps enabled.\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)message_buffer, strlen(message_buffer), HAL_MAX_DELAY);
        }
    }
    osSemaphoreRelease(mySemaphore01Handle); // Release semaphore

    osDelay(pdMS_TO_TICKS(100)); // Check tank status every 100ms
  }
  /* USER CODE END func_updatePetrol */
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
    // Potentially toggle an LED to indicate error
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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
