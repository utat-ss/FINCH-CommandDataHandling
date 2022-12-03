/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "string.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <time.h>	// seeding pseudo RNG
#include <stdlib.h>	// for rng
//#include "cmsis_os2.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#if defined (__ICCARM__) || defined (__ARMCC_VERSION)
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#elif defined(__GNUC__)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)

GETCHAR_PROTOTYPE;
#endif

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30000260
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30000260))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for CommandHandler */
osThreadId_t CommandHandlerHandle;
const osThreadAttr_t CommandHandler_attributes = {
  .name = "CommandHandler",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
  .name = "Task2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for CommandIdle */
osThreadId_t CommandIdleHandle;
const osThreadAttr_t CommandIdle_attributes = {
  .name = "CommandIdle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task3 */
osThreadId_t Task3Handle;
const osThreadAttr_t Task3_attributes = {
  .name = "Task3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for ThreadTerminato */
osThreadId_t ThreadTerminatoHandle;
const osThreadAttr_t ThreadTerminato_attributes = {
  .name = "ThreadTerminato",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for CommandQueue */
osMessageQueueId_t CommandQueueHandle;
const osMessageQueueAttr_t CommandQueue_attributes = {
  .name = "CommandQueue"
};
/* Definitions for FollowThroughSem */
osSemaphoreId_t FollowThroughSemHandle;
const osSemaphoreAttr_t FollowThroughSem_attributes = {
  .name = "FollowThroughSem"
};
/* Definitions for ThreadExitSem */
osSemaphoreId_t ThreadExitSemHandle;
const osSemaphoreAttr_t ThreadExitSem_attributes = {
  .name = "ThreadExitSem"
};
/* USER CODE BEGIN PV */
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[] = {42, 69};
uint8_t RxData[2];
uint8_t USART_DATA[2];

osMutexId_t FollowThroughMutexHandle;

const osMutexAttr_t FollowThroughMutex_attributes = {
    "FollowThroughMutex",
		osMutexRobust | osMutexPrioInherit,
		NULL,
		0U
  };

int value;

char msg[] = "Hello this is a message\n";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_FDCAN1_Init(void);
void StartCommandHandler(void *argument);
void StartTask1(void *argument);
void StartTask2(void *argument);
void StartCommandIdle(void *argument);
void StartTask3(void *argument);
void StartThreadTerminator(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int count=0;
gpio_t user_button;

void check_button(){
	if (HAL_GPIO_ReadPin(user_button.bank, user_button.pin) == 1){	// button is pressed
		printf("DEBUG: button pressed!\r\n");
		while(HAL_GPIO_ReadPin(user_button.bank, user_button.pin) == 1){ // wait until release
			// do nothing
		}
		printf("DEBUG: button released!\r\n");
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
	//OBC = 0: EPS, NEEDS TO SEND DATA TO OBC.
	//OBC = 1: ON-BOARD COMPUTER, NEEDS TO HANDLE INCOMING DATA
	//IF TX, OBC = 0
  int OBC = 1;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

	/**
	 * user button
	 * um1974-stm32-nucleo144-boards-mb1137-stmicroelectronics.pdf
	 * section 6.6
	 */
	user_button.bank = GPIOC;
	user_button.pin = GPIO_PIN_13;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

  //Enable Interrupt Mode for USART3
  HAL_UART_Receive_IT(&huart3, USART_DATA, 1);

  printf("USART3 initialized\r\n");
  printf("FDCAN1 initialized\r\n");

  /* reception filter for Rx FIFO 0 */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
//  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXBUFFER;
//  sFilterConfig.FilterID1 = 0x7FF;

  /**
   * Yong Da Li, Reid Sox-Harris
   * Saturday, Sept 03, 2022
   * don't use the FilterConfig = FDCAN_FILTER_TO_RXBUFFER mode
   * couldn't get it working
   * use the FilterConfig = FDCAN_FILTER_TO_RXFIFO0
   * 	then need to set FilterType = FDCAN_FILTER_MASK
   * section 4.3.1 of AN5348 App Note: FDCAN Peripheral on STM32 devices
   * Classic bit mask filter: to match groups of identifiers by masking bits of a received identifier. The first ID
   * configured is used as message ID filter, the second ID is used as filter mask. Each zero bit at the filter mask
   * masks out the corresponding bit position of the configured ID filter
   */
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x111;	// since all filterID2 is set to 1's, then must be exact match on filterID1
  sFilterConfig.FilterID2 = 0x7FF;
  // don't need the other struct components since we're using RX FIFO
  // they're only used if FilterConfig == FDCAN_FITLER_TO_RXBUFFER

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK){
	  Error_Handler();
  }
  printf("FDCAN filter is configured\r\n");

//  /* Configure global filter:
//	Filter all remote frames with STD and EXT ID
//	Reject non matching frames with STD ID and EXT ID
//   */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK){
	  Error_Handler();
  }

  if ((&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK){
	  Error_Handler();
  }
  printf("FDCAN global filter is configured\r\n");

  /* Prepare Tx message Header */
  TxHeader.Identifier = 0x111;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;	// transmitting node is error active
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;	// needs to be set to classic CAN mode, not FDCAN mode
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;		// message marker to identify Tx message status, not too important
  printf("prepared TxHeader\r\n");

  // don't do any delay compensation
  // only required if we're using BRS = bit rate switching mode
  // for this basic setup, we're not doing that

  /* === trying to put FDCAN into interrupt mode === */
  /**
   * check out the example called "FDCAN_Com_IT" for the STM32H743I_EVAL discover board
   * open up a new project, and navigate to the example selector
   * Yong Da is basing his code from there
   */

  printf("putting the FDCAN module into interrupt mode\r\n");
  /* Configure Rx FIFO 0 watermark to 2 */
  HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 2);

  /* Activate Rx FIFO 0 watermark notification */
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_WATERMARK, 0);


  /* === starting FDCAN module, must be last thing to do === */
  printf("trying to start FDCAN module\r\n");\
  if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK){
	  Error_Handler();
  }
  printf("successfully started FDCAN module\r\n");


  /* === seed random number === */
  srand(time(NULL));


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  FollowThroughMutexHandle = osMutexNew(&FollowThroughMutex_attributes);
  //if(FollowThroughMutexHandle == NULL)
  //	Error_Handler();
  //else
  //	printf("%s", osMutexGetName(FollowThroughMutexHandle));
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of FollowThroughSem */
  FollowThroughSemHandle = osSemaphoreNew(1, 1, &FollowThroughSem_attributes);

  /* creation of ThreadExitSem */
  ThreadExitSemHandle = osSemaphoreNew(1, 1, &ThreadExitSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  osSemaphoreAcquire(ThreadExitSemHandle, 0); //Start at 0. //Change the creation instead to start at 0 later.
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CommandQueue */
  CommandQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &CommandQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CommandHandler */
  CommandHandlerHandle = osThreadNew(StartCommandHandler, NULL, &CommandHandler_attributes);

  /* creation of Task1 */
  Task1Handle = osThreadNew(StartTask1, NULL, &Task1_attributes);

  /* creation of Task2 */
  Task2Handle = osThreadNew(StartTask2, NULL, &Task2_attributes);

  /* creation of CommandIdle */
  CommandIdleHandle = osThreadNew(StartCommandIdle, NULL, &CommandIdle_attributes);

  /* creation of Task3 */
  Task3Handle = osThreadNew(StartTask3, NULL, &Task3_attributes);

  /* creation of ThreadTerminato */
  ThreadTerminatoHandle = osThreadNew(StartThreadTerminator, NULL, &ThreadTerminato_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadTerminate(Task1Handle);
  osThreadTerminate(Task2Handle);
  osThreadTerminate(Task3Handle);
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
	  if(OBC == 0)
	  {
		  /* ========== TX super loop ========== */
		  printf("count: %d\n\r", count);
		  count++;

		  printf("trying to add message to TxFifo: TxData[0]=%3d, TxData[1]=%3d\r\n", TxData[0], TxData[1]);
		  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK){
			  Error_Handler();
		  }
		  printf("successfully added message to TxFifo\r\n");

		  TxData[0] = (uint8_t)rand();
		  TxData[1] = (uint8_t)rand();
		  //Try changing this value here back and forth from 0x111 to 0x100. First make sure it works.
		  if(TxHeader.Identifier == 0x111)
		  {
			  printf("Changing Identifier to 0x111\r\n");
			  TxHeader.Identifier = 0x110;
		  }
		  else
		  {
			  printf("Changing Identifier to 0x110\r\n");
			  TxHeader.Identifier = 0x111;
		  }


		  check_button();

		  HAL_Delay(250);
		  /* ========== TX super loop end ========== */
	  }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 16;
  hfdcan1.Init.NominalTimeSeg1 = 63;
  hfdcan1.Init.NominalTimeSeg2 = 16;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 32;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 32;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 1;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 4;
  hfdcan1.Init.TxBuffersNbr = 4;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK) == RESET){
		Error_Handler();
	}

	printf("hit the callback RXFifo0\n\r");

	// https://overiq.com/c-programming-101/local-global-and-static-variables-in-c/
	static int count = 0;	// retain the variable value between different function calls

	/* Retrieve Rx message from RX FIFO0 */
	if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
	  Error_Handler();
	}

	printf("count=%3d, received RxData[0]=%3d, RxData[1]=%d\r\n", count, RxData[0], RxData[1]);
	count++;
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

GETCHAR_PROTOTYPE
{
	uint8_t ch = 0;
	/* Place your implementation of fgetc here */

	/* Clear the Overrun flag just before receiving the first character */
	__HAL_UART_CLEAR_OREFLAG(&huart3);

	/* Wait for reception of a character on the USART1 RX line
     and echo this character on console */
	HAL_UART_Receive(&huart3, (uint16_t *)&ch, 1, 0xFFFF);
	HAL_UART_Transmit(&huart3, (uint16_t *)&ch, 1, 0xFFFF);
	return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart3)
{
	printf("HAL_UART_RxCpltCallback: %u\n", USART_DATA[0]);
	uint16_t command = (uint16_t) USART_DATA[0];
	osMessageQueuePut(CommandQueueHandle, (void*) &command, 0, 0);
	HAL_UART_Receive_IT(huart3, USART_DATA, 1);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCommandHandler */
/**
  * @brief  Function implementing the CommandHandler thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCommandHandler */
void StartCommandHandler(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint16_t command;
  /* Infinite loop */
  while(1)
  {
  	if(osMessageQueueGet(CommandQueueHandle, (void*) &command, NULL, osWaitForever) == osOK)
  	{

  		command = command - 48; //From ASCII to int

  		if(command < 0 || command > 9)
  		{
  			printf("Invalid INT. Try again\n");
  		}

  		else if(command == 1)
  		{
  			if(osThreadGetState(Task1Handle) == osThreadInactive ||
  				 osThreadGetState(Task1Handle) == osThreadTerminated)
  			{
  				Task1Handle = osThreadNew(StartTask1, NULL, &Task1_attributes);
  			}
  			else
  			{
  				printf("Task1 Is Already Running\n");
  			}
  		}

  		else if(command == 2)
  		{
  			if(osThreadGetState(Task2Handle) == osThreadInactive ||
  			   osThreadGetState(Task2Handle) == osThreadTerminated)
  			{
  				Task2Handle = osThreadNew(StartTask2, NULL, &Task2_attributes);
  			}
  		  else
  			{
  			  printf("Task2 Is Already Running\n");
  			}
  		}

  		else if(command == 3)
  		{
  			if(osThreadGetState(Task3Handle) == osThreadInactive ||
  			   osThreadGetState(Task3Handle) == osThreadTerminated)
  			{
  				Task3Handle = osThreadNew(StartTask3, NULL, &Task3_attributes);
  			}
  			else
  			{
  				printf("Task3 Is Already Running\n");
  			}
  		}
  		else
  		{
  			printf("Command not implemented\n");
  		}
  	}

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask1 */
/**
* @brief Function implementing the Task1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask1 */
void StartTask1(void *argument)
{
  /* USER CODE BEGIN StartTask1 */
  /* Infinite loop */
  for(;;)
  {
  	printf("Task 1 is running\n");
    osDelay(100);
  }
  /* USER CODE END StartTask1 */
}

/* USER CODE BEGIN Header_StartTask2 */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask2 */
void StartTask2(void *argument)
{
  /* USER CODE BEGIN StartTask2 */

	if(osSemaphoreAcquire(FollowThroughSemHandle, osWaitForever) == osOK)
	{

		printf("Task 2 is running\n");

		/* Non-Infinite loop */
		for(int i = -1 ; i < 1000; ++ i)
		{
			for(int d = 0; d < 30000; ++d)
			{
				;
			}

			if((i+1) % 250 == 0)
				printf("Task 2 is %u %% finished\n", (uint16_t)((i+1) / 10));

		}
	}
	osSemaphoreRelease(ThreadExitSemHandle);
  osThreadExit(); //This releases Mutex (Or it should)
  /* USER CODE END StartTask2 */
}

/* USER CODE BEGIN Header_StartCommandIdle */
/**
* @brief Function implementing the CommandIdle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommandIdle */
void StartCommandIdle(void *argument)
{
  /* USER CODE BEGIN StartCommandIdle */
  /* Infinite loop */
  for(;;)
  {
  	//Inf Loop, idle-ing waiting for interrupts
  }
  /* USER CODE END StartCommandIdle */
}

/* USER CODE BEGIN Header_StartTask3 */
/**
* @brief Function implementing the Task3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask3 */
void StartTask3(void *argument)
{
  /* USER CODE BEGIN StartTask3 */

	if(osSemaphoreAcquire(FollowThroughSemHandle, osWaitForever) == osOK)
	{
		printf("Task 3 is running\n");

		/* Non-Infinite loop */
		for(int i = -1 ; i < 1000; ++ i)
		{
			for(int d = 0; d < 30000; ++d)
			{
				;
			}
			if((i+1) % 250 == 0)
				printf("Task 3 is %u %% finished\n", (uint16_t)((i+1) / 10));

		}
	}
	osSemaphoreRelease(ThreadExitSemHandle);
	osThreadExit(); //This releases Mutex (Or it should)
  /* USER CODE END StartTask3 */
}

/* USER CODE BEGIN Header_StartThreadTerminator */
/**
* @brief Function implementing the ThreadTerminato thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartThreadTerminator */
void StartThreadTerminator(void *argument)
{
  /* USER CODE BEGIN StartThreadTerminator */
  /* Infinite loop */
  for(;;)
  {
    while(osSemaphoreAcquire(ThreadExitSemHandle, osWaitForever) == osOK)
    {
    	osSemaphoreRelease(FollowThroughSemHandle);
    }
  }
  /* USER CODE END StartThreadTerminator */
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
  if (htim->Instance == TIM6) {
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
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	  HAL_Delay(1000);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  HAL_Delay(1000);
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
