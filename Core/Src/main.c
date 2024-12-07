/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MS5611.h"
#include "LIS3.h"
#include "LSM6.h"
#include "LoRa.h"
#include "MicroSD.h"
#include "WQ.h"
#include "Servo.h"
#include "CircularBuffer.h"
#include <math.h>
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
struct ImuData {
	uint32_t time; //millis from start
	int32_t temp; //MS56 temperature, centigrade*10e2
	uint32_t press; //MS56 pressure, Pa
	float magData[3]; //LIS3 mag, mG
	float accelData[3]; //LSM6, mG
	float gyroData[3]; //LSM6, mdps
	int32_t altitude; //Altitude (zero at start), cm
	uint32_t flags; //Flags, (0|0|0|0|Land|ResSys|Eject|Start)
	uint32_t press0; //MS56 pressure at 0 Alt, Pa
	float vectAbs; //Absolute value of accel vector
	uint32_t wqAdr;
} imuData;

enum states {
	INIT, LORA_WAIT, MAIN, LANDING, DUMP
};

enum states lastState = LANDING;
enum states currentState = INIT;
CircularBuffer *cbPress;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
void StoreVectAbs(struct ImuData *dat);
void ImuGetAll(struct ImuData *imuData);
void ImuSaveAll(struct ImuData *imuData);
void FlashLED(uint16_t led);
void Error(uint8_t errorCode);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void init_state(){
	static uint8_t errorCode = 0;
	HAL_Delay(1000);
	if(currentState != lastState) {
		lastState = currentState;
		if(!MS5611_Init(&hspi1, MS_NSS_GPIO_Port, MS_NSS_Pin)) errorCode = 1;
		if(!LIS3_Init(&hspi1, LIS_NSS_GPIO_Port, LIS_NSS_Pin)) errorCode = 2;
		if(!LSM6_Init(&hspi1, LSM_NSS_GPIO_Port, LSM_NSS_Pin)) errorCode = 3;
		if(!LoRa_Init(&hspi1, LORA_NSS_GPIO_Port, LORA_NSS_Pin)) errorCode = 4;
		if(!microSD_Init()) errorCode = 5;
		if(!WQ_Init(&hspi1, WQ_NSS_GPIO_Port, WQ_NSS_Pin)) errorCode = 6;
		Servo_Init(&htim3, TIM_CHANNEL_1);
		HAL_TIM_Base_Start_IT(&htim11);
		if(!errorCode) { //POST_OK (DOT)
			MS5611_SetOS(MS56_OSR_4096, MS56_OSR_4096);
			LIS3_Config(LIS_CTRL1, LIS_MODE_HP | LIS_ODR_80);
			LIS3_Config(LIS_CTRL2, LIS_SCALE_4);
			LIS3_Config(LIS_CTRL3, LIS_CYCLIC);
			LSM6_ConfigAG(LSM6_ACCEL_16G | LSM6_CFG_12_5_Hz, LSM6_GYRO_2000DPS | LSM6_CFG_12_5_Hz);
			cbPress = CB_Create(PRESS_BUFFER_LEN);
			imuData.wqAdr = 0;
			currentState = LORA_WAIT;
		} else Error(errorCode);
	}

}

void lora_wait_state(){
	static uint8_t rxbuf[1];
	static uint8_t pingFlag = 0;
	if(currentState != lastState) {
		lastState = currentState;
	}

	if(!pingFlag) {
		if(HAL_GetTick() % 300 > 150) FlashLED(LED1_Pin);
		else FlashLED(LED2_Pin);
	} else if(HAL_GetTick() % 500 > 250) FlashLED(LED1_Pin);
	else FlashLED(0);

	if(LoRa_Receive(rxbuf, 1)) {
		if(rxbuf[0] == '0') {
			pingFlag = 1;
			LoRa_Transmit("Ping OK\n", 8);
		}
		if(pingFlag) {
			switch (rxbuf[0]){
				case '1':
					LoRa_Transmit("Starting\n", 9);
					currentState = MAIN;
					break;
				case '2':
					currentState = DUMP;
					LoRa_Transmit("Memory dump\n", 12);
					break;
				case '3':
					LoRa_Transmit("Erase All\n", 10);
					WQ_Chip_Erace();
					microSD_RemoveFile(SD_FILENAME);
					microSD_RemoveFile(SD_FILENAME_WQ);
					LoRa_Transmit("Done\n", 5);
					break;
				case '4':
					LoRa_Transmit("RS Test\n", 8);
					Servo_SetAngle(RS_ANGLE);
					HAL_TIM_Base_Start_IT(&htim11);
					break;
				default:
					break;
			}
		}
	}
}

void main_state(){
	if(currentState != lastState) {
		lastState = currentState;
		for(uint8_t i = 0; i < 10; i++) {
			MS5611_Read(&imuData.temp, &imuData.press0); //Skip 10 measurements aka cold start
			HAL_Delay(50);
		}
		MS5611_Read(&imuData.temp, &imuData.press0); //press0 for height zero level
		StoreVectAbs(&imuData);
		imuData.time = HAL_GetTick();
	}

	if(HAL_GetTick() - imuData.time >= DATA_PERIOD) {
		HAL_ADC_Start(&hadc1);
		ImuGetAll(&imuData);

		if(imuData.altitude > START_TH) bitSet(imuData.flags, 0); //Start flag
		if(bitRead(imuData.flags, 0) && ADC1->DR < EJECT_TH) bitSet(imuData.flags, 1); //Eject flag

		if(bitRead(imuData.flags, 1) && !bitRead(imuData.flags, 2) && (imuData.altitude <= RS_EJC_HEIGHT)) {
			bitSet(imuData.flags, 2); //ResSys flag
			Servo_SetAngle(RS_ANGLE);
			HAL_TIM_Base_Start_IT(&htim11);
		}

		if(bitRead(imuData.flags, 2)) {
			CB_Add(cbPress, imuData.press);
			if(CB_Diff(cbPress) < PRESS_LAND_DELTA) {
				bitSet(imuData.flags, 3); //Land flag
				currentState = LANDING;
			}
		}
		ImuSaveAll(&imuData);
	}
}

void landing_state(){
	if(currentState != lastState) {
		lastState = currentState;
		imuData.time = HAL_GetTick();
		Servo_SetAngle(0);
	}
	if(HAL_GetTick() - imuData.time >= DATA_PERIOD_LND) {
		ImuGetAll(&imuData);
		ImuSaveAll(&imuData);
	}
	//POST Iterate landing
}

void dump_state(){
	if(currentState != lastState) {
		lastState = currentState;
	}
	uint8_t buf[FRAME_SIZE];
	for(uint32_t addr = 0; addr < 0xFFFFFF; addr += FRAME_SIZE) {
		FlashLED(LED2_Pin);
		WQ_Read(addr, buf, FRAME_SIZE);
		if(buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF && buf[3] == 0xFF) break;
		LoRa_Transmit(buf, FRAME_SIZE);
		FlashLED(LED1_Pin);
		if(microSD_Write(buf, FRAME_SIZE, SD_FILENAME_WQ) != FR_OK) {
			FlashLED(LED_ERR_Pin);
			HAL_Delay(1000);
		}
	}
	LoRa_Transmit("Done\n", 5);
	currentState = LORA_WAIT;
}

void StoreVectAbs(struct ImuData *dat){
	dat->vectAbs = 0;
	for(uint8_t i = 0; i < 3; i++)
		dat->vectAbs += dat->accelData[i] * dat->accelData[i];
	dat->vectAbs = sqrt(dat->vectAbs);
}

void ImuGetAll(struct ImuData *imuData){
	imuData->time = HAL_GetTick();
	MS5611_Read(&imuData->temp, &imuData->press);
	LIS3_Read(imuData->magData);
	LSM6_Read(imuData->accelData, imuData->gyroData);
	StoreVectAbs(imuData);
	imuData->altitude = MS5611_GetAltitude(&imuData->press, &imuData->press0);
}

void ImuSaveAll(struct ImuData *imuData){
	FlashLED(LED2_Pin);
	LoRa_Transmit(imuData, FRAME_SIZE);
	WQ_Write(imuData->wqAdr, imuData, FRAME_SIZE);
	imuData->wqAdr += FRAME_SIZE;
	FlashLED(LED1_Pin);
	if(microSD_Write(imuData, FRAME_SIZE, SD_FILENAME) != FR_OK) {
		FlashLED(LED_ERR_Pin);
		MX_SDIO_SD_Init();
		MX_FATFS_Init();
		microSD_Init();
	}
	FlashLED(0);
}

void FlashLED(uint16_t led){
	LED1_GPIO_Port->ODR &= ~LED1_Pin;
	LED2_GPIO_Port->ODR &= ~LED2_Pin;
	LED_ERR_GPIO_Port->ODR &= ~LED_ERR_Pin;
//@formatter:off
	switch (led){
		case LED1_Pin: LED1_GPIO_Port->ODR |= LED1_Pin; break;
		case LED2_Pin: LED2_GPIO_Port->ODR |= LED2_Pin; break;
		case LED_ERR_Pin: LED_ERR_GPIO_Port->ODR |= LED_ERR_Pin; break;
		default: break;
	}}
//@formatter:on

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM11){
		HAL_TIM_Base_Stop(&htim11);
		htim11.Instance->CNT=0;
		Servo_SetAngle(0);
	}
}

void Error(uint8_t errCode){
	while(1) {
		for(uint8_t i = 0; i < errCode; i++) {
			FlashLED(LED_ERR_Pin);
			HAL_Delay(200);
			FlashLED(0);
			HAL_Delay(200);
		}
		HAL_Delay(500);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		switch (currentState){
			case INIT:
				init_state();
				break;
			case LORA_WAIT:
				lora_wait_state();
				break;
			case MAIN:
				main_state();
				break;
			case LANDING:
				landing_state();
				break;
			case DUMP:
				dump_state();
			default:
				break;
		}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd.Init.ClockDiv = 6;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1679;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 8399;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 30000;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim11, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PWR2_EN_Pin|WQ_NSS_Pin|LORA_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED_ERR_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WQ_HOLD_GPIO_Port, WQ_HOLD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MS_NSS_Pin|LIS_NSS_Pin|LSM_NSS_Pin|ADC_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PWR2_EN_Pin */
  GPIO_InitStruct.Pin = PWR2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR2_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED_ERR_Pin LED2_Pin WQ_HOLD_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED_ERR_Pin|LED2_Pin|WQ_HOLD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : WQ_NSS_Pin LORA_NSS_Pin */
  GPIO_InitStruct.Pin = WQ_NSS_Pin|LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MS_NSS_Pin LIS_NSS_Pin LSM_NSS_Pin ADC_NSS_Pin */
  GPIO_InitStruct.Pin = MS_NSS_Pin|LIS_NSS_Pin|LSM_NSS_Pin|ADC_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SDIO_CD_Pin JMP_PROG_Pin */
  GPIO_InitStruct.Pin = SDIO_CD_Pin|JMP_PROG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();

	while(1);
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
