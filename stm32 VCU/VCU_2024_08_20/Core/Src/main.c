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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "VCU.h"
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
ADC_HandleTypeDef hadc2;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ---------- MODOS DEBUG ----------
#define DEBUG 1

// ---------- VARIABLES DEL CAN ----------
FDCAN_TxHeaderTypeDef TxHeader_Inv;
FDCAN_RxHeaderTypeDef RxHeader_Inv;
FDCAN_TxHeaderTypeDef TxHeader_Acu;
FDCAN_RxHeaderTypeDef RxHeader_Acu;

uint8_t TxData_Inv[8];
uint8_t RxData_Inv[8];
uint8_t TxData_Acu[8];
uint8_t RxData_Acu[8];

// Datos a recibir del inversor
INT32U datos_inversor[N_DATOS_INV] = {T_MOTOR, T_IGBT, T_AIR, N_ACTUAL, I_ACTUAL};

// Detector de flanco botón de arranque
int start_button_act;
int start_button_ant = 0; //1

// ---------- VARIABLES DE LECTURA DE SENSORES ----------

// Inversor
int inv_dc_bus_voltage = 0; // Lectura de DC_BUS_VOLTAGE
int inv_t_motor;        // Lectura de motor temperature
int inv_t_igbt;         // Lectura de power stage temperature
int inv_t_air;          // Lectura de air temperature
int inv_n_actual;       // Lectura de speed actual value

// Sensores
int s1_aceleracion; // Lectura del sensor 1 del pedal de aceleración
int s2_aceleracion; // Lectura del sensor 2 del pedal de aceleración
float s1_aceleracion_aux;
float s2_aceleracion_aux;
int s_freno; // Lectura del sensor de freno
float s_freno_aux;
int sdd_suspension; // Lectura del sensor delantero derecho de suspensión
int sdi_suspension; // Lectura del sensor delantero izquierdo de suspensión
float aux_velocidad;
float v_celda_min = 3600; // Contiene el ultimo valor de tension minima de una celda enviada por el AMS.

// ---------- VARIABLES DE CONTROL DEL INVERSOR ----------
int porcentaje_pedal_acel;
uint16_t torque_1;
uint16_t torque_2;
uint16_t torque_total;
int torque_limitado;
uint16_t real_torque;
int media_s_acel;
uint8_t state = 0;

// Revisar normativa
int flag_EV_2_3 = 0;
int flag_T11_8_9 = 0;
int count_T11_8_9 = 0;


char uart_msg[100];
uint8_t error = 0;

void print(char uart_buffer[]);
void printValue(int value);
void printHex(uint8_t value);

void ADC1_Select_SF(void);
void ADC1_Select_SA1(void);
void ADC1_Select_SA2(void);

void ADC2_Select_FL(void);
void ADC2_Select_FR(void);
void ADC2_Select_RL(void);
void ADC2_Select_RR(void);

uint8_t setTorque(void);

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //Inicialización de buses CAN

  //Inversor
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK){
#if DEBUG
print("Error al inicializar CAN_INV");
#endif
  }

  //Acumulador
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK){
#if DEBUG
print("Error al inicializar CAN_ACU");
#endif
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
#if DEBUG
print("Error al activar NOTIFICATION CAN_ACU");
#endif
  }


  //---------- SECUENCIA DE ARRANQUE ----------

  //Espera ACK inversor (DC bus)
  while (config_inv_lectura_v == 0){
#if(DEBUG)
print("Solicitar tensión inversor");
#endif
	if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader_Inv, RxData_Inv) == HAL_OK){
		if(RxHeader_Inv.Identifier == TX_STATE_7 && RxHeader_Inv.DataLength == 6){
			config_inv_lectura_v = 1; //Sale del bucle
#if DEBUG
print("CAN_INV: Lectura de DC_BUS_VOLTAGE correctamente");
#endif
		}
	}
  }

  //Estado STAND BY inversor
  while(state!=3){
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader_Inv, RxData_Inv);
	if(RxHeader_Inv.Identifier == TX_STATE_3){
		state = RxData_Inv[2]>>0x1;
	}
  }

	//PRE-CHARGE
  while(precarga_inv == 0){
#if DEBUG
print("Precarga");
#endif
	//Lectura DC_BUS_VOLTAGE del CAN_INV
	if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader_Inv, RxData_Inv) == HAL_OK){
		if(RxHeader_Inv.Identifier == TX_STATE_7 && RxHeader_Inv.DataLength == 6){
			uint8_t byte0 = RxData_Inv[0];
			uint8_t byte1 = RxData_Inv[1];
			printHex(byte0);
			printHex(byte1);
			//inv_dc_bus_voltage = (int)((byte1<<8)|byte0);
#if DEBUG
print("DC_BUS_VOLTAGE (V):");
printValue((int)((byte1<<8)|byte0));
#endif

			//Reenvío DC_BUS_VOLTAGE al AMS por CAN_ACU
			TxHeader_Acu.Identifier = ID_dc_bus_voltage;
			TxHeader_Acu.DataLength = 2;
			TxHeader_Acu.IdType = FDCAN_EXTENDED_ID;
			TxHeader_Acu.FDFormat = FDCAN_CLASSIC_CAN;
			TxHeader_Acu.TxFrameType = FDCAN_DATA_FRAME;

			TxData_Acu[0] = byte0;
			TxData_Acu[1] = byte1;
			if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Acu, TxData_Acu) == HAL_OK){
#if DEBUG
print("CAN_ACU: DC_BUS_VOLTAGE enviado a AMS");
#endif
			}

	    }
	}
  }

#if DEBUG
print("state : stand by");
#endif

/*
 * TIM16 -> APB2 => 264MHzw
 * 10 ms interruption => 10ms * 264MHz = 2640000
 * prescalado 264 (por ejemplo)
 * timer count = 2640000 / 264 = 10000
 */
  HAL_TIM_Base_Start_IT(&htim16);


  //Estado READY inversor
  TxHeader_Inv.Identifier = RX_SETPOINT_1;
  TxHeader_Inv.DataLength = 3;
  TxHeader_Inv.IdType = FDCAN_STANDARD_ID;

  TxData_Inv[0] = 0x0;
  TxData_Inv[1] = 0x0;
  TxData_Inv[2] = 0x4;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);
  while(state!=4){
	  HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader_Inv, RxData_Inv);
	  if(RxHeader_Inv.Identifier == TX_STATE_3){
		  state = RxData_Inv[2]>>0x1;
	  }
  }
#if DEBUG
print("state: ready");
#endif


  //Estado TORQUE inversor
  TxHeader_Inv.Identifier = RX_SETPOINT_1;
  TxHeader_Inv.DataLength = 3;
  TxHeader_Inv.IdType = FDCAN_STANDARD_ID;


  TxData_Inv[0] = 0x0;
  TxData_Inv[1] = 0x0;
  TxData_Inv[2] = 0x6;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);
  while(state!=6){
	  HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader_Inv, RxData_Inv);
	  if(RxHeader_Inv.Identifier == TX_STATE_3){
		  state = RxData_Inv[2]>>0x1;
	  }
  }


  //Espera a que se pulse el botón de arranque mientras se pisa el freno
  while(boton_arranque == 0){
	  //Enciende el LED del boton para avisar al piloto
	  HAL_GPIO_WritePin(START_BUTTON_LED_GPIO_Port, START_BUTTON_LED_Pin, GPIO_PIN_RESET);

	  start_button_act = HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin);
	  if(start_button_act == 1 && start_button_ant == 0){
		  ADC1_Select_SF();
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		  s_freno= HAL_ADC_GetValue(&hadc1);
		  HAL_ADC_Stop(&hadc1);
#if DEBUG
print("Botón Start + Freno:");
printValue(s_freno);
#endif
		  if(s_freno>UMBRAL_FRENO){
			  boton_arranque = 1;
#if DEBUG
print("Coche arrancado correctamente");
#endif
		  }
		  else{
#if DEBUG
print("Pulsar freno para arrancar");
#endif
		  }
	  }
  }


  // Activar READY-TO-DRIVE-SOUND (RTDS) durante 2s
#if DEBUG
print("RTDS sonando");
#endif
  HAL_GPIO_WritePin(START_BUTTON_LED_GPIO_Port, START_BUTTON_LED_Pin, GPIO_PIN_SET); //Apaga LED botón
  HAL_GPIO_WritePin(RTDS_GPIO_Port, RTDS_Pin, GPIO_PIN_SET); //Enciende RTDS
  HAL_Delay(2000);
  HAL_GPIO_WritePin(RTDS_GPIO_Port, RTDS_Pin, GPIO_PIN_RESET); //Apaga RTDS
#if DEBUG
  print("RTDS apagado");
#endif


  // Avisar a resto de ECUs de que pueden comenzar ya a mandar datos al CAN (RTD_all)
  // Inicia telemetria y activa los ventiladores
  /*TxHeader_Acu.Identifier = ID_RTD_all;
  TxHeader_Acu.DataLength = 1;
  TxData_Acu[0] = 1;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Acu, TxData_Acu);*/


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//Estado TORQUE inversor
		TxHeader_Inv.Identifier = RX_SETPOINT_1;
		TxHeader_Inv.DataLength = 3;
		TxHeader_Inv.IdType = FDCAN_STANDARD_ID;


		TxData_Inv[0] = 0x0;
		TxData_Inv[1] = 0x0;
		TxData_Inv[2] = 0x6;
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);

		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader_Inv, RxData_Inv);
		if(RxHeader_Inv.Identifier == TX_STATE_3){
			state = RxData_Inv[2]>>0x1;
			if(state == 10){
				error = RxData_Inv[0];
			}
		}
	  	  // Envío datos telemetría
	  	  if(0){
	  		  while(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader_Inv, RxData_Inv) == HAL_OK){
	  			  if(RxHeader_Inv.Identifier == txID_inversor){
	  				  switch(RxData_Inv[0]){
	  				  case DC_BUS_VOLTAGE:
	  					  TxHeader_Acu.Identifier = ID_dc_bus_voltage;
	  					  TxHeader_Acu.DataLength = 2;
	  					  TxHeader_Acu.IdType = FDCAN_EXTENDED_ID;
	  					  inv_dc_bus_voltage = (int)RxData_Inv[0] << 8;
	  					  TxData_Acu[0] = inv_dc_bus_voltage >> 8 & 0xFF;
	  					  TxData_Acu[1] = inv_dc_bus_voltage & 0xFF;
	  					  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Acu, TxData_Acu);
	  					  break;

	  				  case T_MOTOR:
	  					  TxHeader_Acu.Identifier = ID_t_motor;
	  					  TxHeader_Acu.DataLength = 2;
	  					  TxHeader_Acu.IdType = FDCAN_EXTENDED_ID;
	  					  TxData_Acu[0] = RxData_Inv[2];
	  					  TxData_Acu[1] = RxData_Inv[1];
	  					  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Acu, TxData_Acu);
	  					  break;

	  				  case T_IGBT:
	  					  TxHeader_Acu.Identifier = ID_t_igbt;
	  					  TxHeader_Acu.DataLength = 2;
	  					  TxHeader_Acu.IdType = FDCAN_EXTENDED_ID;
	  					  TxData_Acu[0] = RxData_Inv[2];
	  					  TxData_Acu[1] = RxData_Inv[1];
	  					  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Acu, TxData_Acu);
	  					  break;

	  				  case T_AIR:
	  					  TxHeader_Acu.Identifier = ID_t_air;
	  					  TxHeader_Acu.DataLength = 2;
	  					  TxHeader_Acu.IdType = FDCAN_EXTENDED_ID;
	  					  TxData_Acu[0] = RxData_Inv[2];
	  					  TxData_Acu[1] = RxData_Inv[1];
	  					  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Acu, TxData_Acu);
	  					  break;

	  				  case N_ACTUAL:
	  					  TxHeader_Acu.Identifier = ID_n_actual;
	  					  TxHeader_Acu.DataLength = 2;
	  					  TxHeader_Acu.IdType = FDCAN_EXTENDED_ID;
	  					  TxData_Acu[0] = RxData_Inv[2];
	  					  TxData_Acu[1] = RxData_Inv[1];
	  					  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Acu, TxData_Acu);
	  					  break;

	  				  case I_ACTUAL:
	  					  TxHeader_Acu.Identifier = ID_i_actual;
	  					  TxHeader_Acu.DataLength = 2;
	  					  TxHeader_Acu.IdType = FDCAN_EXTENDED_ID;
	  					  TxData_Acu[0] = RxData_Inv[2];
	  					  TxData_Acu[1] = RxData_Inv[1];
	  					  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Acu, TxData_Acu);
	  					  break;
	  				  default:
	  					  break;

	  				  }
	  			  }
	  		  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 16;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 2950;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 6;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 5;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.RxFifo0ElmtsNbr = 32;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 32;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0;
  sFilterConfig.FilterID2 = 0x0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 6;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 10;
  hfdcan2.Init.NominalTimeSeg2 = 5;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 1;
  hfdcan2.Init.RxFifo0ElmtsNbr = 16;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 16;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 16;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0;
  sFilterConfig.FilterID2 = 0x0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 2640- 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000 - 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, START_BUTTON_LED_Pin|RTDS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS18B20_Data_GPIO_Port, DS18B20_Data_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : START_BUTTON_LED_Pin RTDS_Pin */
  GPIO_InitStruct.Pin = START_BUTTON_LED_Pin|RTDS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Data_Pin */
  GPIO_InitStruct.Pin = DS18B20_Data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS18B20_Data_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : START_BUTTON_Pin */
  GPIO_InitStruct.Pin = START_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(START_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PG9 PG10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_FDCAN3;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void print(char uart_buffer[]){
	sprintf(uart_msg, "%s \n\r", uart_buffer);
	HAL_UART_Transmit(&huart2,(uint8_t*)uart_msg,strlen(uart_msg),HAL_MAX_DELAY);

}

void printValue(int value){
	sprintf(uart_msg, "%hu \n\r", value);
	HAL_UART_Transmit(&huart2,(uint8_t*)uart_msg,strlen(uart_msg),HAL_MAX_DELAY);

}

void printHex(uint8_t value){
	sprintf(uart_msg, "%02X \n\r", value);
	HAL_UART_Transmit(&huart2,(uint8_t*)uart_msg,strlen(uart_msg),HAL_MAX_DELAY);
}


void ADC1_Select_SA1 (void){
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}

void ADC1_Select_SA2 (void){
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}

void ADC1_Select_SF (void){
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLE_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}

void ADC2_Select_FR(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}

void ADC2_Select_FL(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}

void ADC2_Select_RL(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}

void ADC2_Select_RR(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retreive Rx messages from RX FIFO1 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader_Acu, RxData_Acu) == HAL_OK)
    {
		switch (RxHeader_Acu.Identifier){
			case 0x20://ID_ack_precarga:
				if(RxData_Acu[0] == 0){
					precarga_inv = 1;
					#if DEBUG
					print("CAN_ACU: Precarga correcta");
					#endif
				}
				break;
		}

    }

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
    }
  }
}

uint8_t setTorque(){
	// Leemos sensores de posición del pedal de acelaración
	ADC1_Select_SA1();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	s1_aceleracion = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	ADC1_Select_SA2();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	s2_aceleracion = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

#if DEBUG
print("Sensor 1: ");
printValue(s1_aceleracion);
print("");
print("Sensor 2: ");
printValue(s2_aceleracion);
print("");
#endif

	// Leemos sensor de freno
	ADC1_Select_SF();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	s_freno = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

#if DEBUG
print("Sensor freno: ");
printValue(s_freno);
#endif

	// Calculamos % torque  en función de la posición de los sensores
	s1_aceleracion_aux = (s1_aceleracion - 500) / (7.48 - 5);
	if(s1_aceleracion_aux < 0) s1_aceleracion_aux = 0;
	s2_aceleracion_aux = (s2_aceleracion - 482) / (6.59 - 4.82);
	if(s2_aceleracion_aux < 0) s2_aceleracion_aux = 0;

#if DEBUG
print("Sensor % 1: ");
printValue(s1_aceleracion_aux);
print("");
print("Sensor % 2: ");
printValue(s2_aceleracion_aux);
print("");
#endif

	// Torque enviado es la media de los dos sensores
	torque_total = (s1_aceleracion_aux + s2_aceleracion_aux) / 2;

	// Por debajo de un 10% no acelera y por encima de un 90% esta a tope
	if (torque_total < 5)
	{
	  torque_total = 0;
	}
	else if (torque_total > 90)
	{
	  torque_total = 100;
	}

    // Comprobamos EV 2.3 APPS/Brake Pedal Plausibility Check
    // En caso de que se esté pisando el freno y mas de un 25% del pedal para. Se resetea
	// solo si el acelerador vuelve por debajo del 5%
    if (s_freno > UMBRAL_FRENO_APPS && torque_total > 25)
    {
      print("EV_2_3");
      flag_EV_2_3 = 1;
    }
    else if (s_freno < UMBRAL_FRENO_APPS && torque_total < 5)
    {
      flag_EV_2_3 = 0;
    }

    // T11.8.9 Implausibility is defined as a deviation of more than ten percentage points
    // pedal travel between any of the used APPSs
    if (abs(s1_aceleracion_aux - s2_aceleracion_aux) > 10)
    {
      count_T11_8_9 = count_T11_8_9 + 1;
      if (count_T11_8_9 * periodo_inv > 100)
      {
    	print("T11.8.9");
        flag_T11_8_9 = 1;
      }
    }
    else
    {
      count_T11_8_9 = 0;
      flag_T11_8_9 = 0;
    }

    if (flag_EV_2_3 || flag_T11_8_9)
    {
      torque_total = 0;
    }

#if DEBUG
print("Torque total solicitado: ");
printValue(torque_total);
#endif

    // Limitación del torque en función de la carga
    /*if (v_celda_min < 3500)
    {
      if (v_celda_min > 2800)
      {
        torque_limitado = torque_total * (1.357 * v_celda_min - 3750) / 1000;
      }
      else
      {
        torque_limitado = torque_total * 0.05;
      }
    }
    else
    {
      torque_limitado = torque_total;
    }*/


#if 0
print("Torque limitado en: ");
printValue(torque_limitado);
#endif

	torque_total = torque_total * 2.4;
    // Invertir todos los bits (complemento a uno)
    uint16_t complement_one = ~torque_total;

    // Sumar 1 para obtener el complemento a dos
    uint16_t torque_real = complement_one + 1;


    // Envío telemetría aceleración y freno
    /*TxHeader_Acu.Identifier = ID_torque_total;
    TxHeader_Acu.DataLength = 2;
    TxData_Acu[0] = ((int)torque_limitado) >> 8;
    TxData_Acu[1] = ((int)torque_limitado);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Acu, TxData_Acu);

    s_freno_aux = s_freno;
    s_freno_aux=(s_freno_aux-121.0)/572.0*100.0;
    TxHeader_Acu.Identifier = ID_s_freno;
   	TxHeader_Acu.DataLength = 2;
   	TxData_Acu[0] = ((int)s_freno_aux) >> 8;
   	TxData_Acu[1] = ((int)s_freno_aux);
   	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_Acu, TxData_Acu);*/

    if(torque_real == 0){
    	torque_real = 0xFF;
    }
#if 1
print("Torque mandado al inversor: ");
printHex(torque_real);
#endif
	return torque_real;
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim == &htim16){
		 // ---------- CONTROL DEL INVERSOR ----------
		if(state == 4){
			TxHeader_Inv.Identifier = 0x362;
			TxHeader_Inv.DataLength = 4;

			real_torque = 0;

			TxData_Inv[0] = 0x0;
			TxData_Inv[1] = 0x0;
			TxData_Inv[2] = real_torque;
			TxData_Inv[3] = 0x0;
			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);
		}
		if(state == 6){
			print("state: torque");

			//Estado TORQUE inversor
			TxHeader_Inv.Identifier = RX_SETPOINT_1;
			TxHeader_Inv.DataLength = 3;
			TxHeader_Inv.IdType = FDCAN_STANDARD_ID;


			TxData_Inv[0] = 0x0;
			TxData_Inv[1] = 0x0;
			TxData_Inv[2] = 0x6;
			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);

			//Request TORQUE inversor
			TxHeader_Inv.Identifier = 0x362;
			TxHeader_Inv.DataLength = 4;

			//real_torque = setTorque();

			TxData_Inv[0] = 0x0;
			TxData_Inv[1] = 0x0;
			TxData_Inv[2] = 0xFF; //torque 0 - 240
			TxData_Inv[3] = 0xFF; //negative torque
			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Inv, TxData_Inv);
		}
		if(state == 10){
			print("state: soft fault");
			printValue(error);
		}


	}
}
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
