/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N_points 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

//__IO uint16_t ADC_number = 0;
__IO static uint16_t ADC_res;
__IO static uint16_t ADC_array[N_points] = {0,};
__IO static uint16_t DAC_array[N_points] = {0,};
__IO static uint8_t  Receive_array[255] = {0,};
__IO static uint8_t  Transmit_array[N_points] = {0,};

__IO uint8_t  _timestep = 1; // (step in us)*2-1, if only not 0.5 (in that case == 0)
__IO uint16_t _time_on = 500; // between 0 and 1000
__IO uint16_t _cur_max = 2000; // max ~ 3.3V; 2V = 2480 = 0x9B0
__IO uint16_t _cur_min = 000; // can't put less than 30 mV = 37 = 0x25
__IO uint8_t  _avgn = 1; // It is unused now
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void prog_init(void);
void loop(void);
void blink(uint8_t num_blink);
uint16_t send_result(uint16_t res);
uint16_t read_ADC_values(uint16_t N_of);
uint8_t send_back(void);
uint8_t measure(void);
uint8_t try_to_read(void);
volatile uint16_t get_num(char *num_target_array, uint8_t str_len);
volatile uint16_t z_func(char *target_array);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void prog_init(void) {
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);

  BSP_LED_On(LED3);
  HAL_Delay(50);
  BSP_LED_Off(LED3);
  HAL_Delay(50);
}

void loop() {
//  blink(1);
  try_to_read();

//  HAL_Delay(1);  
}

void blink(uint8_t num_blink) {
  for (int i = 0; i<num_blink; i++) {
    HAL_Delay(125);
    BSP_LED_On(LED2);
    HAL_Delay(125);
    BSP_LED_Off(LED2);
  }
  BSP_LED_Off(LED1);
  BSP_LED_Off(LED3);
}

uint8_t try_to_read(void) {
  Receive_array[0] = 0;
  HAL_UART_Receive(&huart3, (uint8_t*) Receive_array, 255, 50);
  HAL_Delay(1);

/*
  if ((nekoneko[0] > 2)&&(nekoneko[0] < 40)) {
    //HAL_UART_Transmit(&huart3, nekoneko, 255, 50);
  }
  if (nekoneko[0] == 1) {
//    _timestep = 0;
//    measure();
//    send_back();
  }
  if (nekoneko[0] == 2) {
//    _timestep = 1;
//    measure();
//    send_back();
  }
  if (nekoneko[0] == 3) {
//    _timestep = 3;
//    measure();
//    send_back();
  }
*/
  if (Receive_array[0] > 3) {
    z_func((char*) Receive_array);
    for (int i = 0; i<255; i++) {
      Receive_array[i] = 0;
    }
  }
//  measure();
  return Receive_array[0];
}

uint8_t measure(void) {
  uint16_t i;

  MX_TIM2_Init();

  for (i = 0; i<_time_on; i++) {
    DAC_array[i] = _cur_max;
  }
  for (i = _time_on; i<N_points; i++) {
    DAC_array[i] = _cur_min;
  }

  for (i = 0; i<N_points; i++) {
    ADC_array[i] = 0;
    Transmit_array[i] = 0xff;
  }

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

  read_ADC_values(N_points);

//  HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
//  HAL_ADC_Stop(&hadc1);

  for (i=0; i<N_points; i++) {
    Transmit_array[i] = ADC_array[i] >> 1;
//    Transmit_array[i*2+1] = ADC_array[i] & 0xFF;
  }

  return 0;
}

uint8_t send_back(void) {
  HAL_UART_Transmit(&huart3, (uint8_t*) Transmit_array, N_points, 0xFF);
  return 0;
}

uint16_t read_ADC_values(uint16_t N_of) {
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_array, N_points);
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)&DAC_array, N_points, DAC_ALIGN_12B_R);

  HAL_Delay(1);
  HAL_TIM_Base_Start(&htim2);
  HAL_Delay(2*_timestep);
  HAL_TIM_Base_Stop(&htim2);

  return N_of;
}

volatile uint16_t z_func(char *target_array) {
  uint8_t num_flag = 0;
  uint16_t func_num = 0;
  char z1[] = "step";
  char z2[] = "imp";
  char z3[] = "start";
  char z4[] = "loadb";
  char z7[] = "cur";
  char back[] = "back";
  char avgn[] = "avgn";
  char def[] = "def";

  if (strstr(target_array, z1) != NULL) {
    func_num = get_num(target_array, 4);
    if (func_num > 0) func_num = (func_num*2-1);
    _timestep = func_num;
//    printf("step %d\n", func_num);
    num_flag = 1;
    HAL_UART_Transmit(&huart3, (uint8_t*)"s>\n", 3, 0xFF);
  }
  if (strstr(target_array, z2) != NULL) {
    func_num = get_num(target_array, 3);
    _time_on = func_num*10;
//    printf("imp %d\n", func_num);
    num_flag = 1;
  }
  if (strstr(target_array, z3) != NULL) {
    measure();
//    printf("start\n");
    num_flag = 0;
  }
  if (strstr(target_array, z4) != NULL) {
    send_back();
//    printf("loadb\n");
    num_flag = 0;
  }
  if (strstr(target_array, z7) != NULL) {
    func_num = get_num(target_array, 3);
    _cur_max = func_num*(2*40.96/3.3);
//    printf("cur %d\n", func_num);
    num_flag = 1;
  }
  if (strstr(target_array, back) != NULL) {
    func_num = get_num(target_array, 4);
    _cur_min = func_num*(2*40.96/3.3);
//    printf("cur %d\n", func_num);
    num_flag = 1;
  }
  if (strstr(target_array, avgn) != NULL) {
    func_num = get_num(target_array, 4);
    _avgn = func_num;
//    printf("cur %d\n", func_num);
    num_flag = 1;
  }
  if (strstr(target_array, def) != NULL) {
    _timestep = 1;
    _cur_max = 12.5*(2*40.96/3.3);
    _time_on = 500;
//    printf("def\n");
    num_flag = 0;
  }

  if (num_flag == 0) {
    return 0;
  } else {
    return func_num;
  }
}

volatile uint16_t get_num(char *num_target_array, uint8_t str_len) {
  uint8_t trr = strlen(num_target_array);
  uint8_t array_end = trr;
  char only_num[255] = "";
  uint16_t num_in_func;

  for (int i=trr-1; i>0; i--) {
    if ((num_target_array[i] == 0x0D)||(num_target_array[i] == 0x0A)) (array_end = i);
  }

  if (strstr(num_target_array, "step0.5") != NULL) {
    num_in_func = 0;
  } else {
    for (int i = str_len; i < array_end; i++) {
      only_num[i-str_len] = num_target_array[i];
    }
    num_in_func = atoi(only_num);
  }
  return num_in_func;
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

  prog_init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  measure();
//  read_ADC_values(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    loop();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  // Input for ADC_Channel_10 is PC0
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /*
  Try next to increase ADC speed:
  static void MX_ADC1_Init(uint8_t timestep)
  
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  or add nest strings instead auto "preacaler" string
  */

  if (_timestep == 0) {
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  } else {
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  }

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
//  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
//  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */
  // Output for DAC_channel_1 is PA4
  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
//  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */
  /* USER CODE END DAC_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 95;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  MX_ADC1_Init();
  // also put it instead auto "prescaler" string
  htim2.Init.Prescaler = _timestep;
  /* USER CODE END TIM2_Init 1 */

  htim2.Instance = TIM2;
//  htim2.Init.Prescaler = timestep;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 47;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  // if we need to connect via Unix-based system
  // stty 115200 -F /dev/ttyACM0 -raw
  // cat /dev/ttyACM0 | hexdump -C
  // echo $'\x01' > /dev/ttyACM0

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
//  huart3.Init.BaudRate = 115200;
  huart3.Init.BaudRate = 57600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint16_t send_result(uint16_t res) {
  uint8_t c[] = {res>>8, res & 0xFF};
  HAL_UART_Transmit(&huart3, c, 2, 0xFF);
  return res;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
  /* Turn LED1 on: Transfer process is correct */
//  DAC_val = 0;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
//  ADC_array[ADC_number] = HAL_ADC_GetValue(&hadc);
//  ADC_number ++;
//  BSP_LED_Off(LED1);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
