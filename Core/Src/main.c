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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PACKET_SIZE 64
#define BUFFER_SIZE 256
#define DISP_DELAY  100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

static char buffer[ BUFFER_SIZE ];

static volatile uint32_t systick_ms   = 0;
static volatile uint32_t freq         = 0; /* 32bit = approx. 4.3G ticks per second. */
static volatile uint32_t freq_scratch = 0; /* scratch pad. */
static volatile uint8_t hold          = 0;
static uint8_t mco_current = 0; // Default to off.

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void usbcdc_printf( const char *fmt, uint32_t arg );
void usbcdc_clear_screen(void);
void tim2_interrupt(void);
void sys_tick_interrupt(void);
void usbcdc_printfreq( uint32_t frequency, uint8_t hold, uint8_t mco_val );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // Wait 500ms for USB setup to complete before trying to send anything.
  HAL_Delay( 500 );

  __TIM2_CLK_ENABLE();
  HAL_TIM_Base_Start_IT( &htim2 );
//  HAL_TIM_Base_Start( &htim2 );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // simple LED blinking
//	  CDC_Transmit_FS( "X\r\n", 3 ); // USB test
//	  HAL_Delay( 1000 );

	  usbcdc_clear_screen();
	  usbcdc_printfreq( freq, hold, mco_current );
	  HAL_Delay( DISP_DELAY );
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_NOCLOCK, RCC_MCODIV_1);
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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ETRF;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// =============================================================================
void poll_usb_command( uint8_t one_num ) {
	switch( one_num ) {
	case 'A' :
	case 'a' : HAL_GPIO_WritePin( GPIOC, GPIO_PIN_13, GPIO_PIN_SET );
	           break;
	case 'D' :
	case 'd' : HAL_GPIO_WritePin( GPIOC, GPIO_PIN_13, GPIO_PIN_RESET );
	           break;
	case 'T' :
	case 't' : HAL_GPIO_TogglePin( GPIOC, GPIO_PIN_13 );
	           break;
	case 'C' :
	case 'c' : usbcdc_clear_screen();
	           break;
	case 'M' :
	case 'm' : if( ++mco_current > 3 ) mco_current = 0;
		       set_mco();
	           break;
	case 'H' :
	case 'h' : hold = hold ? 0 : 1;
	           break;
	}
}

void usbcdc_clear_screen(void) {
	CDC_Transmit_FS( "\033c\r", 3 );
}

void usbcdc_printfreq( uint32_t frequency, uint8_t hold, uint8_t mco_val ) {
	  uint16_t len1;
	  uint8_t len2;
	  uint8_t loop;
	  char *end_text1 = " MHz\r\n\r\n";
	  char *end_text2 = " MHz HOLD\r\n\r\n";
	  char *mco_text = "Clock output: ";
	  const uint8_t MAX_BUF_SIZE = 20;
	  char buffer1[ MAX_BUF_SIZE ];

	  len1 = snprintf( buffer1, MAX_BUF_SIZE, "%4lu.", frequency / 1000000 );
	  for( loop = 0; loop < len1; loop++ ) {
		  buffer[ loop ] = buffer1[ loop ];
	  }
	  len2 = snprintf( buffer1, MAX_BUF_SIZE, "%06lu", frequency % 1000000 );
	  for( loop = 0; loop < len2; loop++ ) {
		  buffer[ len1 + loop ] = buffer1[ loop ];
	  }
	  len1 += len2;
	  if( hold ) {
		  len2 = strlen( end_text2 );
		  for( loop = 0; loop < len2; loop++ ) {
			  buffer[ len1 + loop ] = end_text2[ loop ];
		  }
	  }
	  else {
		  len2 = strlen( end_text1 );
		  for( loop = 0; loop < len2; loop++ ) {
			  buffer[ len1 + loop ] = end_text1[ loop ];
		  }
	  }
	  len1 += len2;
	  len2 = strlen( mco_text );
	  for( loop = 0; loop < len2; loop++ ) {
		  buffer[ len1 + loop ] = mco_text[ loop ];
	  }
	  len1 += len2;
	  buffer[ len1++ ] = 0;
	  switch( mco_val ) {
	  	  case 0 : strcat( buffer, "OFF\r\n");
				   len1 += 5;
				   break;
	  	  case 1 : strcat( buffer, "PLL/2 = 36MHz\r\n");
				   len1 += 15;
				   break;
	  	  case 2 : strcat( buffer, "HSI = 8MHz\r\n");
				   len1 += 12;
				   break;
	  	  case 3 : strcat( buffer, "HSE = 8MHz\r\n");
				   len1 += 12;
				   break;
	  }
	  CDC_Transmit_FS( buffer, len1);
}

void set_mco() {
	switch( mco_current ) {
	case 0 : HAL_RCC_MCOConfig( RCC_MCO, RCC_MCO1SOURCE_NOCLOCK, RCC_MCODIV_1 ); // no MCO
	         break;
	case 1 : HAL_RCC_MCOConfig( RCC_MCO, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1 ); // PLLCLK / 2 = 36MHz
	         break;
	case 2 : HAL_RCC_MCOConfig( RCC_MCO, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1 ); // HSI ~8MHz
	         break;
	case 3 : HAL_RCC_MCOConfig( RCC_MCO, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1 ); // HSE =8MHz
	         break;
	default : HAL_RCC_MCOConfig( RCC_MCO, RCC_MCO1SOURCE_NOCLOCK, RCC_MCODIV_1 );
	          mco_current = 0;
	         break;
	}
}

// =============================================================================

void tim2_interrupt(void) {
    freq_scratch += 65536; /* TIM2 is 16-bit and overflows every 65536 events. */
}

void sys_tick_interrupt(void) {
  systick_ms++;

  if (systick_ms % 1000 == 0) {
    /* Scratch pad to finalized result */
    if( ! hold ) {
      freq = freq_scratch + __HAL_TIM_GetCounter( &htim2 );
      freq *= 1.000004;
    }
    // Reset the counter. This will generate one extra overflow for next measurement.
    // In case of nothing got counted, manually generate a reset to keep consistency.
    __HAL_TIM_SetCounter( &htim2, 1);
    __HAL_TIM_SetCounter( &htim2, 0);
    freq_scratch = 0;
    HAL_GPIO_TogglePin( GPIOC, GPIO_PIN_13 );
  }
}


// =============================================================================
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
