/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t estado_rx;
uint8_t estado_spi_tx;
uint8_t rx_buffer[]={0};
uint8_t get_status[]={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void recibir ();

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin, 0);
  HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, 0);
  HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 0);
  HAL_GPIO_WritePin(LED_SPARE_GPIO_Port, LED_SPARE_Pin, 0);
 //  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1); //Estado inicial del pin de selección para la comunicación SPI

   uint8_t comand[]={0};
   uint8_t direction[]={0};

   uint32_t pasos = 200;
   uint8_t fwd = 1;
   uint8_t move = 0x40;

   uint8_t el_pos = 0x00;
   uint8_t t_val = 0b01111100;
   uint8_t t_fast = 0b11111111;
   uint8_t ton_min = 0b11111111;
   uint8_t toff_min = 0b11111111;
   uint8_t ocd_th = 0x08;

   uint8_t step_mode_bit3 = 0x1 << 3;
   uint8_t step_sel = (step_mode_bit3 | 0x00);
   uint8_t sync_sel = 0x00;
   uint8_t step_mode = (sync_sel | \
 		  	  	  	  step_sel);

   uint8_t alarm_en = 0b11111111;

   uint16_t config_osc_sel = 0x0000;
   uint16_t config_oc_sd = 0x0080; //Para el motor ante una detección de sobre corriente. 0x0000 para deshabilitarlo.
   uint16_t config_pow_sr = 0x0000;
   uint16_t config_tq_reg = 0x0000;
   uint16_t config_toff = 0x007F;
   uint16_t config_reg = (config_osc_sel | \
 		  	  	  	  	  config_oc_sd | \
 						  config_pow_sr | \
 						  config_tq_reg | \
 						  config_toff);

   uint8_t estado_spi_tx;


   //Configuraciones de los modos básicos en los registros esenciales:
   //Posición electrica:
   direction[3] = 0x02;
   comand [2] = el_pos;
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
   estado_spi_tx = HAL_SPI_Transmit(&hspi1, (uint8_t*) direction, 1, 100);
   HAL_SPI_Transmit(&hspi1, (uint8_t*) comand, 3, 100);
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
   HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 1);
   recibir ();
   estado_rx = HAL_UART_Transmit(&huart1, rx_buffer, 1, 100);
   sprintf (rx_buffer, "Valor recibido: %u,\n\r", estado_rx);

   //Valor de regulación de corriente de torque
   direction[3] = 0x09;
   comand [2] = t_val;
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
   estado_spi_tx = HAL_SPI_Transmit(&hspi1, (uint8_t*) direction, 1, 100);
   HAL_SPI_Transmit(&hspi1, (uint8_t*) comand, 3, 100);
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
   HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 1);
   recibir ();
   estado_rx = HAL_UART_Transmit(&huart1, rx_buffer, 1, 100);
   sprintf (rx_buffer, "Valor recibido: %u,\n\r", estado_rx);

   //Maximum fast decay time (TOFF_FAST) and the maximum fall step time (FALL_STEP) used by the current control system
   direction[3] = 0x0E;
   comand [2] = t_fast;
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
   estado_spi_tx = HAL_SPI_Transmit(&hspi1, (uint8_t*) direction, 1, 100);
   HAL_SPI_Transmit(&hspi1, (uint8_t*) comand, 3, 100);
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
   HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 1);
   recibir ();
   estado_rx = HAL_UART_Transmit(&huart1, rx_buffer, 1, 100);
   sprintf (rx_buffer, "Valor recibido: %u,\n\r", estado_rx);

   //Mínimo tiempo encendido, en ambos casos se configura con el máximo permitido.
   direction[3] = 0x0F;
   comand [2] = ton_min;
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
   estado_spi_tx = HAL_SPI_Transmit(&hspi1, (uint8_t*) direction, 1, 100);
   HAL_SPI_Transmit(&hspi1, (uint8_t*) comand, 3, 100);
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
   HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 1);
   recibir ();
   estado_rx = HAL_UART_Transmit(&huart1, rx_buffer, 1, 100);
   sprintf (rx_buffer, "Valor recibido: %u,\n\r", estado_rx);

   //Mínimo tiempo apagado
   direction[3] = 0x10;
   comand [2] = toff_min;
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
   estado_spi_tx = HAL_SPI_Transmit(&hspi1, (uint8_t*) direction, 1, 100);
   HAL_SPI_Transmit(&hspi1, (uint8_t*) comand, 3, 100);
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
   HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 1);
   recibir ();
   estado_rx = HAL_UART_Transmit(&huart1, rx_buffer, 1, 100);
   sprintf (rx_buffer, "Valor recibido: %u,\n\r", estado_rx);

   //Valor de umbral de sobre corriente
   direction[3] = 0x13;
   comand [2] = ocd_th;
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
   estado_spi_tx = HAL_SPI_Transmit(&hspi1, (uint8_t*) direction, 1, 100);
   HAL_SPI_Transmit(&hspi1, (uint8_t*) comand, 3, 100);
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
   HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 1);
   recibir ();
   estado_rx = HAL_UART_Transmit(&huart1, rx_buffer, 1, 100);
   sprintf (rx_buffer, "Valor recibido: %u,\n\r", estado_rx);

   // Configuración del modo de paso
   direction[3] = 0x18;
   comand [2] = config_reg;
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
   estado_spi_tx = HAL_SPI_Transmit(&hspi1, (uint8_t*) direction, 1, 100);
   HAL_SPI_Transmit(&hspi1, (uint8_t*) comand, 3, 100);
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
   HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 1);
   recibir ();
   estado_rx = HAL_UART_Transmit(&huart1, rx_buffer, 1, 100);
   sprintf (rx_buffer, "Valor recibido: %u,\n\r", estado_rx);

   //Habilitación de alarmas
   direction[3] = 0x17;
   comand [2] = alarm_en;
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
   estado_spi_tx = HAL_SPI_Transmit(&hspi1, (uint8_t*) direction, 1, 100);
   HAL_SPI_Transmit(&hspi1, (uint8_t*) comand, 3, 100);
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
   HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 1);
   recibir ();
   estado_rx = HAL_UART_Transmit(&huart1, rx_buffer, 1, 100);
   sprintf (rx_buffer, "Valor recibido: %u,\n\r", estado_rx);

   //Configuraciones del registro CONFIG
   direction[3] = 0x16;
   comand [2] = step_mode;
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
   estado_spi_tx = HAL_SPI_Transmit(&hspi1, (uint8_t*) direction, 1, 100);
   HAL_SPI_Transmit(&hspi1, (uint8_t*) comand, 3, 100);
   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
   HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 1);
   recibir ();
   estado_rx = HAL_UART_Transmit(&huart1, rx_buffer, 1, 100);
   sprintf (rx_buffer, "Valor recibido: %u,\n\r", estado_rx);


   while (1)
   {
 	  HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin, 0);
 	  HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, 0);
 	  HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 0);
 	  HAL_GPIO_WritePin(LED_SPARE_GPIO_Port, LED_SPARE_Pin, 0);

 	  direction[3] = (move | fwd);
 	  comand [2] = pasos >> 16;
 	  comand [1] = pasos >> 8;
 	  comand [0] = pasos;

 	  // Mover motor hacia adelante por 5s
 	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
 	  //Se envía la dirección o registro a escribirse
 	  estado_spi_tx = HAL_SPI_Transmit(&hspi1, (uint8_t*) direction, 1, 100);
 	  //Encender LED para depurar
 	  if (estado_spi_tx==HAL_OK) HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin, 1);
 	  else HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 1);
 	  //Se envía el comando
 	  estado_spi_tx = HAL_SPI_Transmit(&hspi1, (uint8_t*) comand, 3, 100);
 	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
 	  if (estado_spi_tx==HAL_OK) HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin, 1);
 	  else HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 1);

 	  recibir ();
 	  estado_rx = HAL_UART_Transmit(&huart1, rx_buffer, 1, 100);
 	  sprintf (rx_buffer, "Valor recibido: %u,\n\r", estado_rx);


 	  HAL_Delay (5000);
 	  HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 0);
 	  HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin, 0);
 	  HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, 0);
 	  HAL_GPIO_WritePin(LED_SPARE_GPIO_Port, LED_SPARE_Pin, 0);

 	  //Detener motor
 	  comand [0] = 0xB0;
 	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
 	  estado_spi_tx= HAL_SPI_Transmit(&hspi1, (uint8_t*) comand, 3, 100);
 	  if (estado_spi_tx==HAL_OK) HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin, 1);
 	  else HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 1);
 	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);

 	  recibir ();
 	  estado_rx = HAL_UART_Transmit(&huart1, rx_buffer, 1, 100);
 	  sprintf (rx_buffer, "Valor recibido: %u,\n\r", estado_rx);

 	  if (estado_rx==HAL_OK) HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, 1);
 	  else HAL_GPIO_WritePin(LED_SPARE_GPIO_Port, LED_SPARE_Pin, 1);

 	  HAL_Delay (1000);
 	  HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 0);
 	  HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin, 0);
 	  HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, 0);
 	  HAL_GPIO_WritePin(LED_SPARE_GPIO_Port, LED_SPARE_Pin, 0);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
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

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_SPARE_Pin|LED_ERROR_Pin|LED_BUSY_Pin|LED_READY_Pin
                          |SW_Pin|STBY_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STCK_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FLAG_Pin|BUSY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_SPARE_Pin LED_ERROR_Pin LED_BUSY_Pin LED_READY_Pin
                           SW_Pin STBY_RESET_Pin */
  GPIO_InitStruct.Pin = LED_SPARE_Pin|LED_ERROR_Pin|LED_BUSY_Pin|LED_READY_Pin
                          |SW_Pin|STBY_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT_Pin RIGHT_Pin */
  GPIO_InitStruct.Pin = LEFT_Pin|RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STCK_Pin CS_Pin */
  GPIO_InitStruct.Pin = STCK_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FLAG_Pin BUSY_Pin */
  GPIO_InitStruct.Pin = FLAG_Pin|BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void recibir ()
{
	get_status[0] = 0b11010000;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
	estado_spi_tx = HAL_SPI_Transmit(&hspi1, (uint8_t*) get_status, 1, 100);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
	//Escuchar respuesta del driver:
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
	estado_rx = HAL_SPI_Receive(&hspi1, rx_buffer, 1, 1000);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
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
