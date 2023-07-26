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

uint8_t param[4]={0};
uint8_t value[3]={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void recibir ();
void enviar (uint8_t* param, uint8_t* value, uint16_t nbp, uint16_t nbv);
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
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1); //Estado inicial del pin de selección para la comunicación SPI



   uint32_t pasos = 60000;
   uint8_t fwd = 1;
   //uint8_t move = 0x64;

   double acc = 2008, dec = 2008, max_speed = 992, fs_spd = 15625;
   uint8_t min_speed = 0, tboost = 0, tcc = 0b00000101, tdt = 1;
   uint8_t tval_hold = 0x00, tval_run = 0x00, tval_acc = 0x00, tval_dec = 0x00; //Regulación de torque configurada en todos los casos a 7.8mV
   uint8_t t_fast = 5;

   uint8_t ton_min = 3;
   uint8_t toff_min = 21;
   uint8_t ocd_th = 0x08;

   uint8_t step_sel = 0b00000111;
   uint8_t sync_sel = 0b00000101<<4;
   uint8_t step_mode = (0b10000000|sync_sel|0x08|step_sel);
   uint8_t el_pos = (step_mode | 0x00);

   uint8_t overcurrent=0x01, shutdown=0x02, th_warning=0x04, uvlo=0x08, adc_uvlo=0x10, turn_on=0x40, w_nperf_cmd=0x80;

   //Se pueden escoger que alarmas se activan o no, acá se activan todas.
   uint8_t alarm_en = (overcurrent | shutdown | th_warning | uvlo | adc_uvlo | turn_on | w_nperf_cmd);

   uint8_t wd_en = 0;
   uint8_t igate = 0x00;
   uint16_t gatecfg1 = (tcc|igate<<5|tboost<<8|wd_en<<11);
   uint8_t gatecfg2 = (tdt|0b00000001<<5);


   uint16_t osc_sel = 0x0000;
   uint16_t ext_clk = 0b00000000;
   uint16_t sw_mode = 0b00010000;
   uint16_t oc_sd = 0x0000; //Para el motor ante una detección de sobre corriente. 0x0000 para deshabilitarlo.
   uint16_t vcc_val= 0x0300;
   uint16_t uvloval = 0x0100;
   uint16_t tq_reg = 0x0000;
   uint16_t tsw = 0x02<<10;
   uint16_t pred_en = 0b1000000000000000;
   uint16_t config = (osc_sel | \
 		  	  	  	  	  ext_clk | \
 						  sw_mode | \
 						  vcc_val | \
 						  oc_sd | uvloval | tq_reg | tsw | pred_en);


  // uint8_t estado_spi_tx;


   //Configuraciones de los modos básicos en los registros:
   //Posición electrica:
   param[0] = (0x00|0x02);
   value[0] = el_pos;
   enviar (param, value, 1, 1);

   //Aceleración
   param[0] = 0x05;
   value[0] = acc;
   enviar (param, value, 1, 2);

   //Desaceleración
   param[0] = 0x06;
   value[0] = dec;
   enviar (param, value, 1, 2);

   //Velocidad mínima
   param[0] = 0x07;
   value[0] = min_speed;
   enviar (param, value, 1, 1);

   //Vel máxima
   param[0] = 0x08;
   value[0] = max_speed;
   enviar (param, value, 1, 2);

   //Full-step speed
   param[0] = 0x15;
   value[0] = fs_spd;
   enviar (param, value, 1, 2);

   //T_VALs
   param[0] = 0x09;
   value[0] = tval_hold;
   enviar (param, value, 1, 1);

   param[0] = 0x0A;
   value[0] = tval_run;
   enviar (param, value, 1, 1);

   param[0] = 0x0B;
   value[0] = tval_acc;
   enviar (param, value, 1, 1);

   param[0] = 0x0C;
   value[0] = tval_dec;
   enviar (param, value, 1, 1);

   //Maximum fast decay time (TOFF_FAST) and the maximum fall step time (FALL_STEP) used by the current control system
   param[0] = 0x0E;
   value[0] = t_fast;
   enviar (param, value, 1, 1);

   //Mínimo tiempo encendido, en ambos casos se configura con el máximo permitido.
   param[0] = 0x0F;
   value[0] = ton_min;
   enviar (param, value, 1, 1);

   //Mínimo tiempo apagado
   param[0] = 0x10;
   value[0] = toff_min;
   enviar (param, value, 1, 1);

   //Valor de umbral de sobre corriente
   param[0] = 0x13;
   value[0] = ocd_th;
   enviar (param, value, 1, 1);

   //Step mode
    param[0] = 0x16;
    value[0] = step_mode;
    enviar (param, value, 1, 1);

    //Habilitación de alarmas
    param[0] = 0x17;
    value[0] = alarm_en;
    enviar (param, value, 1, 1);

   // GATECFG1
   param[0] = 0x18;
   value[0] = gatecfg1;
   enviar (param, value, 1, 2);

   // GATECFG2
   param[0] = 0x19;
   value[0] = gatecfg2;
   enviar (param, value, 1, 1);

   // Config
   param[0] = 0x1A;
   value[0] = config;
   enviar (param, value, 1, 2);




   while (1)
   {

	  uint8_t direction[1] = {0};
	  uint8_t speed[3] = {0};

	  direction [0] = (0x68 | fwd);
 	  speed [0] = pasos >> 16;
 	  speed [1] = pasos >> 8;
 	  speed [2] = pasos;

 	  enviar (direction, speed, 1, 3);
// 	  // Mover motor hacia adelante por 5s
// 	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
// 	  //Se envía la dirección: 1 FW, 0 BW
// 	  HAL_SPI_Transmit(&hspi1, (uint8_t*) direction, 1, 500);
// 	  //Se envía el comando
// 	 // while(1)
// 	 // {
// 	  HAL_SPI_Transmit(&hspi1, (uint8_t*) speed, 3, 500);
// 	 // }
// 	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);

// 	  HAL_Delay (3000);

 	 /* //Detener motor
 	  comand [0] = 0xB0;
 	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
 	  estado_spi_tx= HAL_SPI_Transmit(&hspi1, (uint8_t*) comand, 1, 500);
 	  if (estado_spi_tx==HAL_OK) HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin, 1);
 	  else HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 1);
 	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);

 	  recibir ();
 	  estado_rx = HAL_UART_Transmit(&huart1, rx_buffer, 3, 500);
 	 // sprintf (rx_buffer, "Valor recibido: %u,\n\r", estado_rx);
 	  if (estado_rx==HAL_OK) HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, 1);
 	  else HAL_GPIO_WritePin(LED_SPARE_GPIO_Port, LED_SPARE_Pin, 1);
 	  HAL_Delay (1000);

 	  HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 0);
 	  HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin, 0);
 	  HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, 0);
 	  HAL_GPIO_WritePin(LED_SPARE_GPIO_Port, LED_SPARE_Pin, 0);
 	  HAL_Delay (1000);*/
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
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
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

void enviar (uint8_t* param, uint8_t* value, uint16_t nbp, uint16_t nbv)
{
	   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
	   HAL_SPI_Transmit(&hspi1, (uint8_t*) param, nbp, 500);
	   HAL_SPI_Transmit(&hspi1, (uint8_t*) value, nbv, 500);
	   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
}
void recibir ()
{
	get_status[0] = 0b11010000;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
	estado_spi_tx = HAL_SPI_Transmit(&hspi1, (uint8_t*) get_status, 1, 500);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
	//Escuchar respuesta del driver:
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
	estado_rx = HAL_SPI_Receive(&hspi1, rx_buffer, 3, 500);
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
