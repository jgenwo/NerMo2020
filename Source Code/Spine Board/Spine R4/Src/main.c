/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <mouseSpine.h>

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

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
long ledCounter = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void setLEDcounter(long l) {
	ledCounter = l;
}

// ***********************************************************************************************************
//#define DELAY_1US	  ((unsigned long) 140)				// timing for RELEASE
//#define DELAY_1MS	  ((unsigned long) 108823)			// timing for RELEASE

#define DELAY_1US	  ((unsigned long) 189)				// timing for RELEASE
#define DELAY_1MS	  ((unsigned long) 146899)			// timing for RELEASE

void sleepUS(unsigned long delayTimeUS) {
	unsigned long m, n;
	for (n = 0; n < delayTimeUS; n++) {
		for (m = DELAY_1US; m; m--) {
			asm volatile ("nop");
		}
	}
}

void sleepMS(unsigned long delayTimeMS) {
	unsigned long m, n;
	for (n = 0; n < delayTimeMS; n++) {
		for (m = DELAY_1MS; m; m--) {
			asm volatile ("nop");
		}
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	// **************************************************************************************************	IWDG (independent watch-dog timer)
	if ((RCC->CSR) & BIT(29)) {		// are we waking up from a WDT reset?
		RCC->CSR |= BIT(24);// clear reset flags (memory of reset source) (so next time we're not ending here)
		enterBootLoader();// start boot loader, so we can update software if needed!
	}

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	SCB_InvalidateICache();
	SCB_DisableICache();				// disable instruction cache
	SCB_InvalidateDCache();
	SCB_DisableDCache();				// disable data cache

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART3_UART_Init();
	MX_ADC1_Init();
	MX_UART4_Init();
	MX_UART5_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM5_Init();
	/* USER CODE BEGIN 2 */

	// ***********************************************************************************************************************
	GPIO_InitTypeDef pinDefML;
	pinDefML.Pin = (PORTC_ENABLE_RPIZW_POWER);		// enable the Vreg for RPIZW
	pinDefML.Mode = GPIO_MODE_OUTPUT_PP;
	pinDefML.Pull = GPIO_PULLUP;
	pinDefML.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOC, &pinDefML);
	PORTC->SET = PORTC_ENABLE_RPIZW_POWER;

	pinDefML.Pin = (PORTB_ENABLE_SERVOS_POWER);	// enable the Vreg for Servos
	pinDefML.Mode = GPIO_MODE_OUTPUT_PP;
	pinDefML.Pull = GPIO_PULLUP;
	pinDefML.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &pinDefML);
	PORTB->SET = PORTB_ENABLE_SERVOS_POWER;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	uartShowVersion();

	long UART_BytesAvailable;
	char newChar;

	systemTimeHalfMS = 0;		// set System Time (Timer 5: TIM5->CNT) to zero
	HAL_TIM_Base_Start(&htim5);

	I2C1->CR1 |= I2C_CR1_PE;				// enable I2C
	I2C1->CR2 &= ~(BIT(11));				// set 7 bit addressing mode


	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// ***********************************************************************************************************************
		if (ledCounter) {										// LED blinking
			if ((--ledCounter) == 0) {
				ledCounter = 200000;
				if ((PORTA->INP) & PORTA_LED_A) {
					PORTA->CLR = PORTA_LED_A;
				} else {
					PORTA->SET = PORTA_LED_A;
				}
			}
		}

		// ***********************************************************************************************************************
		UART_BytesAvailable = (-((DMA1_Stream1->NDTR) + uart3RXReadPointerDMA))
				& (UART_BUFFER_MASK);		// U3 - RasPIZW UART receive (UART3)
		while (UART_BytesAvailable) {
			newChar = uart3RXBufferDMA[uart3RXReadPointerDMA++];
			uart3RXReadPointerDMA &= UART_BUFFER_MASK;
			UART_BytesAvailable--;

			if ((newChar == '\n') || (newChar == '\r')) {
				uart3RXBuffer[uart3RXWritePointer++] = '\n';
				uart3RXBuffer[uart3RXWritePointer] = 0;
				uartSendChar('\n');
				uartParseCommand(uart3RXBuffer);
				uart3RXWritePointer = 0;
			} else {

				if (newChar == 8) {								// BACKSPACE
					if (uart3RXWritePointer) {
						uart3RXWritePointer--;
						uartSendChar(8);
					}
				} else {

					if (uart3RXWritePointer < (UART_BUFFER_SIZE - 2)) {
						uart3RXBuffer[uart3RXWritePointer++] = newChar;
					}
					uartSendChar(newChar);
				}
			}
		}

		// ***********************************************************************************************************************
		if (uart3TXWritePointer != uart3TXReadPointer) {// RPIZW UART transmit
			if ((USART3->ISR) & BIT(7)) {
				USART3->TDR = uart3TXBuffer[uart3TXReadPointer++];
				uart3TXReadPointer &= UART_BUFFER_MASK;
			}
		}

		// ***********************************************************************************************************************	U2 to Servo Chain Spine (UART 2)
		UART_BytesAvailable = (-((DMA1_Stream5->NDTR) + uartSRXReadPointerDMA))
				& (UART_BUFFER_MASK);

		while (UART_BytesAvailable) {
			newChar = uartSRXBufferDMA[uartSRXReadPointerDMA++];
			uartSRXReadPointerDMA &= UART_BUFFER_MASK;
			UART_BytesAvailable--;

			if (uartSRXDiscardCounter) {// is this one of the chars we sent out (ignore the echo because of one-wire transmission)
				uartSRXDiscardCounter--;
			} else {

				if (uartSRXWritePointer < (UART_BUFFER_SIZE - 2))
					uartSRXBuffer[uartSRXWritePointer++] = newChar;
				if ((newChar == '\n') || (newChar == '\r')) {
					uartSRXBuffer[uartSRXWritePointer] = 0;
					uartSendString("S:");
					uartSendString(uartSRXBuffer);
					uartSRXWritePointer = 0;
				}

			}
		}

		if (uartSTXWritePointer != uartSTXReadPointer) {// ***************************************************************
			if ((USART2->ISR) & BIT(7)) {
				USART2->TDR = uartSTXBuffer[uartSTXReadPointer++];
				uartSTXReadPointer &= UART_BUFFER_MASK;
#ifdef UART_SINGLE_WIRE
				uartSRXDiscardCounter++;
#endif
			}
		}

		// ***********************************************************************************************************************	U4 to Servo Chain Right (UART 4)
		UART_BytesAvailable = (-((DMA1_Stream2->NDTR) + uartRRXReadPointerDMA))
				& (UART_BUFFER_MASK);

		while (UART_BytesAvailable) {
			newChar = uartRRXBufferDMA[uartRRXReadPointerDMA++];
			uartRRXReadPointerDMA &= UART_BUFFER_MASK;
			UART_BytesAvailable--;

			if (uartRRXDiscardCounter) {// is this one of the chars we sent out (ignore the echo because of one-wire transmission)
				uartRRXDiscardCounter--;
			} else {

				if (uartRRXWritePointer < (UART_BUFFER_SIZE - 2))
					uartRRXBuffer[uartRRXWritePointer++] = newChar;
				if ((newChar == '\n') || (newChar == '\r')) {
					uartRRXBuffer[uartRRXWritePointer] = 0;
					uartSendString("R:");
					uartSendString(uartRRXBuffer);
					uartRRXWritePointer = 0;
				}

			}
		}

		if (uartRTXWritePointer != uartRTXReadPointer) {// ***************************************************************
			if ((UART4->ISR) & BIT(7)) {
				UART4->TDR = uartRTXBuffer[uartRTXReadPointer++];
				uartRTXReadPointer &= UART_BUFFER_MASK;
#ifdef UART_SINGLE_WIRE
				uartRRXDiscardCounter++;
#endif
			}
		}

		// ***********************************************************************************************************************	U5 to Servo Chain Left (UART 5)
		UART_BytesAvailable = (-((DMA1_Stream0->NDTR) + uartLRXReadPointerDMA))
				& (UART_BUFFER_MASK);

		while (UART_BytesAvailable) {
			newChar = uartLRXBufferDMA[uartLRXReadPointerDMA++];
			uartLRXReadPointerDMA &= UART_BUFFER_MASK;
			UART_BytesAvailable--;

			if (uartLRXDiscardCounter) {// is this one of the chars we sent out (ignore the echo because of one-wire transmission)
				uartLRXDiscardCounter--;
			} else {

				if (uartLRXWritePointer < (UART_BUFFER_SIZE - 2))
					uartLRXBuffer[uartLRXWritePointer++] = newChar;
				if ((newChar == '\n') || (newChar == '\r')) {
					uartLRXBuffer[uartLRXWritePointer] = 0;
					uartSendString("L:");
					uartSendString(uartLRXBuffer);
					uartLRXWritePointer = 0;
				}

			}
		}

		if (uartLTXWritePointer != uartLTXReadPointer) {// ***************************************************************
			if ((UART5->ISR) & BIT(7)) {
				UART5->TDR = uartLTXBuffer[uartLTXReadPointer++];
				uartLTXReadPointer &= UART_BUFFER_MASK;
#ifdef UART_SINGLE_WIRE
				uartLRXDiscardCounter++;
#endif
			}
		}

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_UART4 | RCC_PERIPHCLK_UART5
			| RCC_PERIPHCLK_I2C1;
	PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
	PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_SYSCLK;
	PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_SYSCLK;
	PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
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
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x20404768;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 54000;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 0xffffffff;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 1000000;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_HalfDuplex_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	// **************************************************************************************************	configure DMA input
	hdma_uart4_rx.Instance = DMA1_Stream2;
	hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_uart4_rx.Init.Mode = DMA_CIRCULAR;
	hdma_uart4_rx.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma_uart4_rx.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_uart4_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;

	HAL_DMA_Init(&hdma_uart4_rx);							// initialize DMA
	UART4->CR3 |= USART_CR3_DMAR;					// enable UART4 receive DMA
	HAL_DMA_Start(&hdma_uart4_rx, (uint32_t) (&(UART4->RDR)),
			(uint32_t) uartRRXBufferDMA, UART_BUFFER_SIZE);	// start continuous ring buffer DMA transfer from UART4 -> RIGHT_RX_BUFFER

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void) {

	/* USER CODE BEGIN UART5_Init 0 */

	/* USER CODE END UART5_Init 0 */

	/* USER CODE BEGIN UART5_Init 1 */

	/* USER CODE END UART5_Init 1 */
	huart5.Instance = UART5;
	huart5.Init.BaudRate = 1000000;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_HalfDuplex_Init(&huart5) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART5_Init 2 */

	// **************************************************************************************************	configure DMA input
	hdma_uart5_rx.Instance = DMA1_Stream0;
	hdma_uart5_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_uart5_rx.Init.Mode = DMA_CIRCULAR;
	hdma_uart5_rx.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_uart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma_uart5_rx.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_uart5_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;

	HAL_DMA_Init(&hdma_uart5_rx);							// initialize DMA
	UART5->CR3 |= USART_CR3_DMAR;					// enable UART5 receive DMA
	HAL_DMA_Start(&hdma_uart5_rx, (uint32_t) (&(UART5->RDR)),
			(uint32_t) uartLRXBufferDMA, UART_BUFFER_SIZE);	// start continuous ring buffer DMA transfer from UART5 -> LEFT_RX_BUFFER

	/* USER CODE END UART5_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 1000000;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_HalfDuplex_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	// **************************************************************************************************	configure DMA input
	hdma_usart2_rx.Instance = DMA1_Stream5;
	hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
	hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma_usart2_rx.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_usart2_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;

	HAL_DMA_Init(&hdma_usart2_rx);							// initialize DMA
	USART2->CR3 |= USART_CR3_DMAR;					// enable UART2 receive DMA
	HAL_DMA_Start(&hdma_usart2_rx, (uint32_t) (&(USART2->RDR)),
			(uint32_t) uartSRXBufferDMA, UART_BUFFER_SIZE);	// start continuous ring buffer DMA transfer from UART2 -> SPINE_RX_BUFFER

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 1000000;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_CTS;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	// **************************************************************************************************	set RTS active
	GPIO_InitTypeDef pinDefU3;

	pinDefU3.Pin = (PORTB_USART3_RTS);
	pinDefU3.Mode = GPIO_MODE_OUTPUT_PP;
	pinDefU3.Pull = GPIO_NOPULL;
	pinDefU3.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &pinDefU3);
	PORTB->CLR = PORTB_USART3_RTS;	// set RTS low (i.e. allow incoming data)

	// **************************************************************************************************	configure DMA input
	hdma_usart3_rx.Instance = DMA1_Stream1;
	hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
	hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma_usart3_rx.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_usart3_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;

	HAL_DMA_Init(&hdma_usart3_rx);							// initialize DMA
	USART3->CR3 |= USART_CR3_DMAR;					// enable UART3 receive DMA
	HAL_DMA_Start(&hdma_usart3_rx, (uint32_t) (&(USART3->RDR)),
			(uint32_t) uart3RXBufferDMA, UART_BUFFER_SIZE);	// start continuous ring buffer DMA transfer from UART3 -> U3_RX_BUFFER

	/* USER CODE END USART3_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	/* DMA1_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_ADC_Supply_GND_Pin | GPIO_ADC_2_GND_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIO_out_LED_GPIO_Port, GPIO_out_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO____U3_RTS_Pin | GPIO_enable_Vreg_Servo_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIO_enable_Vreg_RPIZW_GPIO_Port,
	GPIO_enable_Vreg_RPIZW_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : GPIO_ADC_Supply_GND_Pin GPIO_ADC_2_GND_Pin */
	GPIO_InitStruct.Pin = GPIO_ADC_Supply_GND_Pin | GPIO_ADC_2_GND_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : GPIO_out_LED_Pin */
	GPIO_InitStruct.Pin = GPIO_out_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIO_out_LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PA3 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : GPIO_Nose_Bumper_Pin GPIO_Head_Padding_Pin */
	GPIO_InitStruct.Pin = GPIO_Nose_Bumper_Pin | GPIO_Head_Padding_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : GPIO_PCB_routing_dummy_Pin GPIO_input_routing_dummy_Pin PB3 */
	GPIO_InitStruct.Pin = GPIO_PCB_routing_dummy_Pin
			| GPIO_input_routing_dummy_Pin | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : GPIO____U3_RTS_Pin GPIO_enable_Vreg_Servo_Pin */
	GPIO_InitStruct.Pin = GPIO____U3_RTS_Pin | GPIO_enable_Vreg_Servo_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : GPIO_enable_Vreg_RPIZW_Pin */
	GPIO_InitStruct.Pin = GPIO_enable_Vreg_RPIZW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIO_enable_Vreg_RPIZW_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC11 */
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PD2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// ***********************************************************************************************************************
void enterBootLoader(void) {

	GPIO_InitTypeDef pinDefBL;

	__HAL_RCC_GPIOA_CLK_DISABLE();		// not needed
	__HAL_RCC_GPIOB_CLK_ENABLE();// used for PB10+11 (UART3) and POWER-enable for Servos
	__HAL_RCC_GPIOC_CLK_ENABLE();		// used for POWER-enable for RPIZW

	pinDefBL.Pin = (PORTC_ENABLE_RPIZW_POWER);
	pinDefBL.Mode = GPIO_MODE_OUTPUT_PP;
	pinDefBL.Pull = GPIO_PULLUP;
	pinDefBL.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOC, &pinDefBL);
	PORTC->SET = PORTC_ENABLE_RPIZW_POWER;

	pinDefBL.Pin = (PORTB_ENABLE_SERVOS_POWER);	// enable the Vreg for Servos
	pinDefBL.Mode = GPIO_MODE_OUTPUT_PP;
	pinDefBL.Pull = GPIO_PULLUP;
	pinDefBL.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &pinDefBL);
	PORTB->SET = PORTB_ENABLE_SERVOS_POWER;

	SysTick->CTRL = 0;// at least one of these is required before bootloader call
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	__set_PRIMASK(1);				// Disable interrupts

	// fetch the desired stack pointer from address 0x1FF00000 and store as SP
	__ASM volatile ("movw r3, #0x0000\nmovt r3, #0x1FF0\nldr r3, [r3, #0]\nMSR msp, r3\n" : : : "r3", "sp");
	((void (*)(void)) *((uint32_t*) 0x1FF00004))();

	while (1)
		;

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
//uartSendString("HAL error\n");
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
