/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */


// TODO:
//
//  -) DONE: implement I2C ADC to read foot pressure sensor
//
//  -) develop / tune PID controller. Think about timing for D (and I?) parts...
//     - currently, iteration time is dubious (non-consistent). Can I use another timer to generate "system time"?
//     -  or shall I call motor control at fixed iteration only? (e.g. 100Hz)
////
//  -) internal ADC to read battery voltage
//        -) why?
//
//  -) internal ADC synchronized with timer to read motor currents
//
//


/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <mouseServo.h>


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
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;
TIM_HandleTypeDef htim22;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

unsigned long systemTimeMS;

unsigned long SERVO_ID = 0;

long ledCounter = 1;
long ledOnFlag = 0;
long ledCounterInit = 250;								// toggle after 250ms -> period 500ms -> 2Hz
long ledBrightness = 100;								// 0 .. 1000
long UART2_BytesAvailable=0;

extern long UARTOutputEnabled;							// is UART output enabled?

#ifdef UART_SINGLE_WIRE
extern long uartRXDiscardCounter;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM21_Init(void);
static void MX_TIM22_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ***********************************************************************************************************************
void sleepMS(unsigned long miliseconds) {														// rough calibration (i.e. not precise!)
	if (miliseconds) {
		for ( ; miliseconds; miliseconds--) {
#ifdef DEBUG
			for (unsigned long t=0; t<(5470); t++) { asm volatile ("nop"); }
#else
			for (unsigned long t=0; t<(5470); t++) { asm volatile ("nop"); }
#endif
		}
	}
}

// ***********************************************************************************************************************
void identifyServoID(void) {
	// use analog input to identify servo ID

	// open	-> AD = 4095		-> ID = 0;
	// 0K	-> AD = 0			-> ID = 1;
	// 1K	-> AD = 114			-> ID = 2;
	// 10K	-> AD = 910			-> ID = 3;
	// 100K	-> AD = 3033		-> ID = 4;

	long r, dummy;

	__HAL_RCC_ADC1_CLK_ENABLE();

	ADC1->SMPR &= (uint32_t)(~ADC_SMPR_SMPR);						// Clear the old sampling time
	ADC1->SMPR |= ADC_SAMPLETIME_79CYCLES_5;						// Set the new sample time (4 = 79.5 cycles)

	HAL_ADC_Start(&hadc);										// start the on-chip ADC converter		// this will read VSupply
	HAL_ADC_PollForConversion(&hadc, 10);						// timeout in ms
	r = HAL_ADC_GetValue(&hadc);

	HAL_ADC_Start(&hadc);										// start the on-chip ADC converter		// this will read ServoID
	HAL_ADC_PollForConversion(&hadc, 10);						// timeout in ms
	r = HAL_ADC_GetValue(&hadc);

	HAL_ADC_Start(&hadc);										// start the on-chip ADC converter		// this will read motorCurrent
	HAL_ADC_PollForConversion(&hadc, 10);						// timeout in ms
	dummy = HAL_ADC_GetValue(&hadc);
	dummy++;													// (use dummy, so the compilation warning goes)

	if		(r > 4000)		SERVO_ID = 0;
	else if	(r < 50)		SERVO_ID = 1;
	else if	(r < 510)		SERVO_ID = 2;
	else if	(r < 1970)		SERVO_ID = 3;
	else					SERVO_ID = 4;

}

void DUMMY_old_ServoID(void) {

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* use PA1 to retrieve Servo ID */

	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;							// first output GND
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	PORTA->CLR = GPIO_PIN_1;											// now we "push" GND

	sleepMS(2);															// wait briefly

	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;								// change to input and pullup
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	asm volatile ("nop");												// wait *very briefly*
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");

	if ((PORTA->INP & GPIO_PIN_1)) {									// this pin has "followed immediately" to HIGH -> it's a floating pin -> Servo ID 0
		SERVO_ID = 0;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		return;
	}

	sleepMS(50);														// wait for internal pull-up to charge capacitor	// TODO: 20 should be enough

	if ((PORTA->INP & GPIO_PIN_1)==0) {									// this pin is a short to GND ... we are servo ID 0
		SERVO_ID = 1;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		return;
	}

																		// else no shortcut to GND -> we can "fully charge the capacitor with a high output pin

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;							// now output HIGH
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	PORTA->SET = GPIO_PIN_1;											// now we "push" HIGH

	sleepMS(2);															// wait briefly

	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;								// change to input, NO pull
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	long count = 0;
	while ((PORTA->INP & GPIO_PIN_1) && (count<1000)) {					// wait for port input to drop or timeout
		count++;
		sleepMS(1);														// wait 1ms
	}

//	SERVO_ID = count; return;
																		// the prototypes (2, 3, 4) show 24 (0x18), 51 (0x33), 102 (0x66); so we separate in the middle :)
	if (count < 38) { SERVO_ID = 2; }
	else {
		if (count < 77) { SERVO_ID = 3; }
		else			{ SERVO_ID = 4; }
	}

	return;
}

// ***********************************************************************************************************************
void enterBootLoader(void) {
	GPIO_InitTypeDef pinDefBL;

	__HAL_RCC_GPIOA_CLK_ENABLE();		// needed for PA2+3 (UART2 TX+RX) + motor-control
	__HAL_RCC_GPIOB_CLK_ENABLE();		// needed for motor-control
	__HAL_RCC_GPIOC_CLK_DISABLE();		// not needed

	pinDefBL.Pin = (GPIO_PIN_8);						// configure PA8 as input with pullup (to switch off motor)
	pinDefBL.Mode = GPIO_MODE_INPUT;
	pinDefBL.Pull = GPIO_PULLUP;
	pinDefBL.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &pinDefBL);

	pinDefBL.Pin = (GPIO_PIN_3);						// configure PB3 as input with pullup (to switch off motor)
	pinDefBL.Mode = GPIO_MODE_INPUT;
	pinDefBL.Pull = GPIO_PULLUP;
	pinDefBL.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &pinDefBL);


	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;
	/** * Step: Disable all interrupts */
	__disable_irq();

	/* ARM Cortex-M Programming Guide to Memory Barrier Instructions.*/
	__DSB();
	__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

	/* Remap is not visible at once. Execute some unrelated command! */
	__DSB();
	__ISB();


//	JumpToApplication = (void (*)(void)) (*((uint32_t *)(0x1FFF0000 + 4)));
//
//	/* Initialize user application's Stack Pointer */
//	__set_MSP(*(__IO uint32_t*) 0x1FFF0000);
//	JumpToApplication();


    __ASM volatile ("movs r3, #0\nldr r3, [r3, #0]\nMSR msp, r3\n" : : : "r3", "sp");

    ((void (*)(void)) *((uint32_t*) 0x00000004))();

}

// ***********************************************************************************************************************
void stopAndEnterBootLoader(void) {
	HAL_DMA_DeInit(&hdma_usart2_rx);

//	HAL_IW
	HAL_ADC_DeInit(&hadc);
	HAL_UART_DeInit(&huart2);
	HAL_TIM_Base_DeInit(&htim2);
	HAL_I2C_DeInit(&hi2c1);

	HAL_RCC_DeInit();

	HAL_GPIO_DeInit(GPIOA, 0xFFFF);		// set all pins to default values
	HAL_GPIO_DeInit(GPIOB, 0xFFFF);		// set all pins to default values
	HAL_GPIO_DeInit(GPIOC, 0xFFFF);		// set all pins to default values

    HAL_DeInit();

    enterBootLoader();
}

// ***********************************************************************************************************************
inline void setLED(long onFlag, long brightness) {

	if (onFlag) {
		ledCounterInit = 1;			// const on (ok, toggling at 1ms -> 500Hz)
		ledCounter = 1;
	} else {
		ledCounter = 0;				// const off
		PORTA->CLR = PORTA_LED_K;
		TIM22->CCR1 = 0;
	}

	ledBrightness = brightness;
	if (ledBrightness > 1000) ledBrightness = 1000;
}

// ***********************************************************************************************************************
inline void setLEDBlink(long periodeMS, long brightness) {
	ledCounterInit = periodeMS;					// this is used both for on and off
	ledCounter = 1;

	ledBrightness = brightness;
	if (ledBrightness > 1000) ledBrightness = 1000;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// **************************************************************************************************	IWDG (independent watch-dog timer)
	if ((RCC->CSR) & BIT(29)) {		// are we waking up from a IWDG reset?
		RCC->CSR |= BIT(23);		// clear reset flags (memory of reset source) (so next time we're not ending here)

		HAL_Init();
		SystemClock_Config();

		GPIO_InitTypeDef GPIO_InitStruct = {0};

		__HAL_RCC_GPIOA_CLK_ENABLE();

		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

		/*Configure GPIO pin : PA6 + PA7 */
		GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		PORTA->SET = BIT(6);
		PORTA->CLR = BIT(7);

		sleepMS(500);

		enterBootLoader();			// start boot loader, so we can update software if needed!
	}

	// be careful with code here, as the IWDG is NOT initialized yet!

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	// be careful with code here, as the IWDG is NOT initialized yet!
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	// be careful with code here, as the IWDG is NOT initialized yet!
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM21_Init();
  MX_TIM22_Init();
  /* USER CODE BEGIN 2 */

#ifndef UART_SINGLE_WIRE
  uartSendStringDirect("\n\n\n");
#endif


  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
  long ADCcal = HAL_ADCEx_Calibration_GetValue(&hadc, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_SetValue(&hadc, ADC_SINGLE_ENDED, ADCcal);


  identifyServoID();

  sensorsInit();
  motorControlInit();

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

#ifndef UART_SINGLE_WIRE
  uartShowVersion();
#endif

  TIM21->CNT = 0;								// set timer to zero
  systemTimeMS = 0;								// set System Time Variable to zero
  HAL_TIM_Base_Start(&htim21);


  TIM22->CNT = 0;								// set timer to zero
  HAL_TIM_Base_Start(&htim22);
  HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_1);	// start PWM output
  PORTA->CLR = PORTA_LED_K;						// set LED pin to GND		--- all LED off

  setLEDBlink(250, LED_DEFAULT_BRIGHTNESS);							// set LED blinking period 250ms -> 4Hz; brightness 100/1000

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // ***********************************************************************************************************************
	  IWDG_CLEAR();																		// clear watchdog, assuming all well :)

	  // ***********************************************************************************************************************
	  if ((systemTimeMS & 0xFFFF) != (TIM21->CNT)) {									// check if (1ms) has elapsed
		  systemTimeMS++;																	// advance MS system time

	  // ***********************************************************************************************************************
		  if (ledCounter>0) {															// LED blinking
			  if ((--ledCounter) == 0) {
				  ledCounter = ledCounterInit;

				  if (ledOnFlag) {										// here the PCBWays LED is on
					  PORTA->SET = PORTA_LED_K;							// set LED pin to GND		-- PCBWAYS
					  TIM22->CCR1 = 1000-ledBrightness;
					  ledOnFlag = 0;
				  } else {												// here the hand-soldered LED is on
					  PORTA->CLR = PORTA_LED_K;							// set LED pin to GND		-- HAND SOLDERED
					  TIM22->CCR1 = ledBrightness;
					  ledOnFlag = 1;
				  }

			  }
		  }

	  // ***********************************************************************************************************************
		  motorControlIterate1KHz();															// run motor control at 1 KHz

	  }

	  // ***********************************************************************************************************************
	  UART2_BytesAvailable = (-((DMA1_Channel5->CNDTR) + uartRXReadPointer)) & (UART_BUFFER_MASK);

	  while (UART2_BytesAvailable) {
		  char newChar = uartRXBufferDMA[uartRXReadPointer++];
		  uartRXReadPointer &= UART_BUFFER_MASK;
		  UART2_BytesAvailable--;

#ifdef UART_SINGLE_WIRE
		  if (uartRXDiscardCounter) {							// is this one of the chars we sent out (ignore the echo because of one-wire transmission)
			  uartRXDiscardCounter--;
		  } else {
			  uartReceiveChar(newChar);
		  }
#else
		  uartReceiveChar(newChar);
#endif
	  }

//	  if ((USART2->ISR) & BIT(5)) {														// UART receive
//		  uartReceiveChar(USART2->RDR);
//	  }
//	  if ((USART2->ISR) & BIT(3)) {														// Overrun detected?
//		  USART2->ICR |= BIT(3);								// clear overrun flag
//		  uartSendChar('x');
//	  }

	  // ***********************************************************************************************************************
	  if (uartTXWritePointer != uartTXReadPointer) {									// UART transmit
		  if ((USART2->ISR) & BIT(7)) {
			  if (UARTOutputEnabled) {
				  USART2->TDR = uartTXBuffer[uartTXReadPointer];
#ifdef UART_SINGLE_WIRE
				  uartRXDiscardCounter++;
#endif
			  }
			  uartTXReadPointer++;
			  uartTXReadPointer &= UART_BUFFER_MASK;
		  }
	  }
	  // ***********************************************************************************************************************
	  sensorsIterate();
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_79CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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

  /* Timing according to Reference Manual
   * 0 -> no Prescaler
   * 0 -> reserved
   * 3 -> Data Setup 125ns
   * 0 -> Data Hold 0ns
   * 15 -> HIGH 687,5ns
   * 2D -> LOW 1437,5ns
   *
   * -> ca. 2.62us per cycle -> 380kHz I2C Clock Signal
   */
  hi2c1.Init.Timing = 0x0030152D;

  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4095;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 32000;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 65535;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */

}

/**
  * @brief TIM22 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM22_Init(void)
{

  /* USER CODE BEGIN TIM22_Init 0 */

  /* USER CODE END TIM22_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM22_Init 1 */

  /* USER CODE END TIM22_Init 1 */
  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 2;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = 1000;
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim22.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim22) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim22, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim22) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim22, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim22, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM22_Init 2 */

  /* USER CODE END TIM22_Init 2 */
  HAL_TIM_MspPostInit(&htim22);

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
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  // **************************************************************************************************	configure DMA input
  hdma_usart2_rx.Instance = DMA1_Channel5;
  hdma_usart2_rx.Init.Request = DMA_REQUEST_4;
  hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
  hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;

  HAL_DMA_Init(&hdma_usart2_rx);								// initialize DMA
  USART2->CR3 |= USART_CR3_DMAR;						// enable UART2 receive DMA
  HAL_DMA_Start(&hdma_usart2_rx, (uint32_t)(&(USART2->RDR)), (uint32_t) uartRXBufferDMA, UART_BUFFER_SIZE);		// start continuous ring buffer DMA transfer from UART3 -> U3_RX_BUFFER

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
