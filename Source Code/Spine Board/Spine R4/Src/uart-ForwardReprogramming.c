/*
 * uart-forwardReprogramming.c
 *
 */
#include <mouseSpine.h>

#define FORWARD_PROGRAMMING_BAUD_RATE		(57600)

						// REMEBER that for reprogramming, we need to swap pins for the UART port on Spine (TxD-RxD pinswap), so that TxD(Spine) is connected to RxD-Servo)



// ****************************************************************************	// change UART port parameters to reprogramming mode	UART 3 RPIZW
void U3SetToReprogConfiguration() {
	GPIO_InitTypeDef pinDefU3;

	PORTB->SET = GPIO_PIN_10;													// set TxD pin to high
	pinDefU3.Pin = GPIO_PIN_10;													// switch TxD to PP output pin
	pinDefU3.Mode = GPIO_MODE_OUTPUT_PP;
	pinDefU3.Pull = GPIO_PULLUP;
	pinDefU3.Speed = GPIO_SPEED_HIGH;
	pinDefU3.Alternate = 0;
	HAL_GPIO_Init(GPIOB, &pinDefU3);
	huart3.Init.BaudRate = FORWARD_PROGRAMMING_BAUD_RATE;
	huart3.Init.WordLength = UART_WORDLENGTH_9B;
	huart3.Init.Parity = UART_PARITY_EVEN;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	HAL_UART_Init(&huart3);
	pinDefU3.Mode = GPIO_MODE_AF_PP;
	pinDefU3.Pull = GPIO_NOPULL;
	pinDefU3.Alternate = GPIO_AF7_USART3;											// use AF7: USART3 TxD, RxD
	HAL_GPIO_Init(GPIOB, &pinDefU3);
}



// ***********************************************************************************************************************
// ***********************************************************************************************************************
void uartForwardReprogramServoChainLeft(char servoID) {							// UART5, Left
	GPIO_InitTypeDef pinDef;

	// ************************************************************************ // "flush" RPIZW UART
	uartBusyWaitForTXDone();

	// ************************************************************************ // get servo(s) in reprogramming mode and clear UART buffer
	uartLSendChar('\n');													// clear possibly left-over data
	uartLSendChar(servoID);												// send servo ID
	uartLSendString("BOOTL\n");												// send reprogramming command
	while (uartLTXWritePointer != uartLTXReadPointer) {						// transmit all data to servo chain
		if ((UART5->ISR) & BIT(7)) {
			UART5->TDR = uartLTXBuffer[uartLTXReadPointer++];
			uartLTXReadPointer &= UART_BUFFER_MASK;
		}
	}
	while (((UART5->ISR) & BIT(6))==0) { asm volatile ("nop"); };				// wait for last character to finish transmission

	// ************************************************************************ // switch RxD and TxD for Servo Chain to input with pullup
	pinDef.Pin = GPIO_PIN_12;													// switch TxD (PC12) to input pin with pullup
	pinDef.Mode = GPIO_MODE_INPUT;
	pinDef.Pull = GPIO_PULLUP;
	pinDef.Speed = GPIO_SPEED_HIGH;
	pinDef.Alternate = 0;
	HAL_GPIO_Init(GPIOC, &pinDef);
	pinDef.Pin = GPIO_PIN_2;													// switch RxD (PD2) to input pin with pullup
	pinDef.Mode = GPIO_MODE_INPUT;
	pinDef.Pull = GPIO_PULLUP;
	pinDef.Speed = GPIO_SPEED_HIGH;
	pinDef.Alternate = 0;
	HAL_GPIO_Init(GPIOD, &pinDef);

	// ************************************************************************ // wait for servo to settle and discard all UART data
	sleepMS(200);																// wait
	uartDiscardAllIncomingData();												// discard all UART data

	// ************************************************************************ // change UART port parameters to reprogramming mode	UART 5 Legs Left Front
	huart5.Init.BaudRate = FORWARD_PROGRAMMING_BAUD_RATE;						// set baud rate for reprogramming
	huart5.Init.WordLength = UART_WORDLENGTH_9B;								// enable 8 bits + parity -> 9 bits
	huart5.Init.Parity = UART_PARITY_EVEN;										// set even parity
	huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;				// swap TxD and RxD pins
	huart5.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
	HAL_UART_Init(&huart5);														// configure UART

	pinDef.Pin = GPIO_PIN_12;													// switch TxD (PC12) to UART function
	pinDef.Mode = GPIO_MODE_AF_PP;
	pinDef.Pull = GPIO_NOPULL;
	pinDef.Alternate = GPIO_AF8_UART5;											// use AF8: UART5 TxD pin (now with RxD function because of pin swap)
	HAL_GPIO_Init(GPIOC, &pinDef);

	pinDef.Pin = GPIO_PIN_2;													// switch RxD (PD2) to UART function
	pinDef.Mode = GPIO_MODE_AF_PP;
	pinDef.Pull = GPIO_NOPULL;
	pinDef.Alternate = GPIO_AF8_UART5;											// use AF8: UART5 RxD pin (now with TxD function because of pin swap)
	HAL_GPIO_Init(GPIOD, &pinDef);

	// ************************************************************************ // change UART port parameters to reprogramming mode	UART 3 RPIZW
	U3SetToReprogConfiguration();

	// ************************************************************************ // now forward all data between ports
	char newChar;
	long ledCounter=1;

	long timeMemory = systemTimeHalfMS;
	long allowedTimeoutMS = 30000;									// allow 30000 sek until 512 chars are transmitted; afterwards "shorter"
	long progDataCounter = 0;										// count how many chars are transmitted PC -> Spine -> Servo

	while (timeMemory) {

		// ****************************************************************** // LED blinking
		if (ledCounter) {
			if ((--ledCounter) == 0) {
				ledCounter = 100000;
				if ((PORTA->INP) & PORTA_LED_A) {
					PORTA->CLR = PORTA_LED_A;
				} else {
					PORTA->SET = PORTA_LED_A;
				}
			}
		}

		// ****************************************************************** // check "end-of-programming" (i.e. two seconds no communication)
		if (timeMemory + (allowedTimeoutMS*2) < systemTimeHalfMS) {
			timeMemory = 0;
		}

		// ****************************************************************** // UART 3 (RPIZW)  -->  UART 5 (Legs Left Front)
		if ((-((DMA1_Stream1->NDTR) + uart3RXReadPointerDMA)) & (UART_BUFFER_MASK)) {
			newChar = uart3RXBufferDMA[uart3RXReadPointerDMA++];				// get from U3
			uart3RXReadPointerDMA &= UART_BUFFER_MASK;

			while (((UART5->ISR) & BIT(7)) == 0) { asm volatile ("nop"); }		// send to U5
			UART5->TDR = newChar;

			timeMemory = systemTimeHalfMS;									 	// we have received data; reset time memory
			if (++progDataCounter == 512) allowedTimeoutMS = 1000;				// only one second allowed as of now
		}

		// ****************************************************************** // UART 5 (Legs Left Front)  -->  UART 3 (RPIZW)
		if ((-((DMA1_Stream0->NDTR) + uartLRXReadPointerDMA)) & (UART_BUFFER_MASK)) {
			newChar = uartLRXBufferDMA[uartLRXReadPointerDMA++];				// get from U5
			uartLRXReadPointerDMA &= UART_BUFFER_MASK;

			while (((USART3->ISR) & BIT(7)) == 0) { asm volatile ("nop"); }		// send to U3
			USART3->TDR = newChar;
		}

	}	// end of while "programming active"

	// ************************************************************************ // done with reprogramming, reset Spine to reset configuration
	NVIC_SystemReset();
}


// ***********************************************************************************************************************
// ***********************************************************************************************************************
void uartForwardReprogramServoChainRight(char servoID) {						// UART4, Right
	GPIO_InitTypeDef pinDef;

	// ************************************************************************ // "flush" RPIZW UART
	uartBusyWaitForTXDone();

	// ************************************************************************ // get servo(s) in reprogramming mode and clear UART buffer
	uartRSendChar('\n');														// clear possibly left-over data
	uartRSendChar(servoID);												// send servo ID
	uartRSendString("BOOTL\n");												// send reprogramming command
	while (uartRTXWritePointer != uartRTXReadPointer) {						// transmit all data to servo chain
		if ((UART4->ISR) & BIT(7)) {
			UART4->TDR = uartRTXBuffer[uartRTXReadPointer++];
			uartRTXReadPointer &= UART_BUFFER_MASK;
		}
	}
	while (((UART4->ISR) & BIT(6))==0) { asm volatile ("nop"); };				// wait for last character to finish transmission

	// ************************************************************************ // switch RxD and TxD for Servo Chain to input with pullup
	pinDef.Pin = (GPIO_PIN_10 | GPIO_PIN_11);									// switch TxD (PC10) and RxD (PC11) to input pin with pullup
	pinDef.Mode = GPIO_MODE_INPUT;
	pinDef.Pull = GPIO_PULLUP;
	pinDef.Speed = GPIO_SPEED_HIGH;
	pinDef.Alternate = 0;
	HAL_GPIO_Init(GPIOC, &pinDef);

	// ************************************************************************ // wait for servo to settle and discard all UART data
	sleepMS(200);																// wait
	uartDiscardAllIncomingData();												// discard all UART data

	// ************************************************************************ // change UART port parameters to reprogramming mode	UART 4 Legs Left Back
	huart4.Init.BaudRate = FORWARD_PROGRAMMING_BAUD_RATE;						// set baud rate for reprogramming
	huart4.Init.WordLength = UART_WORDLENGTH_9B;								// enable 8 bits + parity -> 9 bits
	huart4.Init.Parity = UART_PARITY_EVEN;										// set even parity
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;				// swap TxD and RxD pins
	huart4.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
	HAL_UART_Init(&huart4);														// configure UART

	pinDef.Pin = (GPIO_PIN_10 | GPIO_PIN_11);									// switch TxD (PC10) and RxD (PC11) to UART function
	pinDef.Mode = GPIO_MODE_AF_PP;
	pinDef.Pull = GPIO_NOPULL;
	pinDef.Alternate = GPIO_AF8_UART4;											// use AF8: UART4 RxD and TxD pins (pin swap)
	HAL_GPIO_Init(GPIOC, &pinDef);

	// ************************************************************************ // change UART port parameters to reprogramming mode	UART 3 RPIZW
	U3SetToReprogConfiguration();

	// ************************************************************************ // now forward all data between ports
	char newChar;
	long ledCounter=1;

	long timeMemory = systemTimeHalfMS;
	long allowedTimeoutMS = 30000;									// allow 30000 sek until 512 chars are transmitted; afterwards "shorter"
	long progDataCounter = 0;										// count how many chars are transmitted PC -> Spine -> Servo

	while (timeMemory) {

		// ****************************************************************** // LED blinking
		if (ledCounter) {
			if ((--ledCounter) == 0) {
				ledCounter = 100000;
				if ((PORTA->INP) & PORTA_LED_A) {
					PORTA->CLR = PORTA_LED_A;
				} else {
					PORTA->SET = PORTA_LED_A;
				}
			}
		}

		// ****************************************************************** // check "end-of-programming" (i.e. two seconds no communication)
		if (timeMemory + (allowedTimeoutMS*2) < systemTimeHalfMS) {
			timeMemory = 0;
		}

		// ****************************************************************** // UART 3 (RPIZW)  -->  UART 4 (Legs Left Back)
		if ((-((DMA1_Stream1->NDTR) + uart3RXReadPointerDMA)) & (UART_BUFFER_MASK)) {
			newChar = uart3RXBufferDMA[uart3RXReadPointerDMA++];				// get from U3
			uart3RXReadPointerDMA &= UART_BUFFER_MASK;

			while (((UART4->ISR) & BIT(7)) == 0) { asm volatile ("nop"); }		// send to U4
			UART4->TDR = newChar;

			timeMemory = systemTimeHalfMS;									  	// we have received data; reset time memory
			if (++progDataCounter == 512) allowedTimeoutMS = 1000;				// only one second allowed as of now
		}

		// ****************************************************************** // UART 4 (Legs Left Back)  -->  UART 3 (RPIZW)
		if ((-((DMA1_Stream2->NDTR) + uartRRXReadPointerDMA)) & (UART_BUFFER_MASK)) {
			newChar = uartRRXBufferDMA[uartRRXReadPointerDMA++];				// get from U4
			uartRRXReadPointerDMA &= UART_BUFFER_MASK;

			while (((USART3->ISR) & BIT(7)) == 0) { asm volatile ("nop"); }		// send to U3
			USART3->TDR = newChar;
		}

	}	// end of while "programming active"

	// ************************************************************************ // done with reprogramming, reset Spine to reset configuration
	NVIC_SystemReset();
}


// ***********************************************************************************************************************
// ***********************************************************************************************************************
void uartForwardReprogramServoChainSpine(char servoID) {						// USART2, Spine
	GPIO_InitTypeDef pinDef;

	// ************************************************************************ // "flush" RPIZW UART
	uartBusyWaitForTXDone();

	// ************************************************************************ // get servo(s) in reprogramming mode and clear UART buffer
	uartSSendChar('\n');														// clear possibly left-over data
	uartSSendChar(servoID);												// send servo ID
	uartSSendString("BOOTL\n");												// send reprogramming command
	while (uartSTXWritePointer != uartSTXReadPointer) {						// transmit all data to servo chain
		if ((USART2->ISR) & BIT(7)) {
			USART2->TDR = uartSTXBuffer[uartSTXReadPointer++];
			uartSTXReadPointer &= UART_BUFFER_MASK;
		}
	}
	while (((USART2->ISR) & BIT(6))==0) { asm volatile ("nop"); };				// wait for last character to finish transmission

	// ************************************************************************ // switch RxD and TxD for Servo Chain to input with pullup
	pinDef.Pin = (GPIO_PIN_2 | GPIO_PIN_3);										// switch TxD (PA2) and RxD (PA3) to input pin with pullup
	pinDef.Mode = GPIO_MODE_INPUT;
	pinDef.Pull = GPIO_PULLUP;
	pinDef.Speed = GPIO_SPEED_HIGH;
	pinDef.Alternate = 0;
	HAL_GPIO_Init(GPIOA, &pinDef);

	// ************************************************************************ // wait for servo to settle and discard all UART data
	sleepMS(200);																// wait
	uartDiscardAllIncomingData();												// discard all UART data

	// ************************************************************************ // change UART port parameters to reprogramming mode	UART 2 Legs Head Spine
	huart2.Init.BaudRate = FORWARD_PROGRAMMING_BAUD_RATE;						// set baud rate for reprogramming
	huart2.Init.WordLength = UART_WORDLENGTH_9B;								// enable 8 bits + parity -> 9 bits
	huart2.Init.Parity = UART_PARITY_EVEN;										// set even parity
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;				// swap TxD and RxD pins
	huart2.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
	HAL_UART_Init(&huart2);														// configure UART

	pinDef.Pin = (GPIO_PIN_2 | GPIO_PIN_3);										// switch TxD (PA2) and RxD (PA3) to UART function
	pinDef.Mode = GPIO_MODE_AF_PP;
	pinDef.Pull = GPIO_NOPULL;
	pinDef.Alternate = GPIO_AF7_USART2;											// use AF7: USART2 RxD and TxD pins (pin swap)
	HAL_GPIO_Init(GPIOA, &pinDef);

	// ************************************************************************ // change UART port parameters to reprogramming mode	UART 3 RPIZW
	U3SetToReprogConfiguration();

	// ************************************************************************ // now forward all data between ports
	char newChar;
	long ledCounter=1;

	long timeMemory = systemTimeHalfMS;
	long allowedTimeoutMS = 30000;									// allow 30000 sek until 512 chars are transmitted; afterwards "shorter"
	long progDataCounter = 0;										// count how many chars are transmitted PC -> Spine -> Servo

	while (timeMemory) {

		// ****************************************************************** // LED blinking
		if (ledCounter) {
			if ((--ledCounter) == 0) {
				ledCounter = 100000;
				if ((PORTA->INP) & PORTA_LED_A) {
					PORTA->CLR = PORTA_LED_A;
				} else {
					PORTA->SET = PORTA_LED_A;
				}
			}
		}

		// ****************************************************************** // check "end-of-programming" (i.e. two seconds no communication)
		if (timeMemory + (allowedTimeoutMS*2) < systemTimeHalfMS) {
			timeMemory = 0;
		}

		// ****************************************************************** // UART 3 (RPIZW)  -->  USART 2 (Head Spine)
		if ((-((DMA1_Stream1->NDTR) + uart3RXReadPointerDMA)) & (UART_BUFFER_MASK)) {
			newChar = uart3RXBufferDMA[uart3RXReadPointerDMA++];				// get from U3
			uart3RXReadPointerDMA &= UART_BUFFER_MASK;

			while (((USART2->ISR) & BIT(7)) == 0) { asm volatile ("nop"); }		// send to U2
			USART2->TDR = newChar;

			timeMemory = systemTimeHalfMS;									    // we have received data; reset time memory
			if (++progDataCounter == 512) allowedTimeoutMS = 1000;				// only one second allowed as of now
		}

		// ****************************************************************** // UART 2 (Head Spine)  -->  UART 3 (RPIZW)
		if ((-((DMA1_Stream5->NDTR) + uartSRXReadPointerDMA)) & (UART_BUFFER_MASK)) {
			newChar = uartSRXBufferDMA[uartSRXReadPointerDMA++];				// get from U2
			uartSRXReadPointerDMA &= UART_BUFFER_MASK;

			while (((USART3->ISR) & BIT(7)) == 0) { asm volatile ("nop"); }		// send to U3
			USART3->TDR = newChar;
		}

	}	// end of while "programming active"

	// ************************************************************************ // done with reprogramming, reset Spine to reset configuration
	NVIC_SystemReset();
}
