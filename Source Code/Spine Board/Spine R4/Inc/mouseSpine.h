/*
 * mouseSpine.h
 *
 */

#ifndef MOUSESPINE_H_
#define MOUSESPINE_H_

#include "stm32f7xx_hal.h"
#include "simpleIODef.h"


#define SOFTWARE_VERSION		"0.8"


#define UART_SINGLE_WIRE


// ***********************************************************************************************************************
#define BIT(x) ((1) << (x))


// ***********************************************************************************************************************
#define PORTC_ENABLE_RPIZW_POWER	(BIT(8))								// enable Vreg for RPIZW
#define PORTB_ENABLE_SERVOS_POWER	(BIT(15))								// enable Vreg for Servos

#define PORTB_USART3_RTS			(BIT(14))								// GPIO used as RTS signal		// TODO!

#define PORTC_ADC_SUPPLY_GND		(BIT(2) | BIT(3))

#define PORTA_LED_A					(BIT(1))


// ***********************************************************************************************************************
extern ADC_HandleTypeDef hadc1;

extern UART_HandleTypeDef huart4;				// for servo leg right
extern UART_HandleTypeDef huart5;				// for servo leg left
//extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;				// for servo head-spine
extern UART_HandleTypeDef huart3;				// for RPI-ZW
//extern UART_HandleTypeDef huart6;

// ***********************************************************************************************************************
#define UART_BUFFER_SIZE	4096
#define UART_BUFFER_MASK	((UART_BUFFER_SIZE)-1)


// ***********************************************************************************************************************	world time
#define systemTimeHalfMS		(TIM5->CNT)
//extern unsigned long systemTimeMS;				// system Time since power-up in milliseconds
													// (up to 2^32ms = 4294967296ms = 429496s = 71582min = 1193 hours = 49 days ... --> long enough)
													// currently "HalfMS" -> "only" 24.5 days ... ;)


// ***********************************************************************************************************************	to RPIZW
extern char uart3RXBufferDMA[], uart3RXBuffer[], uart3TXBuffer[];
long uart3RXReadPointerDMA, uart3RXWritePointer, uart3TXWritePointer, uart3TXReadPointer;


// ***********************************************************************************************************************	to Servo Chain Left (UART 5)
extern char uartLRXBufferDMA[], uartLRXBuffer[], uartLTXBuffer[];
long uartLRXReadPointerDMA, uartLRXWritePointer, uartLRXDiscardCounter, uartLTXWritePointer, uartLTXReadPointer;

// ***********************************************************************************************************************	to Servo Chain Right Front (UART 4)
extern char uartRRXBufferDMA[], uartRRXBuffer[], uartRTXBuffer[];
long uartRRXReadPointerDMA, uartRRXWritePointer, uartRRXDiscardCounter, uartRTXWritePointer, uartRTXReadPointer;

// ***********************************************************************************************************************	to Servo Chain Head Spine (UART 2)
extern char uartSRXBufferDMA[], uartSRXBuffer[], uartSTXBuffer[];
long uartSRXReadPointerDMA, uartSRXWritePointer, uartSRXDiscardCounter, uartSTXWritePointer, uartSTXReadPointer;



// ***********************************************************************************************************************
// ***********************************************************************************************************************


// ***************************************************************************** main.c
extern void setLEDcounter(long);
extern void sleepUS(unsigned long);
extern void sleepMS(unsigned long);
extern void enterBootLoader(void);


// ***************************************************************************** uart.c
extern void uartSendChar(char);
extern void uartSendString(char *);

extern void uartLSendChar(char);						// Left Front
extern void uartLSendString(char *);
extern void uartRSendChar(char);						// Right Front
extern void uartRSendString(char *);
extern void uartSSendChar(char);						// Head Spine
extern void uartSSendString(char *);


extern void uartSendCharDirect(char);
extern void uartSendStringDirect(char *);

extern void uartSendHexLong(long);
extern void uartSendHexShort(long);
extern void uartSendHexByte(char hb);

extern void uartBusyWaitForTXDone(void);
extern void uartDiscardAllIncomingData(void);

extern void uartParseCommand(char *);
//extern void uartReceiveChar(char);

extern void uartShowVersion(void);

// ***************************************************************************** uart-ShowVersion.c
extern void uartShowVersion(void);

// ***************************************************************************** uart-ForwardReprogramming.c
void uartForwardReprogramServoChainLeft(char);
void uartForwardReprogramServoChainRight(char);
void uartForwardReprogramServoChainSpine(char);


#endif /* MOUSESPINE_H_ */
