/*
 * mouseServo.h
 *
 */

#ifndef MOUSESERVO_H_
#define MOUSESERVO_H_

#include "stm32l0xx_hal.h"
#include "simpleIODef.h"


#define SOFTWARE_VERSION		"0.8"

#define UART_SINGLE_WIRE

#define SERVO_CENTER_ON_POWERUP						// this makes the servo go to default position on powerup

#define LED_DEFAULT_BRIGHTNESS	50


//#define PROGRAMMING_BAUD_RATE		(57600)

// ***********************************************************************************************************************
#define BIT(x) ((1ul) << (x))


// ***********************************************************************************************************************
#define PORTC_MAGSENSOR_POWER		(BIT(15))
#define PORTA_LED_A					(BIT(6))				// TIM 22 CH 1
#define PORTA_LED_K					(BIT(7))				// TIM 22 CH 2

#define ADC_INPUT_VSUPPLY			0
#define ADC_INPUT_SERVOID			1
#define ADC_INPUT_MOTOR_CURRENT		5

// ***********************************************************************************************************************
#define IWDG_CLEAR()				{IWDG->KR = 0x0000AAAA;}

// ***********************************************************************************************************************
extern ADC_HandleTypeDef hadc;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

// ***********************************************************************************************************************
#define UART_BUFFER_SIZE	256
#define UART_BUFFER_MASK	((UART_BUFFER_SIZE)-1)


// ***********************************************************************************************************************
extern unsigned long SERVO_ID;						// ID in chain, 0..4

extern unsigned long systemTimeMS;					// system Time since power-up in milliseconds
													// (up to 2^32ms = 4294967296ms = 429496s = 71582min = 1193 hours = 49 days ... --> long enough)


extern long currentServoPosition;					// continuously updated in sensors.c
extern long desiredServoPosition;

extern long motorPIDControlEnabled;					// is motor PID control enabled?


// ***********************************************************************************************************************
extern char uartRXBufferDMA[];
extern long uartRXReadPointer;

extern char uartTXBuffer[];
extern long uartTXWritePointer, uartTXReadPointer;


// ***********************************************************************************************************************
// ***********************************************************************************************************************


// ***************************************************************************** main.c
extern void sleepMS(unsigned long);											// note this is only coarsely calibrated!
extern void stopAndEnterBootLoader(void);

void setLED(long onFlag, long brightness);
void setLEDBlink(long periodeMS, long brightness);


// ***************************************************************************** uart.c
extern void uartSendChar(char);
extern void uartSendString(char *);

extern void uartSendCharDirect(char);
extern void uartSendStringDirect(char *);

extern void uartSendHexLong(long);
extern void uartSendHexShort(long);
extern void uartSendHexByte(char hb);

extern void uartBusyWaitForTXDone(void);

extern void uartReceiveChar(char);

extern void uartShowVersion(void);

// ***************************************************************************** uart-ShowVersion.c
extern void uartShowVersion(void);

// ***************************************************************************** motorControl.c
extern void motorControlInit(void);
extern void motorControlIterate1KHz();

extern void setDesiredMotorPosition(long);
extern void setMotorPositionControlOff();

extern void setMotorSpeed(long);
extern void setMotorSpeedDelta(long);
extern long getMotorSpeed(void);


// ***************************************************************************** sensors.c
extern void sensorsInit(void);
extern void sensorsIterate(void);

extern void sensorWriteTest(void);
extern void sensorReadTest(void);
extern void sensorMathTest(void);


#endif /* MOUSESERVO_H_ */
