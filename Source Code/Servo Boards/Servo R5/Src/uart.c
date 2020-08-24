/*
 * uart.c
 *
 */

#include <mouseServo.h>
#include <string.h>

extern long positionError, positionErrorI, positionErrorD;		// TODO: debug
extern long gainP, gainI, gainD;								// TODO: debug

extern long magnetSensorKneeAddress;			// so we know if it "exists" ;)
extern long kneeBx, kneeBy, kneeBz, kneeTemp;					// TODO: debug
extern long sensorBx, sensorBy, sensorBz, sensorTemp;					// TODO: debug

extern long footPressureSensorAddress;			// so we know if it "exists" ;)
extern long footPressureReading;

extern long lastSensorUpdateTimeMS;								// TODO: debug

// ***********************************************************************************************************************

long UARTOutputEnabled = 1;	// disabling this will make the UART completely silent

char uartRXBufferDMA[UART_BUFFER_SIZE];
long uartRXReadPointer = 0;

char uartCMDBuffer[UART_BUFFER_SIZE];
long uartCMDPointer = 0;

#ifdef UART_SINGLE_WIRE
long uartRXDiscardCounter = 0;// as we use single-wire protocol, every char we send is also received.
// so for every char send, also "ingnore" one
#endif

char uartTXBuffer[UART_BUFFER_SIZE];
long uartTXWritePointer = 0;
long uartTXReadPointer = 0;

const char HEX_LOOKUP_TABLE[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8',
		'9', 'A', 'B', 'C', 'D', 'E', 'F' };

// ***********************************************************************************************************************
//void uartInit() {
//}
//void uartIterate() {
//}

// ***********************************************************************************************************************
void uartBusyWaitForTXDone() {
	while (uartTXWritePointer != uartTXReadPointer) {			// UART transmit
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

	while (((USART2->ISR) & BIT(6)) == 0) {
		asm volatile ("nop");
	};
}

// ***********************************************************************************************************************
inline void uartSendChar(char c) {
	uartTXBuffer[uartTXWritePointer++] = c;
	uartTXWritePointer &= UART_BUFFER_MASK;
}

void uartSendString(char *s) {
	while ((*s) != 0) {
		uartSendChar(*(s++));
	}
}

// ***********************************************************************************************************************
inline void uartSendCharDirect(char c) {
	if (UARTOutputEnabled) {
		while (((USART2->ISR) & BIT(7)) == 0) {
			asm volatile ("nop");
		};
		USART2->TDR = c;
#ifdef UART_SINGLE_WIRE
		uartRXDiscardCounter++;
#endif
	}
}

void uartSendStringDirect(char *s) {
	while ((*s) != 0) {
		uartSendCharDirect(*(s++));
	}
}

// ***********************************************************************************************************************
void uartSendHexLong(long hl) {
	uartSendChar(HEX_LOOKUP_TABLE[(hl >> 28) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hl >> 24) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hl >> 20) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hl >> 16) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hl >> 12) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hl >> 8) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hl >> 4) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hl) & 0xF]);
}

void uartSendHexShort(long hs) {
	uartSendChar(HEX_LOOKUP_TABLE[(hs >> 12) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hs >> 8) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hs >> 4) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hs) & 0xF]);
}

void uartSendHexByte(char hb) {
	uartSendChar(HEX_LOOKUP_TABLE[(hb >> 4) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hb) & 0xF]);
}

// ***********************************************************************************************************************
const unsigned char PARSE_HEX_LOOKUP[32] = { 0, 10, 11, 12, 13, 14, 15, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 0 };
#define parseHexDigit(c)  (PARSE_HEX_LOOKUP[((c) & 0x1F)])

// ***********************************************************************************************************************
//inline uint32_t parseHex(char hex) {
//	if ((hex>='0') && (hex<='9')) { return(hex-'0'); }
//	if ((hex>='a') && (hex<='f')) { return(hex-'a'+10); }
//	if ((hex>='A') && (hex<='F')) { return(hex-'A'+10); }
//	return(0);
//}
//
//uint32_t uartDecodeHexShort(char *c) {
//
//	long h;
//	h =
//}

// ***********************************************************************************************************************
void uartShowMotorSpeed() {

	long m;
	m = getMotorSpeed();

	uartSendString("motorSpeed: ");
	if (m >= 0) {
		uartSendChar('+');
		uartSendHexShort(m);
	} else {
		uartSendChar('-');
		uartSendHexShort(-m);
	}
	uartSendChar('\n');
}

// ***********************************************************************************************************************
void uartRXClearDMAForDiscardData(void) {// this function clears the DMA INPUT buffer IF (and only if) the chars will get discarded anyways
// i.e. because they have been an "echo" of the output
	long UART2_DMABytesAvailable = 0;

	UART2_DMABytesAvailable = (-((DMA1_Channel5->CNDTR) + uartRXReadPointer))
			& (UART_BUFFER_MASK);

#ifdef UART_SINGLE_WIRE
	while ((UART2_DMABytesAvailable) && (uartRXDiscardCounter)) {
#else
	while (UART2_DMABytesAvailable) {
#endif
//		char newChar = uartRXBufferDMA[uartRXReadPointer++];			// no need to read char, it's discarded anyways
		uartRXReadPointer++;
		uartRXReadPointer &= UART_BUFFER_MASK;
		UART2_DMABytesAvailable--;

#ifdef UART_SINGLE_WIRE
		uartRXDiscardCounter--;
#endif
	}
}

// ***********************************************************************************************************************
void uartShowHelp(void) {

	uartBusyWaitForTXDone();

	uartShowVersion();

	uartSendStringDirect("Available Commands:\n");
	uartRXClearDMAForDiscardData();

	uartSendStringDirect(
			"  !P=<>         set servo target position [0..4095]\n");
	uartSendStringDirect("  !P=OFF        set servo motor control off\n");
	uartRXClearDMAForDiscardData();
	uartSendStringDirect(
			"  !P=BLOCK      set servo motor control off, motor stiff\n");
	uartSendStringDirect(
			"  ?P            get current and (desired) servo position\n");
	uartRXClearDMAForDiscardData();
//	uartSendStringDirect("  !V()          set servo desired velocity\n");
//	uartSendStringDirect("  ?V            get servo current velocity\n");
//	uartRXClearDMAForDiscardData();

	if (magnetSensorKneeAddress) {
		uartSendStringDirect("  ?K            get magnetic reading from knee sensor\n");
	}

	if (footPressureSensorAddress) {
		uartSendStringDirect(
				"  ?F            get foot pressure sensor reading\n");
	}

	uartSendCharDirect('\n');
	uartRXClearDMAForDiscardData();

	uartSendStringDirect("  !C[P,I,D]=<>  set PID control gains\n");
	uartSendStringDirect("  ?C            get current PID control gains\n");
	uartSendCharDirect('\n');
	uartRXClearDMAForDiscardData();

	uartSendStringDirect("<> is a 16-bit unsigned int in HEX (4 char)\n");
	uartSendStringDirect("() is a 16-bit signed int in HEX (4 char)\n");
	uartSendCharDirect('\n');
	uartRXClearDMAForDiscardData();

	uartSendStringDirect(
			"  !L[-,+,.,=<>] set LED off/on/blinking/blink-frequency\n");
//	uartSendStringDirect("  ?S            send all sensor values (debug)\n");
	uartRXClearDMAForDiscardData();
	uartSendStringDirect(
			"  !U[+,-]       set UART output enabled / disabled\n");
	uartSendCharDirect('\n');
	uartRXClearDMAForDiscardData();

	uartSendStringDirect("  ?ID           report welcome message and ID\n");
	uartSendStringDirect("  RESET         restart servo\n");
	uartSendStringDirect(
			"  BOOTL         enter boot-loader for reprogramming\n");

	uartSendCharDirect('\n');
	uartRXClearDMAForDiscardData();
}

// ***********************************************************************************************************************
void passiveDuringReprogOfOtherServo(void) {

	long timeMemory;
	long dataCount;
	long UART2_BytesAvailable;

	// ************************************************************************************************** set UART2 to 57600, 8E1 to "listen" to reprog
	USART2->CR1 &= ~(0x00000001);							// disable USART2
															// Compute Baud rate: 32MHz / UARTDIV = 57600	--> 32MHz / 57600 = UARTDIV = 555.55 = 0x22C
	USART2->BRR = 0x22C;									// set 57600 Baud
	USART2->CR1 |= BIT(12) | BIT(10);// set 8+parity bits (12); enable parity check (10); set desired parity (9, even)
	USART2->CR1 |= 0x00000001;									// enable USART2

	// **************************************************************************************************
	while ((systemTimeMS & 0xFFFF) != (TIM21->CNT)) {// check if (1 ms) has elapsed
		systemTimeMS++;								// advance MS system time
	}
	timeMemory = systemTimeMS;
	dataCount = 0;

	while (timeMemory) {
		IWDG_CLEAR();					// clear watchdog, assuming all well :)

		if ((systemTimeMS & 0xFFFF) != (TIM21->CNT)) {// check if (1 ms) has elapsed
			systemTimeMS++;							// advance MS system time

			if ((systemTimeMS & 0x1F) == 0) {// toggle LED at about 16Hz (0x3F => toggle every 31ms)
				if ((PORTA->INP) & PORTA_LED_A) {
					PORTA->CLR = PORTA_LED_A;
				} else {
					PORTA->SET = PORTA_LED_A;
				}
			}

			if (dataCount < 64) {
				if ((systemTimeMS - timeMemory) > 30000) {// allow 30 seconds before timeout, when programming is not yet started
					timeMemory = 0;
				}
			} else {
				if ((systemTimeMS - timeMemory) > 2000) {// allow 2 seconds before timeout, when programming is ongoing
					timeMemory = 0;
				}
			}
		}

		UART2_BytesAvailable = (-((DMA1_Channel5->CNDTR) + uartRXReadPointer))
				& (UART_BUFFER_MASK);

		if (UART2_BytesAvailable) {
			char newChar = uartRXBufferDMA[uartRXReadPointer++];
			uartRXReadPointer &= UART_BUFFER_MASK;
			UART2_BytesAvailable--;

			if (newChar == 0x79) {
				dataCount++;
				timeMemory = systemTimeMS;
			}
		}

	}

	// **************************************************************************************************
	NVIC_SystemReset();								// reset for a clean restart

}

// ***********************************************************************************************************************
void uartParseCommand(char *cmd) {

#ifdef UART_SINGLE_WIRE
	if ((cmd[0] - '0') != SERVO_ID) {// on one-wire, only accept commands for THIS servo

		if (cmd[0] == '*') {// if this command is for all servos, delay some short time and continue
//			for (int i=0; i<10*SERVO_ID; i++) {					// wait servo-specific time as 10*10ms per ServoID number
//				sleepMS(10*SERVO_ID);
//				IWDG_CLEAR();									// clear watchdog, assuming all well :)
//			}
			IWDG_CLEAR();				// clear watchdog, assuming all well :)
			sleepMS(2 * SERVO_ID);
			IWDG_CLEAR();				// clear watchdog, assuming all well :)

		} else {

			if (cmd[1] == 'B') {// UNLESS it's a command to REPROGRAM someone else, then go to "passive reprog"
				if (cmd[2] == 'O') {
					if (cmd[3] == 'O') {
						if (cmd[4] == 'T') {
							if (cmd[5] == 'L') {
								if (cmd[6] == '\n') {
									passiveDuringReprogOfOtherServo();
								}
							}
						}
					}
				}
			}

			return;
		}
	}
	cmd++;							// skip servo ID, continue parsing command
#endif

	char cmdU[UART_BUFFER_SIZE];
	(void) strcpy(cmdU, cmd);
	(void) strupr(cmdU);

//	if (cmd[0]=='w') {
//		sensorWriteTest();
//		return;
//	}
//	if (cmd[0]=='m') {
//		sensorReadTest();
//		return;
//	}
//	if (cmd[0]=='t') {
//		sensorMathTest();
//		return;
//	}

//	if (cmd[0]=='i') {
//		uartSendString("toggling sensor\n");
//		uartBusyWaitForTXDone();
//
//		uartSendString("-tStart-\n");
//		uartBusyWaitForTXDone();
//
//		sensorsInit();
//
//		uartSendString("toggling sensor done\n");
//		uartBusyWaitForTXDone();
//
//		uartSendString("-tStop-\n");
//		uartBusyWaitForTXDone();
//
//		return;
//	}

//	if (strncmp(cmdU, "TEST", 4)==0) {								// TEST
//		u2test();
//		return;
//	}
//

//	if (strncmp(cmdU, "!ILOOP", 6)==0) {								// infinite loop to test IWDG
//		while (1) {asm volatile ("nop"); }
//		return;
//	}

	if (cmdU[0] == ',') {
		sleepMS(1000);				// sleep 1s will trigger WDT
	}

	if (cmdU[0] == '.') {
		__HAL_RCC_ADC1_CLK_ENABLE();

		for (int st = 0; st < 8; st++) {

			ADC1->SMPR &= (uint32_t) (~ADC_SMPR_SMPR);// Clear the old sampling time
			ADC1->SMPR |= st;						// Set the new sample time

			uartSendString("ST");
			uartSendHexByte(st);

			for (int i = 0; i < 3; i++) {
				HAL_ADC_Start(&hadc);		// start the on-chip ADC converter
				//long ret =
				HAL_ADC_PollForConversion(&hadc, 10);			// timeout in ms
				long r = HAL_ADC_GetValue(&hadc);
//				uartSendChar('0'+i);
				uartSendString(" - ");
//				uartSendHexByte(ret);
//				uartSendString(" - ");
				uartSendHexShort(r);
//				uartSendChar('\n');
			}
			uartSendChar('\n');

		}

		HAL_ADC_Stop(&hadc);
		return;
	}

	if (strncmp(cmdU, "!P=", 3) == 0) {			// Set Servo Position command

		if (strncmp((cmdU + 3), "OFF", 3) == 0) {				// Set Servo Off
			setMotorPositionControlOff();
			//uartSendString("-P OFF\n");
			return;
		}
		if (strncmp((cmdU + 3), "BLOCK", 5) == 0) {	// Set Servo Motors Blocked
			setMotorPositionControlOff();
			TIM2->CCR1 = 65535;
			TIM2->CCR2 = 65535;
			//uartSendString("-P BLOCKED\n");
			return;
		}

		long dSP = ((parseHexDigit(cmd[3])) << 12)
				+ ((parseHexDigit(cmd[4])) << 8) + (parseHexDigit(cmd[ 5]) << 4)
				+ (parseHexDigit(cmd[6]));

		if ((dSP >= 0) && (dSP < 4096)) {
			setDesiredMotorPosition(dSP);
			//uartSendString("-P=");
			//uartSendHexShort(dSP);
			//uartSendChar('\n');
		} else {
			//uartSendString("-P out of range\n");
		}
		return;
	}

	if (strncmp(cmdU, "?ID", 3) == 0) {	// Send welcome message and show servo ID
		uartBusyWaitForTXDone();
		uartShowVersion();
		return;
	}

	if (strncmp(cmdU, "?P", 2) == 0) {			// Get Servo Position command
		uartSendString("P-");
		uartSendHexByte(SERVO_ID);
		uartSendChar('-');
		uartSendHexShort(currentServoPosition);
		uartSendChar('-');
		uartSendHexShort(desiredServoPosition);
		uartSendChar('\n');
		return;
	}

	if (magnetSensorKneeAddress) {
		if (strncmp(cmdU, "?K", 2) == 0) {	// Get Knee Magnetic Flux command
			uartSendString("K-");
			uartSendHexByte(SERVO_ID);
			uartSendChar('-');
			uartSendHexShort(kneeBx);
			uartSendChar('-');
			uartSendHexShort(kneeBy);
			uartSendChar('\n');
			return;
		}
	}

	if (footPressureSensorAddress) {
		if (strncmp(cmdU, "?F", 2) == 0) {	// Get Foot Pressure value command
			uartSendString("F-");
			uartSendHexByte(SERVO_ID);
			uartSendChar('-');
			uartSendHexShort(footPressureReading);
			uartSendChar('\n');
			return;
		}
	}

	if (strncmp(cmdU, "!L", 2) == 0) {							// LED command?
		if (cmd[2] == '+') {
			setLED(1, LED_DEFAULT_BRIGHTNESS);
			//uartSendString("-L+\n");
			return;
		}
		if (cmd[2] == '-') {
			setLED(0, LED_DEFAULT_BRIGHTNESS);
			//uartSendString("-L-\n");
			return;
		}
		if (cmd[2] == '.') {
			setLEDBlink(250, LED_DEFAULT_BRIGHTNESS);
			//uartSendString("-L.\n");
			return;
		}
		if (cmd[2] == '=') {
			long time = ((parseHexDigit(cmd[3])) << 12)
					+ ((parseHexDigit(cmd[4])) << 8)
					+ (parseHexDigit(cmd[ 5]) << 4) + (parseHexDigit(cmd[6]));
			setLEDBlink(time, LED_DEFAULT_BRIGHTNESS);
			//uartSendString("-L");
			//uartSendHexShort(time);
			//uartSendChar('\n');
			return;
		}
		//uartSendString("-L parsing error\n");
		return;
	}

	if (strncmp(cmdU, "!C", 2) == 0) {				// Set PID gain parameter
		long gain = ((parseHexDigit(cmd[4])) << 12)
				+ ((parseHexDigit(cmd[5])) << 8) + (parseHexDigit(cmd[ 6]) << 4)
				+ (parseHexDigit(cmd[7]));
		if (cmdU[2] == 'P') {
			gainP = gain;
			//uartSendString("-CP=");
			//uartSendHexShort(gainP);
			//uartSendChar('\n');
		} else {
			if (cmdU[2] == 'I') {
				gainI = gain;
				//uartSendString("-CI=");
				//uartSendHexShort(gainI);
				//uartSendChar('\n');
			} else {
				if (cmdU[2] == 'D') {
					gainD = gain;
					//uartSendString("-CD=");
					//uartSendHexShort(gainD);
					//uartSendChar('\n');
				} else {
					//uartSendString("-C parsing error\n");
				}
			}
		}
		return;
	}

	if (strncmp(cmdU, "?C", 2) == 0) {				// Get PID gain parameter
		uartSendString("C-");
		uartSendHexByte(SERVO_ID);
		uartSendChar('-');
		uartSendHexShort(gainP);
		uartSendChar('-');
		uartSendHexShort(gainI);
		uartSendChar('-');
		uartSendHexShort(gainD);
		uartSendChar('\n');
		return;
	}

	if (strncmp(cmdU, "!U", 2) == 0) {				// set UART output enable
		if (cmd[2] == '+') {
			UARTOutputEnabled = 1;
			//uartSendString("-U+\n");
		} else {
			uartBusyWaitForTXDone();
			UARTOutputEnabled = 0;
		}
		return;
	}

	if (strncmp(cmd, "RESET", 5) == 0) {							// RESET?
		IWDG_CLEAR();					// clear watchdog, assuming all well :)
		setMotorSpeed(0);
		uartBusyWaitForTXDone();
		uartSendStringDirect("-RESET\n");
		uartBusyWaitForTXDone();

		IWDG_CLEAR();					// clear watchdog, assuming all well :)
		RCC->CSR |= BIT(23);// clear reset flags (memory of reset source) (so we're not ending up in the bootloader)
		NVIC_SystemReset();
	}

	if (strncmp(cmd, "BOOTL", 5) == 0) {						// BOOTLOADER?
		setMotorSpeed(0);
		uartBusyWaitForTXDone();
		uartSendStringDirect("-BOOTLOADER\n");
		uartBusyWaitForTXDone();
		sleepMS(50);							// just to be sure
		uartBusyWaitForTXDone();

		IWDG->KR = 0x0000CCCC;			// enable IWDG (independent watch-dog)
		IWDG->KR = 0x00005555;			// enable register access
		IWDG->PR = 0x07;// prescaler 256 --> 32Khz/256 --> 125Hz --> 8msec ticks
		IWDG->RLR = 1;					// re-load register, load one tick
		while (IWDG->SR != 0x00000000) {
			asm volatile ("nop");
		}		// wait for WD to finish
		IWDG->KR = 0x0000AAAA;			// start WDT

		while (1) {
			asm volatile ("nop");
		}; 					// will trigger Watchdog -> go into BL
//		stopAndEnterBootLoader();
	}

	if (strncmp(cmd, "REPROG", 6) == 0) {						// BOOTLOADER?
		setMotorSpeed(0);
		uartBusyWaitForTXDone();
		uartSendStringDirect("-RP-BOOTLOADER\n");
		uartBusyWaitForTXDone();
		sleepMS(50);							// just to be sure
		uartBusyWaitForTXDone();

		stopAndEnterBootLoader();
	}

	if (strncmp(cmd, "??", 2) == 0) {								// show help
		uartShowHelp();
		return;
	}
	uartSendString("?: ");									// unknown command
	uartSendString(cmd);
}

// ***********************************************************************************************************************
void showPIDGain(void) {
	uartSendString("PID: ");
	uartSendHexShort(gainP);
	uartSendChar('-');
	uartSendHexShort(gainI);
	uartSendChar('-');
	uartSendHexShort(gainD);
	uartSendChar('\n');
}

void uartReceiveChar(char c) {

#ifndef UART_SINGLE_WIRE

	if (c=='[') {
		gainP--;
		showPIDGain();
		return;
	}
	if (c==']') {
		gainP++;
		showPIDGain();
		return;
	}
	if (c=='{') {
		gainD--;
		showPIDGain();
		return;
	}
	if (c=='}') {
		gainD++;
		showPIDGain();
		return;
	}

	if (c=='(') {
		gainI--;
		showPIDGain();
		return;
	}
	if (c==')') {
		gainI++;
		showPIDGain();
		return;
	}


//	if (c=='(') {
//		setDesiredMotorPosition(0x600);
//		uartSendString("-P=0600\n");
//		return;
//	}
//	if (c==')') {
//		setDesiredMotorPosition(0xA00);
//		uartSendString("-P=0A00\n");
//		return;
//	}



//	if (c=='#') {
//		uartSendString("TIM21:       ");
//		uartSendHexShort(TIM21->CNT);
//		uartSendChar('\n');
//		uartSendString("SysTime: ");
//		uartSendHexLong(systemTimeMS);
//		uartSendChar('\n');
//		return;
//	}

	if (c=='@') {
		uartSendString("ST: ");
		uartSendHexLong(systemTimeMS);
		uartSendChar('-');
		uartSendHexLong(lastSensorUpdateTimeMS);
		uartSendChar('\n');

//		uartSendString("A: ");
//		uartSendHexLong(desiredServoPosition);
//		uartSendChar('-');
//		uartSendHexLong(currentServoPosition);
//		uartSendChar('-');
//		uartSendHexLong(positionError);
//		uartSendChar('-');
//		uartSendHexLong(positionErrorD);
//		uartSendChar('-');
//		uartSendHexLong(positionErrorI);
//		uartSendChar('\n');
		return;
	}

	if (c=='>') {
//		setMotorSpeedDelta(+256);
//		uartShowMotorSpeed();

		desiredServoPosition += 16;

		uartBusyWaitForTXDone();
		uartSendStringDirect("-P=");
		uartSendHexShort(desiredServoPosition);
		uartBusyWaitForTXDone();
		uartSendCharDirect('\n');

		return;
	}
	if (c=='<') {

//		setMotorSpeedDelta(-256);
//		uartShowMotorSpeed();

		desiredServoPosition -= 16;

		uartBusyWaitForTXDone();
		uartSendStringDirect("-P=");
		uartSendHexShort(desiredServoPosition);
		uartBusyWaitForTXDone();
		uartSendCharDirect('\n');

		return;
	}
#endif

	if ((c == '\n') || (c == '\r')) {
		uartCMDBuffer[uartCMDPointer++] = '\n';
		uartCMDBuffer[uartCMDPointer++] = 0;
#ifndef UART_SINGLE_WIRE
		uartSendChar('\n');
#endif
		uartParseCommand(uartCMDBuffer);
		uartCMDPointer = 0;
		return;
	}

	if (c == 8) {								// BACKSPACE
		if (uartCMDPointer) {
			uartCMDPointer--;
#ifndef UART_SINGLE_WIRE
			uartSendChar(8);
#endif
		}
		return;
	}

	if (uartCMDPointer < (UART_BUFFER_SIZE - 2)) {
		uartCMDBuffer[uartCMDPointer++] = c;
	}
#ifndef UART_SINGLE_WIRE
	uartSendChar(c);
#endif
}

