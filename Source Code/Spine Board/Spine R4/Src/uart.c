/*
 * uart.c
 *
 */

#include <mouseSpine.h>
#include <string.h>

// ***********************************************************************************************************************	to RPIZW (UART 3)
char uart3RXBufferDMA[UART_BUFFER_SIZE]		__attribute__((section (".SEC_SRAM1"))) __attribute__((aligned(UART_BUFFER_SIZE)));
char uart3RXBuffer[UART_BUFFER_SIZE]		__attribute__((section (".SEC_SRAM1"))) __attribute__((aligned(UART_BUFFER_SIZE)));
long uart3RXReadPointerDMA=0, uart3RXWritePointer=0;

char uart3TXBuffer[UART_BUFFER_SIZE];
long uart3TXWritePointer=0, uart3TXReadPointer=0;


// ***********************************************************************************************************************	to Servo Chain Left (UART 5)
char uartLRXBufferDMA[UART_BUFFER_SIZE]	__attribute__((section (".SEC_SRAM1"))) __attribute__((aligned(UART_BUFFER_SIZE)));
char uartLRXBuffer[UART_BUFFER_SIZE]		__attribute__((section (".SEC_SRAM1"))) __attribute__((aligned(UART_BUFFER_SIZE)));
long uartLRXReadPointerDMA=0, uartLRXWritePointer=0, uartLRXDiscardCounter=0;

char uartLTXBuffer[UART_BUFFER_SIZE];
long uartLTXWritePointer=0, uartLTXReadPointer=0;

// ***********************************************************************************************************************	to Servo Chain Right (UART 4)
char uartRRXBufferDMA[UART_BUFFER_SIZE]	__attribute__((section (".SEC_SRAM1"))) __attribute__((aligned(UART_BUFFER_SIZE)));
char uartRRXBuffer[UART_BUFFER_SIZE]		__attribute__((section (".SEC_SRAM1"))) __attribute__((aligned(UART_BUFFER_SIZE)));
long uartRRXReadPointerDMA=0, uartRRXWritePointer=0, uartRRXDiscardCounter=0;

char uartRTXBuffer[UART_BUFFER_SIZE];
long uartRTXWritePointer=0, uartRTXReadPointer=0;

// ***********************************************************************************************************************	to Servo Chain Spine (UART 2)
char uartSRXBufferDMA[UART_BUFFER_SIZE]	__attribute__((section (".SEC_SRAM1"))) __attribute__((aligned(UART_BUFFER_SIZE)));
char uartSRXBuffer[UART_BUFFER_SIZE]		__attribute__((section (".SEC_SRAM1"))) __attribute__((aligned(UART_BUFFER_SIZE)));
long uartSRXReadPointerDMA=0, uartSRXWritePointer=0, uartSRXDiscardCounter=0;

char uartSTXBuffer[UART_BUFFER_SIZE];
long uartSTXWritePointer=0, uartSTXReadPointer=0;


// ***********************************************************************************************************************
const char HEX_LOOKUP_TABLE[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};


// ***********************************************************************************************************************
//void uartInit() {
//}
//void uartIterate() {
//}

// ***********************************************************************************************************************
void uartBusyWaitForTXDone() {
	while (uart3TXWritePointer != uart3TXReadPointer) {									// UART transmit
		if ((USART3->ISR) & BIT(7)) {
			USART3->TDR = uart3TXBuffer[uart3TXReadPointer++];
			uart3TXReadPointer &= UART_BUFFER_MASK;
		}
	}

	while (((USART3->ISR) & BIT(6))==0) { asm volatile ("nop"); };
}

// ***********************************************************************************************************************		RPIZW
inline void uartSendChar(char c) {
//	while (((USART3->ISR) & BIT(7)) == 0) { asm("nop"); };
//	USART3->TDR = c;
	uart3TXBuffer[uart3TXWritePointer++]=c;
	uart3TXWritePointer &= UART_BUFFER_MASK;
}

void uartSendString(char *s) {
	while ((*s) != 0) {uartSendChar(*(s++));}
}

// ***********************************************************************************************************************		Left
inline void uartLSendChar(char c) {
	uartLTXBuffer[uartLTXWritePointer++]=c;
	uartLTXWritePointer &= UART_BUFFER_MASK;
}
void uartLSendString(char *s) {
	while ((*s) != 0) {uartLSendChar(*(s++));}
}

// ***********************************************************************************************************************		Right
inline void uartRSendChar(char c) {
	uartRTXBuffer[uartRTXWritePointer++]=c;
	uartRTXWritePointer &= UART_BUFFER_MASK;
}
void uartRSendString(char *s) {
	while ((*s) != 0) {uartRSendChar(*(s++));}
}

// ***********************************************************************************************************************		Spine
inline void uartSSendChar(char c) {
	uartSTXBuffer[uartSTXWritePointer++]=c;
	uartSTXWritePointer &= UART_BUFFER_MASK;
}
void uartSSendString(char *s) {
	while ((*s) != 0) {uartSSendChar(*(s++));}
}


// ***********************************************************************************************************************
inline void uartSendCharDirect(char c) {
	while (((USART3->ISR) & BIT(7)) == 0) { asm volatile ("nop"); };
	USART3->TDR = c;
}

void uartSendStringDirect(char *s) {
	while ((*s) != 0) {uartSendCharDirect(*(s++));}
}


// ***********************************************************************************************************************
void uartDiscardAllIncomingData(void) {

	long UART_BytesAvailable;

	UART_BytesAvailable = (-((DMA1_Stream1->NDTR) + uart3RXReadPointerDMA)) & (UART_BUFFER_MASK);				// U3 - RasPIZW UART receive (UART3)
	uart3RXReadPointerDMA += UART_BytesAvailable;
	uart3RXReadPointerDMA &= UART_BUFFER_MASK;

	UART_BytesAvailable = (-((DMA1_Stream5->NDTR) + uartSRXReadPointerDMA)) & (UART_BUFFER_MASK);				// U2 to Servo Chain Spine (UART 2)
	uartSRXReadPointerDMA += UART_BytesAvailable;
	uartSRXReadPointerDMA &= UART_BUFFER_MASK;

	UART_BytesAvailable = (-((DMA1_Stream2->NDTR) + uartRRXReadPointerDMA)) & (UART_BUFFER_MASK);				// U4 to Servo Chain Left (UART 4)
	uartRRXReadPointerDMA += UART_BytesAvailable;
	uartRRXReadPointerDMA &= UART_BUFFER_MASK;

	UART_BytesAvailable = (-((DMA1_Stream0->NDTR) + uartLRXReadPointerDMA)) & (UART_BUFFER_MASK);				// U5 to Servo Chain Right (UART 5)
	uartLRXReadPointerDMA += UART_BytesAvailable;
	uartLRXReadPointerDMA &= UART_BUFFER_MASK;
}

// ***********************************************************************************************************************
void uartSendHexLong(long hl) {
	uartSendChar(HEX_LOOKUP_TABLE[(hl>>28) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hl>>24) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hl>>20) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hl>>16) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hl>>12) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hl>> 8) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hl>> 4) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hl    ) & 0xF]);
}

void uartSendHexShort(long hs) {
	uartSendChar(HEX_LOOKUP_TABLE[(hs>>12) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hs>> 8) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hs>> 4) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hs    ) & 0xF]);
}

void uartSendHexByte(char hb) {
	uartSendChar(HEX_LOOKUP_TABLE[(hb>> 4) & 0xF]);
	uartSendChar(HEX_LOOKUP_TABLE[(hb    ) & 0xF]);
}


// ***********************************************************************************************************************
const unsigned char PARSE_HEX_LOOKUP[32] = { 0,10,11,12,13,14,15, 0, 0, 0, 0, 0, 0, 0, 0, 0,	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 0};
#define parseHexDigit(c)  (PARSE_HEX_LOOKUP[((c) & 0x1F)])



// ***********************************************************************************************************************
void uartShowHelp(void) {

	uartShowVersion();

	uartSendStringDirect("Available Commands:\n");

	uartSendStringDirect("  :ixxx       forward xxx to servo chain i [0..2]\n");
	uartSendCharDirect('\n');

	uartSendStringDirect("  !PS[+,-]    set power to servos\n");
	uartSendStringDirect("  !PR[+,-][D] set power to RPIZW (D:delayed by 30sec)\n");
	uartSendCharDirect('\n');

	uartSendStringDirect("  !L[-,+,.]   set LED off/on/blinking\n");
	uartSendStringDirect("  ?S          send all sensor values (debug)\n");
	uartSendCharDirect('\n');

	uartSendStringDirect("  RESET       restart spine\n");
	uartSendStringDirect("  BOOTL       enter boot-loader for reprogramming\n");
	uartSendStringDirect("  PROG<C><S>  reprogram servo <S>[0..4] on chain <C>[0..2,L,R,S]\n");

	uartSendCharDirect('\n');
}


// ***********************************************************************************************************************
void uartParseCommand(char *cmd) {

	if (cmd[0] == ':') {
		char target = cmd[1]-'0';											// try to parse servo chain by number
		if (target==0) { uartLSendString(cmd+2); return; }
		if (target==1) { uartRSendString(cmd+2); return; }
		if (target==2) { uartSSendString(cmd+2); return; }

																			// try to parse servo chain by name
		if (cmd[1]=='L') { uartLSendString(cmd+2); return; }
		if (cmd[1]=='R') { uartRSendString(cmd+2); return; }
		if (cmd[1]=='S') { uartSSendString(cmd+2); return; }

		uartSendString("forward wrong id ([0..2] or 1-letter name [L,R,S])... ");		// show error
		uartSendString(cmd);											// this string begins with ':' and ends with '\n' :)
		return;
	}

	char cmdU[UART_BUFFER_SIZE];
	(void) strcpy(cmdU, cmd);
	(void) strupr(cmdU);


	if (strncmp(cmdU, "!P", 2)==0) {									// POWER command
		if (cmd[2]=='-') {												// power ALL off immediately
			PORTC->CLR = PORTC_ENABLE_RPIZW_POWER;
			PORTB->CLR = PORTB_ENABLE_SERVOS_POWER;
			uartSendString("-P-\n");
			return;
		}

		if (cmdU[2]=='R') {
			if (cmdU[4] == 'D') {										// delay requested? wait 30sec
				uartBusyWaitForTXDone();
				uartSendStringDirect("Waiting 30sec before power change\n");
				for (int i=0; i<300; i++) {										// 300 * 100ms with LED toggling
					sleepMS(50);
					PORTA->CLR = PORTA_LED_A;
					sleepMS(50);
					PORTA->SET = PORTA_LED_A;
				}
			}

			if (cmd[3]=='-') {
				PORTC->CLR = PORTC_ENABLE_RPIZW_POWER;
				uartSendString("-PR-\n");
			} else {
				PORTC->SET = PORTC_ENABLE_RPIZW_POWER;
				uartSendString("-PR+\n");
			}
			return;
		}

		if (cmd[2]=='S') {
			if (cmd[3]=='-') {
				PORTB->CLR = PORTB_ENABLE_SERVOS_POWER;
				uartSendString("-PS-\n");
			} else {
				PORTB->SET = PORTB_ENABLE_SERVOS_POWER;
				uartSendString("-PS+\n");
			}
			return;
		}

		PORTC->SET = PORTC_ENABLE_RPIZW_POWER;
		PORTB->SET = PORTB_ENABLE_SERVOS_POWER;
		uartSendString("-P+\n");
		return;
	}

	if (strncmp(cmdU, "!L", 2)==0) {									// LED command?
		uartBusyWaitForTXDone();
		if (cmd[2] == '-') {
			setLEDcounter(0);
			PORTA->CLR = PORTA_LED_A;
			uartSendString("-L-\n");
			return;
		}
		if (cmd[2] == '+') {
			setLEDcounter(0);
			PORTA->SET = PORTA_LED_A;
			uartSendString("-L+\n");
			return;
		}
		setLEDcounter(1);
		uartSendString("-L.\n");
		return;
	}

	if (strncmp(cmdU, "RESET", 5)==0) {									// RESET?
		uartSendString("Reset\n");
		uartBusyWaitForTXDone();
		NVIC_SystemReset();
	}

	if (strncmp(cmdU, "BOOTL", 5)==0) {									// BOOTLOADER?
		uartSendString("Entering Bootloader\n");
		uartBusyWaitForTXDone();

//		enterBootLoader();

		IWDG->KR = 0x0000CCCC;			// enable IWDG (independent watch-dog)
		IWDG->KR = 0x00005555;			// enable register access
		IWDG->PR = 0xFF;				// prescaler 256 --> 32Khz/256 --> 125Hz --> 8msec ticks
		IWDG->RLR = 1;					// re-load time (~8ms)
		while (IWDG->SR != 0x00000000) { asm volatile ("nop"); }		// wait for WD to finish
		IWDG->KR = 0x0000AAAA;			// trigger WDT

		while (1) { asm volatile ("nop"); }; // will trigger Watchdog -> go into BL
		//	enterBootLoader();
	}

	if (strncmp(cmdU, "PROG", 4)==0) {									// Start Reprogramming on Servo Chain
		if ((cmd[5] >= '0') && (cmd[5] <= '4')) {							// valid servo ID given?
			if ((cmd[4]=='0') || (cmd[4]=='L')) {								// chain Left?
				uartSendChar('L');
				uartSendChar('-');
				uartSendChar(cmd[4]);
				uartSendChar('-');
				uartSendChar(cmd[5]);
				uartSendChar('\n');
				uartForwardReprogramServoChainLeft(cmd[5]);						// call reprogramming
				return;
			}

			if ((cmd[4]=='1') || (cmd[4]=='R')) {								// chain Right?
				uartSendChar('R');
				uartSendChar('-');
				uartSendChar(cmd[4]);
				uartSendChar('-');
				uartSendChar(cmd[5]);
				uartSendChar('\n');
				uartForwardReprogramServoChainRight(cmd[5]);					// call reprogramming
				return;
			}

			if ((cmd[4]=='2') || (cmd[4]=='S')) {								// chain Spine?
				uartSendChar('S');
				uartSendChar('-');
				uartSendChar(cmd[4]);
				uartSendChar('-');
				uartSendChar(cmd[5]);
				uartSendChar('\n');
				uartForwardReprogramServoChainSpine(cmd[5]);					// call reprogramming
				return;
			}
		}

		uartSendString("PROG: error parsing parameters\n");
		return;
	}


	if (strncmp(cmd, "??", 2)==0) {										// show help
		uartShowHelp();
		return;
	}
	uartSendString("?: ");												// unknown command
	uartSendString(cmd);
}
