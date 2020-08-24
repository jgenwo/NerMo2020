#include <mouseServo.h>

// this file will (ok should, but ISN'T) ALWAYS be recompiled to reflect the current __TIME__ and __DATE__

// ***********************************************************************************************************************
void uartShowVersion(void) {
	uartSendStringDirect("\n\nHBP Mouse Servo Control V");
	uartSendStringDirect(SOFTWARE_VERSION);

	uartSendStringDirect(": ");
	uartSendStringDirect(__DATE__);
	uartSendStringDirect(", ");
	uartSendStringDirect(__TIME__);
#ifdef DEBUG
	uartSendStringDirect("  DEBUG");
#endif
	uartSendCharDirect('\n');


#ifdef UART_SINGLE_WIRE
	uartSendStringDirect("SingleWire ");
#endif
	uartSendStringDirect("Servo ID: ");
	uartSendHexByte(SERVO_ID & 0xFF);
	uartBusyWaitForTXDone();
	uartSendCharDirect('\n');

}
