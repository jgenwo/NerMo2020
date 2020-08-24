#include <mouseSpine.h>


// ***********************************************************************************************************************
void uartShowVersion(void) {
	  uartSendStringDirect("\n\nHBP Mouse Spine V");
	  uartSendStringDirect(SOFTWARE_VERSION);

	  uartSendStringDirect(": ");
	  uartSendStringDirect(__DATE__);
	  uartSendStringDirect(", ");
	  uartSendStringDirect(__TIME__);

#ifdef DEBUG
	  uartSendStringDirect("  DEBUG");
#endif

	  uartSendCharDirect('\n');
}
