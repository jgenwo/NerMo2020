/*
 * sensors.c
 *
 */

#include <math.h>
#include <mouseServo.h>

#define SENSORSM_ADVANCE_STATE()		{ sensorSM++;}											// advance to next state
#define SENSORSM_NEXT_BLOCK()			{ sensorSM = ((sensorSM & 0xFFFFFF00) + 256); }			// advance to next block

long magnetSensorKneeAddress = 0; 					// will contain the I2C address of an optional external magnet sensor
long footPressureSensorAddress = 0;					// will contain the I2C address of an optional external foot pressure sensor

long lastSensorUpdateTimeMS = 0;					// at this "SystemTimeMS" we obtained the last sensor reading. Used to check if sensor alive

long sensorSM = 0;									// sensor State Machine

long currentServoPosition = 2048;					// gives the current sensor position from 0 .. 4095		(12 bit)
long sensorReadDataI2C[7];							// this is "collected" data from the sensor

long sensorBx, sensorBy, sensorBz, sensorTemp;		// we shall remember this for debug
float sensorBxf, sensorByf, sensorAngleF;			// temporary storage as "float" but needed in different iterations

long footPressureReading;							// return value from pressure ADC

long magIndexOld = 0;
long magIndexNew = 0;

long kneeBx, kneeBy, kneeBz, kneeTemp;				// we shall remember this

long kneeMagIndexOld = 0;
long kneeMagIndexNew = 0;

int SMrunning = 0;

// ***********************************************************************************************************************
void I2C_SetRead(void) {
	I2C1->CR2 |= BIT(10);
}
void I2C_SetWrite(void) {
	I2C1->CR2 &= ~(BIT(10));
}

void I2C_SetAddress(unsigned long sa) {
	I2C1->CR2 &= ~(0x000000FE);
	I2C1->CR2 |= ((sa & 0x7F) << 1);
}

void I2C_SetNumberOfBytesToTransmit(unsigned long nb) {
	I2C1->CR2 &= ~(0x00FF0000);
	I2C1->CR2 |= ((nb & 0xFF) << 16);
}

// ***********************************************************************************************************************
void I2CWriteByteSequence(char i2cAddress, char byteCount, char *c) {
	long I2C_ISR = 0;
	long errorFlag = 0;
	long timeOut = 0x00000FFF;

	I2C_SetAddress(i2cAddress);					// write this address (compass on servo PCB)
	I2C_SetWrite();								// write data
	I2C_SetNumberOfBytesToTransmit(byteCount);	// set how many bytes to be transmitted
	I2C1->CR2 |= BIT(25);						// automatic end mode; (i.e. automatically send "stop" after number of bytes are transmitted)
	I2C1->TXDR = *c++;							// this is the first byte to be sent
	byteCount--;

	I2C1->CR2 |= I2C_CR2_START;					// generate I2C start condition

	while ((I2C1->CR2 & I2C_CR2_START) && (--timeOut)) {
		asm volatile ("nop");					// wait for start bit to get cleared (independent of success)
	}
	if (timeOut == 0) {
		errorFlag |= 8;
	}

	while ((((I2C_ISR)
			& (I2C_ISR_TC | I2C_ISR_NACKF | I2C_ISR_STOPF | I2C_ISR_BERR)) == 0)
			&& (--timeOut)) {
		I2C_ISR = (I2C1->ISR);

		if (((I2C_ISR) & (I2C_ISR_TXE)) && (byteCount)) {
			I2C1->TXDR = *c++;					// this is the next byte to be sent
			byteCount--;
		}
	}
	if (timeOut == 0) {
		errorFlag |= 16;
	}
//	uartSendStringDirect("W-I2C_ISR: ");
//	uartSendHexLong(I2C_ISR);
//	uartBusyWaitForTXDone();
//	uartSendCharDirect('\n');

//	uartSendStringDirect("W-Timeout: ");
//	uartSendHexLong(timeOut);
//	uartBusyWaitForTXDone();
//	uartSendCharDirect('\n');

	if ((I2C_ISR) & (I2C_ISR_TC)) {
//		uartSendStringDirect("success\n");
	}

	if ((I2C_ISR) & (I2C_ISR_STOPF)) {
		I2C1->ICR |= I2C_ICR_STOPCF;
	} else {
		errorFlag |= 1;
	}

	if ((I2C_ISR) & (I2C_ISR_BERR)) {
		I2C1->ICR |= I2C_ICR_BERRCF;
		errorFlag |= 2;
	}

	if ((I2C_ISR) & (I2C_ISR_NACKF)) {
		I2C1->ICR |= I2C_ICR_NACKCF;
		errorFlag |= 4;
	}

	if (errorFlag) {
		I2C1->CR2 |= I2C_CR2_STOP;// if an error occurred, just to be sure, generate a STOP
		sleepMS(1);
		I2C1->ICR |=
				BIT(
						3) | BIT(4) | BIT(5) | BIT(8) | BIT(9) | BIT(10) | BIT(11) | BIT(12);// just to be sure, clear ALL flags

//		uartSendStringDirect("I2C-W-Err: ");
//		uartSendCharDirect('0'+errorFlag);
//		uartSendCharDirect('\n');
	}
}

// ***********************************************************************************************************************
void I2CReadByteSequence(char i2cAddress, char byteCount, char *c) {
	long I2C_ISR = 0;
	long errorFlag = 0;
	long timeOut = 0x00000FFF;

	I2C_SetAddress(i2cAddress);	// read from this address (compass on servo PCB)

	I2C_SetRead();									// read data

	I2C_SetNumberOfBytesToTransmit(byteCount);// set how many bytes to be transmitted
	I2C1->CR2 |= BIT(25);// automatic end mode; (i.e. automatically send "end" after number of bytes are transmitted)

	I2C1->CR2 |= I2C_CR2_START;					// generate I2C start condition

	while ((I2C1->CR2 & I2C_CR2_START) && (--timeOut)) {
		asm volatile ("nop");
	}		// wait for start bit to get cleared (independent of success)
	if (timeOut == 0) {
		errorFlag |= 8;
	}

	while ((((I2C_ISR)
			& (I2C_ISR_TC | I2C_ISR_NACKF | I2C_ISR_STOPF | I2C_ISR_BERR)) == 0)
			&& (--timeOut)) {
		I2C_ISR = (I2C1->ISR);

		if (I2C_ISR & I2C_ISR_RXNE) {
			if (byteCount) {
				*c++ = I2C1->RXDR;						// read data
				byteCount--;
			} else {
				(int) I2C1->RXDR;
				errorFlag |= 32;
			}
		}
	}
	if (timeOut == 0) {
		errorFlag |= 16;
	}

//	uartSendStringDirect("R-Timeout: ");
//	uartSendHexLong(timeOut);
//	uartBusyWaitForTXDone();
//	uartSendCharDirect('\n');

	if ((I2C_ISR) & (I2C_ISR_TC)) {
	}

	if ((I2C_ISR) & (I2C_ISR_STOPF)) {
		I2C1->ICR |= I2C_ICR_STOPCF;
	} else {
		errorFlag |= 1;
	}

	if ((I2C_ISR) & (I2C_ISR_BERR)) {
		I2C1->ICR |= I2C_ICR_BERRCF;
		errorFlag |= 2;
	}

	if ((I2C_ISR) & (I2C_ISR_NACKF)) {
		I2C1->ICR |= I2C_ICR_NACKCF;
		errorFlag |= 4;
	}

	if (errorFlag) {
		I2C1->CR2 |= I2C_CR2_STOP;// if an error occurred, just to be sure, generate a STOP
		sleepMS(1);
		I2C1->ICR |=
				BIT(
						3) | BIT(4) | BIT(5) | BIT(8) | BIT(9) | BIT(10) | BIT(11) | BIT(12);// just to be sure, clear ALL flags

//		uartSendStringDirect("I2C-R-Err: ");
//		uartSendCharDirect('0'+errorFlag);
//		uartSendCharDirect('\n');
	}
}

// ***********************************************************************************************************************
unsigned long pingSensor(unsigned int address) {
	long I2C_ISR = 0;
	long timeOut = 0x00000FFF;
	long errorFlag = 0;

	I2C_SetAddress(address);						// read from this address
	I2C_SetWrite();									// write data
	I2C_SetNumberOfBytesToTransmit(1);	// set how many bytes to be transmitted
	I2C1->CR2 |= BIT(25);// automatic end mode; (i.e. automatically send "end" after number of bytes are transmitted)
	I2C1->TXDR = 0;							// this is the first byte to be sent

	I2C1->CR2 |= I2C_CR2_START;					// generate I2C start condition

	while ((I2C1->CR2 & I2C_CR2_START) && (--timeOut)) {
		asm volatile ("nop");
	}		// wait for start bit to get cleared (independent of success)
	if (timeOut == 0) {
		errorFlag |= 8;
	}

	while ((((I2C_ISR)
			& (I2C_ISR_TC | I2C_ISR_NACKF | I2C_ISR_STOPF | I2C_ISR_BERR)) == 0)
			&& (--timeOut)) {
		I2C_ISR = (I2C1->ISR);
	}
	if (timeOut == 0) {
		errorFlag |= 16;
	}

	if ((I2C_ISR) & (I2C_ISR_STOPF)) {
		I2C1->ICR |= I2C_ICR_STOPCF;
	} else {
		errorFlag |= 1;
	}

	if ((I2C_ISR) & (I2C_ISR_BERR)) {
		I2C1->ICR |= I2C_ICR_BERRCF;
		errorFlag |= 2;
	}

	if ((I2C_ISR) & (I2C_ISR_NACKF)) {
		I2C1->ICR |= I2C_ICR_NACKCF;
		errorFlag |= 4;
	}

	if (errorFlag) {
		I2C1->CR2 |= I2C_CR2_STOP;// if an error occurred, just to be sure, generate a STOP
		sleepMS(1);
		I2C1->ICR |=
				BIT(
						3) | BIT(4) | BIT(5) | BIT(8) | BIT(9) | BIT(10) | BIT(11) | BIT(12);// just to be sure, clear ALL flags
	}

//	uartSendStringDirect("ping: ");
//	uartSendCharDirect('0' + errorFlag);
//	uartSendCharDirect('\n');
//
	return (errorFlag);
}

// ***********************************************************************************************************************
void configureKneeMagnetSensor(unsigned long address) {
	char i2c_tx_cmd[17];

	// Sensor Reset on Startup
	for(long i = 0; i < 2; i++){
		i2c_tx_cmd[0] = 0xFF;
		I2CWriteByteSequence(address, 1, i2c_tx_cmd); // S Address Write-bit ACK FF P
	}

	for(long i = 0; i < 2; i++){
		i2c_tx_cmd[0] = 0x00;
		I2CWriteByteSequence(address, 1, i2c_tx_cmd); // S Address Write-bit ACK 00 P
	}

	sleepMS(30); 			// wait

	// now configure sensor
	i2c_tx_cmd[0] = 0b00010000; 	// no trigger | start writing at register 0b10000
	i2c_tx_cmd[1] = 0b11000000;		// disable temp | disable z | no trigger (2 bit) | 12bit | no temperature compensation (2bit) | even parity
	i2c_tx_cmd[2] = 0b00011111; 	// odd parity | address (2 bit) | 1 byte read | no CA | no INT | Fast Mode (2 bit)
	I2CWriteByteSequence(address, 3, i2c_tx_cmd); // S Address Write-bit ACK byte0 ACK byte1 ACK byte2 P
}

// ***********************************************************************************************************************
void configureFootPressureSensor(unsigned long address) {
	char i2c_tx_cmd[4];

	// ***********************************************************************************************************************	// power up sensor
	i2c_tx_cmd[0] = 0;
	I2CWriteByteSequence(address, 1, i2c_tx_cmd);
	//no configurable registers exist, so there is nothing more to configure here
}

// ***********************************************************************************************************************
void configureServoMagnetSensor() {
	char i2c_tx_cmd[4];
	char i2c_rx[10];

	// ***********************************************************************************************************************	// power up sensor
	i2c_tx_cmd[0] = 0;
	I2CWriteByteSequence(0b1011110, 1, i2c_tx_cmd);

	// ***********************************************************************************************************************	// read all registers once
	I2CReadByteSequence(0b1011110, 10, i2c_rx);

	// ***********************************************************************************************************************	// now configure sensor
	i2c_tx_cmd[0] = 0;
	i2c_tx_cmd[1] = (i2c_rx[7] & 0x18) | BIT(1); // copy bits 4+3 from ReadReg[7] + enable FAST conversion mode (BIT(1))
	i2c_tx_cmd[2] = i2c_rx[8];					// copy all bits from ReadReg[8]
	i2c_tx_cmd[3] = 0x80 | 0x40 | 0x20 | (i2c_rx[9] & 0x1F);// 0x80: temperature disabled
															// 0x40: low power fast (-> 12ms conversion time)
															// 0x20: parity check enabled
															// 0x1F: copy bits from ReadReg[9]

	long parity = i2c_tx_cmd[0] ^ i2c_tx_cmd[1] ^ i2c_tx_cmd[2] ^ i2c_tx_cmd[3];
	parity ^= (parity >> 4);
	parity ^= (parity >> 2);
	parity ^= (parity >> 1);
	if ((parity & 0x01) == 0) {
		i2c_tx_cmd[1] |= 0x80;
	}					// if needed, add parity bit to have odd parity

	I2CWriteByteSequence(0b1011110, 4, i2c_tx_cmd);
}

// ***********************************************************************************************************************
void sensorsInit() {

	// ***********************************************************************************************************************
	I2C1->CR1 |= I2C_CR1_PE;						// enable I2C
	I2C1->CR2 &= ~(BIT(11));						// set 7 bit addressing mode

	sleepMS(20);

	// ***********************************************************************************************************************
	if (pingSensor(0b0110101) == 0) {				// check if knee sensor exists and has seen itself with SDA=1 at power-up as 0b0110101
		configureKneeMagnetSensor(0b0110101);
		magnetSensorKneeAddress = 0b0110101;
	} else {
		magnetSensorKneeAddress = 0;				// no knee sensor available
	}

	// ***********************************************************************************************************************
	if (pingSensor(0b1001101) == 0) {				// check if foot sensor exists and has seen itself with SDA=1 at power-up as 0b1001101
		configureFootPressureSensor(0b1001101);
		footPressureSensorAddress = 0b1001101;
	} else {
		footPressureSensorAddress = 0;				// no foot sensor available
	}

	// ***********************************************************************************************************************
	PORTC->CLR = PORTC_MAGSENSOR_POWER;				// pin for on-board (servo) magnet compass off
	sleepMS(20);
	PORTC->SET = PORTC_MAGSENSOR_POWER;				// enable on-board magnetic compass sensor -> faster startup (< 0.001ms)
	sleepMS(20);
	configureServoMagnetSensor();
}

void readKnee() {
	uartSendChar('m');
	uartSendHexByte(magnetSensorKneeAddress);
	uartSendChar('\n');

	char i2c_rx[23];
	I2CReadByteSequence(0b0110101, 23, i2c_rx);
	uartSendStringDirect("gelesen\n");
	for (long c = 0; c < 23; c++) {
		uartSendHexByte(i2c_rx[c]);
		uartBusyWaitForTXDone();
		uartSendChar('-');
	}
	uartSendCharDirect('\n');
}

void sensorWriteTest() {

	long byteCount = 2;
	long oldISR = 0;
	long I2C_ISR = 0;

	uartSendString("I2C Sensor Test\n");
	uartBusyWaitForTXDone();

	I2C1->CR1 |= I2C_CR1_PE;						// enable I2C

	I2C1->CR2 &= ~(BIT(11));						// set 7 bit addressing mode

	I2C_SetAddress(0b1011110);	// read from this address (compass on servo PCB)
//	I2C_SetAddress(0b1011100);						// read from this address (compass on servo PCB)

//	I2C_SetRead();									// read data
	I2C_SetWrite();									// write data

	I2C_SetNumberOfBytesToTransmit(byteCount);// set how many bytes to be transmitted
	I2C1->CR2 |= BIT(25);// automatic end mode; (i.e. automatically send "end" after number of bytes are transmitted)

	if ((I2C1->ISR & I2C_ISR_TXE)) {			// is transmitter free?

		I2C1->TXDR = 00;					// this is the first byte to be sent
		byteCount--;			// so the first byte is already taken care of

		I2C1->CR2 |= I2C_CR2_START;				// generate I2C start condition

		while (I2C1->CR2 & I2C_CR2_START) {
			asm volatile ("nop");
		}		// wait for start bit to get cleared (independent of success)

//		while (((I2C1->ISR) & (I2C_ISR_TXE | I2C_ISR_NACKF))==0) {		asm volatile ("nop"); }			// wait until either successful of failure

		oldISR = I2C1->ISR;

		while (((I2C_ISR)
				& (I2C_ISR_TC | I2C_ISR_NACKF | I2C_ISR_STOPF | I2C_ISR_BERR))
				== 0) {

			I2C_ISR = (I2C1->ISR);

			if ((I2C_ISR) != oldISR) {
				uartSendString("ISR: ");
				uartSendHexLong(oldISR);
				uartSendString(" -> ");
				oldISR = I2C1->ISR;
				uartSendHexLong(oldISR);
				uartSendString("\n");
				uartBusyWaitForTXDone();
			}

			if (byteCount) {
				if ((I2C_ISR) & (I2C_ISR_TXE)) {
					I2C1->TXDR = byteCount;	// this is the next byte to be sent
					byteCount--;
				}
			}
		}

		if ((I2C_ISR) & (I2C_ISR_TC)) {
			uartSendStringDirect("success\n");
		}

		if ((I2C_ISR) & (I2C_ISR_STOPF)) {
			uartSendStringDirect("stop detected");
			I2C1->ICR |= I2C_ICR_STOPCF;
			uartSendStringDirect(" - flag cleared\n");
		}

		if ((I2C_ISR) & (I2C_ISR_BERR)) {
			uartSendStringDirect("bus error detected");
			I2C1->ICR |= I2C_ICR_BERRCF;
			uartSendStringDirect(" - flag cleared\n");
		}

		if ((I2C_ISR) & (I2C_ISR_NACKF)) {
			uartSendString("failure - no ack");
			I2C1->ICR |= I2C_ICR_NACKCF;
			uartSendStringDirect(" - flag cleared\n");
		}

	} else {
		uartSendString("failure - I2C busy\n");
		uartBusyWaitForTXDone();
	}

//	TXIS Flag is set after reception of successful ACK
//	NACKF Flag is set in case of no successful ACK

//	I2C1->TXDR = (new data to be transmitted);			// this also clears the TXIS flag

//	while ((I2C1->ISR) &

}

// ***********************************************************************************************************************
void sensorReadTest() {

	long I2C_ISR = 0;
	long byteCount = 10;
	long c = 0;
	long d[10];

//	uartSendString("I2C Sensor Read Test\n");
//	uartBusyWaitForTXDone();

	I2C1->CR1 |= I2C_CR1_PE;						// enable I2C

	I2C1->CR2 &= ~(BIT(11));						// set 7 bit addressing mode

	I2C_SetAddress(0b1011110);	// read from this address (compass on servo PCB)

	I2C_SetRead();									// read data
//	I2C_SetWrite();									// write data

	I2C_SetNumberOfBytesToTransmit(byteCount);// set how many bytes to be transmitted
	I2C1->CR2 |= BIT(25);// automatic end mode; (i.e. automatically send "end" after number of bytes are transmitted)

	I2C1->CR2 |= I2C_CR2_START;					// generate I2C start condition

	while (I2C1->CR2 & I2C_CR2_START) {
		asm volatile ("nop");
	}		// wait for start bit to get cleared (independent of success)

	//		while (((I2C1->ISR) & (I2C_ISR_TXE | I2C_ISR_NACKF))==0) {		asm volatile ("nop"); }			// wait until either successful of failure

	while (((I2C_ISR)
			& (I2C_ISR_TC | I2C_ISR_NACKF | I2C_ISR_STOPF | I2C_ISR_BERR)) == 0) {

		I2C_ISR = (I2C1->ISR);

		if (I2C_ISR & I2C_ISR_RXNE) {
//			uartSendCharDirect('*');
			d[c++] = I2C1->RXDR;					// read data
		}

	}

//	uartSendCharDirect('\n');
//	for (c=0; c<10; c++) {
//		uartSendHexByte(d[c]); uartBusyWaitForTXDone(); uartSendCharDirect('-');
//	}
//	uartSendCharDirect('\n');

	if ((I2C_ISR) & (I2C_ISR_TC)) {
		uartSendStringDirect("success\n");
	}

	if ((I2C_ISR) & (I2C_ISR_STOPF)) {
//		uartSendStringDirect("stop detected");
		I2C1->ICR |= I2C_ICR_STOPCF;
//		uartSendStringDirect(" - flag cleared\n");
	}

	if ((I2C_ISR) & (I2C_ISR_BERR)) {
		uartSendStringDirect("bus error detected");
		I2C1->ICR |= I2C_ICR_BERRCF;
		uartSendStringDirect(" - flag cleared\n");
	}

	if ((I2C_ISR) & (I2C_ISR_NACKF)) {
		uartSendString("failure - no ack");
		I2C1->ICR |= I2C_ICR_NACKCF;
		uartSendStringDirect(" - flag cleared\n");
	}

	long Bx = (((long) d[0]) << 4) | ((d[4] & 0xF0) >> 4);
	long By = (((long) d[1]) << 4) | ((d[4] & 0x0F));
	long Bz = (((long) d[2]) << 4) | ((d[5] & 0x0F));

	if (Bx & 0x0800)
		Bx -= 4096;
	if (By & 0x0800)
		By -= 4096;
	if (Bz & 0x0800)
		Bz -= 4096;

	long temp = (((long) (d[3] & 0xF0)) << 4) | (d[6]);

	uartSendHexShort(Bx);
	uartSendChar(' ');
	uartSendHexShort(By);
	uartSendChar(' ');
	uartSendHexShort(Bz);
	uartSendChar(' ');
	uartSendHexShort(temp);
	uartSendChar(' ');

	float Bxf = (float) Bx;
	float Byf = (float) By;

	float angleF = atan2f(Bxf, Byf);

	long angle = (long) (1000.0f * angleF);
	uartSendHexShort(angle);

	uartSendChar('\n');

}

// ***********************************************************************************************************************
void sensorMathTest(void) {

	long count = 0;
	long totalAngle = 0;

	uartSendCharDirect('\n');
	uartSendStringDirect("-tStart-\n");

	for (long Bx = -500; Bx < 500; Bx += 10) {
		for (long By = -500; By < 500; By += 10) {
			count++;

			float Bxf = (float) Bx;
			float Byf = (float) By;

			float angleF = atan2f(Bxf, Byf);

			long angle = (long) (1000.0f * angleF);

			totalAngle += angle;
		}
	}
	uartSendStringDirect("-tStop-\n");

	uartSendStringDirect("count: ");
	uartSendHexLong(count);
	uartBusyWaitForTXDone();
	uartSendCharDirect('\n');
	uartSendHexLong(totalAngle);
	uartBusyWaitForTXDone();
	uartSendCharDirect('\n');

}

// ***********************************************************************************************************************
void sensorsIterate() {

	// check if sensor is alive
	if ((systemTimeMS - lastSensorUpdateTimeMS) > 16) {				// max 16ms
		setMotorPositionControlOff();
		setLEDBlink(100, 500);

		if (systemTimeMS < 1000) {
			NVIC_SystemReset();						// reset for a clean restart
		}
//		uartSendString("ALARM: Magnet Sensor not responding!");			// TODO: ALARM
	}

	switch (sensorSM) {						// execute the sensor state machine

	// ***********************************************************************************************************************	// BLOCK 0: I2C read mag Sensor
	case 0:										// start I2C inquiry

		I2C_SetAddress(0b1011110);// read from this address (compass on servo PCB)
		I2C_SetRead();									// read data

		I2C_SetNumberOfBytesToTransmit(7);// set how many bytes to be transmitted
		I2C1->CR2 |= BIT(25);// automatic end mode; (i.e. automatically send "end" after number of bytes are transmitted)

		I2C1->CR2 |= I2C_CR2_START;				// generate I2C start condition

		SENSORSM_ADVANCE_STATE()
		;
		break;

	case 1:
		if ((I2C1->CR2 & I2C_CR2_START) == 0) {	// wait for start bit to get cleared
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 2:												// read byte[0]
		if ((I2C1->ISR) & I2C_ISR_RXNE) {
			sensorReadDataI2C[0] = I2C1->RXDR;					// read data
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 3:												// read byte[1]
		if ((I2C1->ISR) & I2C_ISR_RXNE) {
			sensorReadDataI2C[1] = I2C1->RXDR;					// read data
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 4:												// read byte[2]
		if ((I2C1->ISR) & I2C_ISR_RXNE) {
			sensorReadDataI2C[2] = I2C1->RXDR;					// read data
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 5:												// read byte[3]
		if ((I2C1->ISR) & I2C_ISR_RXNE) {
			sensorReadDataI2C[3] = I2C1->RXDR;					// read data
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 6:												// read byte[4]
		if ((I2C1->ISR) & I2C_ISR_RXNE) {
			sensorReadDataI2C[4] = I2C1->RXDR;					// read data
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 7:												// read byte[5]
		if ((I2C1->ISR) & I2C_ISR_RXNE) {
			sensorReadDataI2C[5] = I2C1->RXDR;					// read data
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 8:												// read byte[6]
		if ((I2C1->ISR) & I2C_ISR_RXNE) {
			sensorReadDataI2C[6] = I2C1->RXDR;					// read data
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 9:						// all data read, ack I2C and check for errors
//		uartSendCharDirect('\n');
//		for (c=0; c<7; c++) {
//			uartSendHexByte(sensorReadDataI2C[c]); uartBusyWaitForTXDone(); uartSendCharDirect('-');
//		}
//		uartSendCharDirect('\n');

		if ((I2C1->ISR) & (I2C_ISR_STOPF)) {// we expect to see a stop here, so no message
			I2C1->ICR |= I2C_ICR_STOPCF;
		} else {
//			uartSendStringDirect("I2C no stop detected\n");				// TODO: Don't show error, this happens once after power up. need to clear
			SENSORSM_NEXT_BLOCK();
			break;
		}

		if ((I2C1->ISR) & (I2C_ISR_BERR)) {
			uartSendStringDirect("bus error detected");
			I2C1->ICR |= I2C_ICR_BERRCF;
			uartSendStringDirect(" - flag cleared\n");
			SENSORSM_NEXT_BLOCK();
			break;
		}

		if ((I2C1->ISR) & (I2C_ISR_NACKF)) {
			uartSendString("failure - no ack");
			I2C1->ICR |= I2C_ICR_NACKCF;
			uartSendStringDirect(" - flag cleared\n");
			SENSORSM_NEXT_BLOCK();
			break;
		}

		SENSORSM_ADVANCE_STATE()
		;
		break;

	case 10:						// parse and convert received sensor data
//		if (sensorReadDataI2C[3] & 0x03) {				// check channel bits, if not equal to zero discard this measurement
//			uartSendChar('x');										// this is not an error, just debug
//			SENSORSM_NEXT_BLOCK();
//			break;
//		}

		magIndexNew = (sensorReadDataI2C[3] & 0x0C) >> 2;
		if (magIndexNew != magIndexOld) {
			magIndexOld = magIndexNew;
//			uartSendChar(':');										// this is not an error, just debug
		} else {
//			uartSendChar('.');										// this is not an error, just debug
			SENSORSM_NEXT_BLOCK();						// no new data available
			break;
		}

		sensorBx = (((long) sensorReadDataI2C[0]) << 4)
				| ((sensorReadDataI2C[4] & 0xF0) >> 4);	// extract magnet sensor "longs"
		sensorBy = (((long) sensorReadDataI2C[1]) << 4)
				| ((sensorReadDataI2C[4] & 0x0F));
		sensorBz = (((long) sensorReadDataI2C[2]) << 4)
				| ((sensorReadDataI2C[5] & 0x0F));
		sensorTemp = (((long) (sensorReadDataI2C[3] & 0xF0)) << 4)
				| (sensorReadDataI2C[6]);		// extract temperature as "long"

		if (sensorBx & 0x0800)
			sensorBx -= 4096;					// make magnetic values signed
		if (sensorBy & 0x0800)
			sensorBy -= 4096;
		if (sensorBz & 0x0800)
			sensorBz -= 4096;

//		uartSendHexShort(sensorBx); uartSendChar(' ');
//		uartSendHexShort(sensorBy); uartSendChar(' ');
//		uartSendHexShort(sensorBz); uartSendChar(' ');
//		uartSendHexShort(sensorTemp); uartSendChar('\n ');

		sensorBxf = (float) sensorBx;// convert the required ones to float in this iteration
		sensorByf = (float) sensorBy;

		SENSORSM_ADVANCE_STATE()
		;
		break;

	case 11:
		sensorAngleF = atan2f(sensorBxf, sensorByf);// compute only atan2 in this iteration (to be really fast back for control)
		SENSORSM_ADVANCE_STATE()
		;
		break;

	case 12:									// convert float angle in long [
		currentServoPosition = (long) ((2048.0f / M_PI) * sensorAngleF) + 2048;

		lastSensorUpdateTimeMS = systemTimeMS;// remember THIS is the time when we got a new sensor update

		SENSORSM_NEXT_BLOCK()
		;
		break;

		// ***********************************************************************************************************************	// BLOCK 1: I2C read external mag Sensor
	case 256:						// here the next "block" of actions begins

		if (magnetSensorKneeAddress == 0) {
			SENSORSM_NEXT_BLOCK();					// goto next block
			break;
		}

		I2C_SetAddress(magnetSensorKneeAddress);// read from this address (compass on knee PCB)
		I2C_SetRead();									// read data

		I2C_SetNumberOfBytesToTransmit(7);// set how many bytes to be transmitted
		I2C1->CR2 |= BIT(25);// automatic end mode; (i.e. automatically send "end" after number of bytes are transmitted)

		I2C1->CR2 |= I2C_CR2_START;				// generate I2C start condition

		SENSORSM_ADVANCE_STATE()
		;
		break;

	case 257:
		if ((I2C1->CR2 & I2C_CR2_START) == 0) {	// wait for start bit to get cleared
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 258:												// read byte[0]
		if ((I2C1->ISR) & I2C_ISR_RXNE) {
			sensorReadDataI2C[0] = I2C1->RXDR;					// read data
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 259:												// read byte[1]
		if ((I2C1->ISR) & I2C_ISR_RXNE) {
			sensorReadDataI2C[1] = I2C1->RXDR;					// read data
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 260:												// read byte[2]
		if ((I2C1->ISR) & I2C_ISR_RXNE) {
			sensorReadDataI2C[2] = I2C1->RXDR;					// read data
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 261:												// read byte[3]
		if ((I2C1->ISR) & I2C_ISR_RXNE) {
			sensorReadDataI2C[3] = I2C1->RXDR;					// read data
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 262:												// read byte[4]
		if ((I2C1->ISR) & I2C_ISR_RXNE) {
			sensorReadDataI2C[4] = I2C1->RXDR;					// read data
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 263:												// read byte[5]
		if ((I2C1->ISR) & I2C_ISR_RXNE) {
			sensorReadDataI2C[5] = I2C1->RXDR;					// read data
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 264:												// read byte[6]
		if ((I2C1->ISR) & I2C_ISR_RXNE) {
			sensorReadDataI2C[6] = I2C1->RXDR;					// read data
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 265:					// all data read, ack I2C and check for errors
//		uartSendCharDirect('\n');
//		for (c=0; c<7; c++) {
//			uartSendHexByte(sensorReadDataI2C[c]); uartBusyWaitForTXDone(); uartSendCharDirect('-');
//		}
//		uartSendCharDirect('\n');

		if ((I2C1->ISR) & (I2C_ISR_STOPF)) {// we expect to see a stop here, so no message
			I2C1->ICR |= I2C_ICR_STOPCF;
		} else {
//			uartSendStringDirect("I2C no stop detected\n");				// TODO: Don't show error, this happens once after power up. need to clear
			SENSORSM_NEXT_BLOCK();
			break;
		}

		if ((I2C1->ISR) & (I2C_ISR_BERR)) {
			uartSendStringDirect("bus error detected");
			I2C1->ICR |= I2C_ICR_BERRCF;
			uartSendStringDirect(" - flag cleared\n");
			SENSORSM_NEXT_BLOCK();
			break;
		}

		if ((I2C1->ISR) & (I2C_ISR_NACKF)) {
			uartSendString("failure - no ack");
			I2C1->ICR |= I2C_ICR_NACKCF;
			uartSendStringDirect(" - flag cleared\n");
			SENSORSM_NEXT_BLOCK();
			break;
		}

		SENSORSM_ADVANCE_STATE()
		;
		break;

	case 266:						// parse and convert received sensor data
//		if (sensorReadDataI2C[3] & 0x03) {				// check channel bits, if not equal to zero discard this measurement
//			uartSendChar('x');										// this is not an error, just debug
//			SENSORSM_NEXT_BLOCK();
//			break;
//		}

		kneeMagIndexNew = sensorReadDataI2C[6] & 0x03;
		if (kneeMagIndexNew != kneeMagIndexOld) {
			kneeMagIndexOld = kneeMagIndexNew;
		} else {
			SENSORSM_NEXT_BLOCK();
			break;
		}
		// extract magnet sensor "longs"
		kneeBx = (((long) sensorReadDataI2C[0]) << 4)
				| ((sensorReadDataI2C[4] & 0xF0) >> 4);
		kneeBy = (((long) sensorReadDataI2C[1]) << 4)
				| ((sensorReadDataI2C[4] & 0x0F));

		SENSORSM_NEXT_BLOCK()
		;
		break;

		// ***********************************************************************************************************************	// BLOCK 2: I2C read foot pressure sensor
	case 512:
		if (footPressureSensorAddress == 0) {
			SENSORSM_NEXT_BLOCK();					// goto next block
			break;
		}

		I2C_SetAddress(footPressureSensorAddress);// read from this address (foot pressure on sensor PCB)
		I2C_SetRead();									// read data

		I2C_SetNumberOfBytesToTransmit(2);// set how many bytes to be transmitted
		I2C1->CR2 |= BIT(25);// automatic end mode; (i.e. automatically send "end" after number of bytes are transmitted)

		I2C1->CR2 |= I2C_CR2_START;				// generate I2C start condition

		SENSORSM_ADVANCE_STATE()
		;
		break;

	case 513:
		if ((I2C1->CR2 & I2C_CR2_START) == 0) {	// wait for start bit to get cleared
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 514:												// read byte[0]
		if ((I2C1->ISR) & I2C_ISR_RXNE) {
			sensorReadDataI2C[0] = I2C1->RXDR;					// read data
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 515:												// read byte[1]
		if ((I2C1->ISR) & I2C_ISR_RXNE) {
			sensorReadDataI2C[1] = I2C1->RXDR;					// read data
			SENSORSM_ADVANCE_STATE();
		}
		break;

	case 516:					// all data read, ack I2C and check for errors
		if ((I2C1->ISR) & (I2C_ISR_STOPF)) {// we expect to see a stop here, so no message
			I2C1->ICR |= I2C_ICR_STOPCF;
		} else {
			//			uartSendStringDirect("I2C no stop detected\n");				// TODO: Don't show error, this happens once after power up. need to clear
			SENSORSM_NEXT_BLOCK();
			break;
		}

		if ((I2C1->ISR) & (I2C_ISR_BERR)) {
			uartSendStringDirect("bus error detected");
			I2C1->ICR |= I2C_ICR_BERRCF;
			uartSendStringDirect(" - flag cleared\n");
			SENSORSM_NEXT_BLOCK();
			break;
		}

		if ((I2C1->ISR) & (I2C_ISR_NACKF)) {
			uartSendString("failure - no ack");
			I2C1->ICR |= I2C_ICR_NACKCF;
			uartSendStringDirect(" - flag cleared\n");
			SENSORSM_NEXT_BLOCK();
			break;
		}

		SENSORSM_ADVANCE_STATE()
		;
		break;

	case 517:						// parse and convert received sensor data

		footPressureReading = (sensorReadDataI2C[0] << 8)
				| (sensorReadDataI2C[1]);

		SENSORSM_NEXT_BLOCK()
		;
		break;

		// ***********************************************************************************************************************	// BLOCK 3: currently nothing, just restart
	case 768:
		sensorSM = 0;									// just restart SM
		break;

	default:// this should not happen; signal error and restart state machine!
		uartSendStringDirect("Servo SM error: ");
		uartSendHexLong(sensorSM);
		uartBusyWaitForTXDone();
		uartSendCharDirect('\n');
		sensorSM = 0;
		break;
	}
}
