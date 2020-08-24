/*
 * motorControl.c
 *
 */

#include <mouseServo.h>
#include <stdlib.h>

long motorSpeed=0;

long motorPIDControlEnabled = 0;					// disabled (for default value, see mouseServo.h, top definitions)

long desiredServoPosition = 2048;

#define DERIVATIVE_TIME_DELTA_MS		(8-1)		// look back this many milliseconds	(use -1 as size for memory table)
														// note that changing this also has an impact on gainI!
long positionErrorMemory[DERIVATIVE_TIME_DELTA_MS];
long positionErrorMemoryIndex;

//long gainP = 0x1A;									// hand tuned initial PID parameters
//long gainI = 0x03;
//long gainD = 0x28;

long gainP = 0x08;									// hand tuned initial PID parameters Peer on Mouse Robot V4
long gainI = 0x01;
long gainD = 0x15;

long positionError, positionErrorD, positionErrorI;

// ***********************************************************************************************************************
void motorControlInit() {

	for (int i=0; i<DERIVATIVE_TIME_DELTA_MS; i++) {
		positionErrorMemory[i] = desiredServoPosition;				// fill with "best guess"
	}
	positionErrorMemoryIndex = 0;

	positionError = 0;
	positionErrorD = 0;
	positionErrorI = 0;												// this is crucial to start at I=0;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);						// start PWM output
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);


#ifdef SERVO_CENTER_ON_POWERUP
	motorPIDControlEnabled = 1;					// enabled by default
#endif

}


// ***********************************************************************************************************************
void motorControlIterate1KHz() {


	if (motorPIDControlEnabled) {

		positionError = (desiredServoPosition - currentServoPosition);

		positionErrorD = (positionError - positionErrorMemory[positionErrorMemoryIndex]);
		positionErrorMemory[positionErrorMemoryIndex++] = positionError;
		if (positionErrorMemoryIndex == DERIVATIVE_TIME_DELTA_MS) positionErrorMemoryIndex=0;

		if (positionError>=0) {
			positionErrorI += (positionError>>1);								// >>1 as to ignore "tiny" position errors
		} else {
			positionErrorI -= ((-positionError)>>1);							// >>1 as to ignore "tiny" position errors
		}

		if ((abs(positionError) < 0x4) && (abs(positionErrorI) < 0x400)) {
//		if (((positionErrorI & 0xFFFFF000)==0) || ((positionErrorI & 0xFFFF000)==0xFFFFF000)) {

			setMotorSpeed(0);

		} else {

			motorSpeed = gainP * positionError  +  gainD * positionErrorD  +  ((gainI * positionErrorI)>>8);
			setMotorSpeed(motorSpeed);
		}
	}

}


// ***********************************************************************************************************************
void setDesiredMotorPosition(long dSP) {
	desiredServoPosition = dSP;
	positionErrorI = 0;												// this is crucial to start at I=0;
	motorPIDControlEnabled=1;
}

void setMotorPositionControlOff(void) {
	setMotorSpeed(0);
	motorPIDControlEnabled=0;
}

// ***********************************************************************************************************************
void setMotorSpeed(long m) {

	motorSpeed = m;

	if (m ==0) {
		TIM2->CCR1 = 0;
		TIM2->CCR2 = 0;

	} else {

		if (m<0) {
//			if (m < (-4095)) m = -4095;
			TIM2->CCR1 = (-m)<<1;
			TIM2->CCR2 = 0;

		} else {

//			if (m > 4095) m = 4095;
			TIM2->CCR1 = 0;
			TIM2->CCR2 = m<<1;
		}
	}
}

void setMotorSpeedDelta(long m) {
	setMotorSpeed(motorSpeed+m);
}

long getMotorSpeed() {
	return(motorSpeed);
}
