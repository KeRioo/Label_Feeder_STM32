/*
 * Stepper.cpp
 *
 *  Created on: Aug 29, 2022
 *      Author: mprzybyl
 */

// Stepper.cpp
//
// Copyright (C) 2009-2013 Mike McCauley
// $Id: Stepper.cpp,v 1.24 2020/04/20 00:15:03 mikem Exp mikem $
#include <Stepper.h>

#define max(x, y) (((x) > (y)) ? (x) : (y))
#define min(x, y) (((x) < (y)) ? (x) : (y))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

void Stepper_moveTo(STEPPER_TypeDef *hstep, long absolute) {
	if (hstep->_targetPos != absolute) {
		hstep->_targetPos = absolute;
		Stepper_computeNewSpeed(hstep);
		// compute new n?
	}
}

void Stepper_move(STEPPER_TypeDef *hstep, long relative) {
	Stepper_moveTo(hstep, hstep->_currentPos + relative);
}

// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
bool Stepper_runSpeed(STEPPER_TypeDef *hstep) {
	// Dont do anything unless we actually have a step interval
	if (!hstep->_stepInterval)
		return false;

	unsigned long time = micros();
	if (time - hstep->_lastStepTime >= hstep->_stepInterval) {
		if (hstep->_direction == DIRECTION_CW) {
			// Clockwise
			hstep->_currentPos += 1;
		} else {
			// Anticlockwise
			hstep->_currentPos -= 1;
		}
		Stepper_step(hstep, hstep->_currentPos);

		hstep->_lastStepTime = time; // Caution: does not account for costs in step()

		return true;
	} else {
		return false;
	}
}

long Stepper_distanceToGo(STEPPER_TypeDef *hstep) {
	return hstep->_targetPos - hstep->_currentPos;
}

long Stepper_targetPosition(STEPPER_TypeDef *hstep) {
	return hstep->_targetPos;
}

long Stepper_currentPosition(STEPPER_TypeDef *hstep) {
	return hstep->_currentPos;
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void Stepper_setCurrentPosition(STEPPER_TypeDef *hstep, long position) {
	hstep->_targetPos = hstep->_currentPos = position;
	hstep->_n = 0;
	hstep->_stepInterval = 0;
	hstep->_speed = 0.0;
}

void Stepper_computeNewSpeed(STEPPER_TypeDef *hstep) {
	long distanceTo = Stepper_distanceToGo(hstep); // +ve is clockwise from curent location

	long stepsToStop = (long) ((hstep->_speed * hstep->_speed)
			/ (2.0 * hstep->_acceleration)); // Equation 16

	if (distanceTo == 0 && stepsToStop <= 1) {
		// We are at the target and its time to stop
		hstep->_stepInterval = 0;
		hstep->_speed = 0.0;
		hstep->_n = 0;
		return;
	}

	if (distanceTo > 0) {
		// We are anticlockwise from the target
		// Need to go clockwise from here, maybe decelerate now
		if (hstep->_n > 0) {
			// Currently accelerating, need to decel now? Or maybe going the wrong way?
			if ((stepsToStop >= distanceTo)
					|| hstep->_direction == DIRECTION_CCW)
				hstep->_n = -stepsToStop; // Start deceleration
		} else if (hstep->_n < 0) {
			// Currently decelerating, need to accel again?
			if ((stepsToStop < distanceTo) && hstep->_direction == DIRECTION_CW)
				hstep->_n = -hstep->_n; // Start accceleration
		}
	} else if (distanceTo < 0) {
		// We are clockwise from the target
		// Need to go anticlockwise from here, maybe decelerate
		if (hstep->_n > 0) {
			// Currently accelerating, need to decel now? Or maybe going the wrong way?
			if ((stepsToStop >= -distanceTo)
					|| hstep->_direction == DIRECTION_CW)
				hstep->_n = -stepsToStop; // Start deceleration
		} else if (hstep->_n < 0) {
			// Currently decelerating, need to accel again?
			if ((stepsToStop < -distanceTo)
					&& hstep->_direction == DIRECTION_CCW)
				hstep->_n = -hstep->_n; // Start accceleration
		}
	}

	// Need to accelerate or decelerate
	if (hstep->_n == 0) {
		// First step from stopped
		hstep->_cn = hstep->_c0;
		hstep->_direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
	} else {
		// Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
		hstep->_cn = hstep->_cn
				- ((2.0 * hstep->_cn) / ((4.0 * hstep->_n) + 1)); // Equation 13
		hstep->_cn = max(hstep->_cn, hstep->_cmin);
	}
	hstep->_n++;
	hstep->_stepInterval = hstep->_cn;
	hstep->_speed = 1000000.0 / hstep->_cn;
	if (hstep->_direction == DIRECTION_CCW)
		hstep->_speed = -hstep->_speed;

}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if the motor is still running to the target position.
bool Stepper_run(STEPPER_TypeDef *hstep) {
	if (Stepper_runSpeed(hstep))
		Stepper_computeNewSpeed(hstep);
	return hstep->_speed != 0.0 || Stepper_distanceToGo(hstep) != 0;
}

void Stepper_Init(STEPPER_TypeDef *hstep, unsigned int pin1, unsigned int pin2,
		bool enable) {
	hstep->_currentPos = 0;
	hstep->_targetPos = 0;
	hstep->_speed = 0.0;
	hstep->_maxSpeed = 1.0;
	hstep->_acceleration = 0.0;
	hstep->_sqrt_twoa = 1.0;
	hstep->_stepInterval = 0;
	hstep->_minPulseWidth = 1;
	hstep->_enablePin = 0xff;
	hstep->_lastStepTime = 0;
	hstep->_pin[0] = pin1;
	hstep->_pin[1] = pin2;
	hstep->_enableInverted = false;

	// NEW
	hstep->_n = 0;
	hstep->_c0 = 0.0;
	hstep->_cn = 0.0;
	hstep->_cmin = 1.0;
	hstep->_direction = DIRECTION_CCW;

	hstep->GPIO_Port = GPIOB;

	int i;
	for (i = 0; i < 2; i++)
		hstep->_pinInverted[i] = 0;
	if (enable)
		Stepper_enableOutputs(hstep);
	// Some reasonable default
	Stepper_setAcceleration(hstep, 1);
}

void Stepper_setMaxSpeed(STEPPER_TypeDef *hstep, float speed) {
	if (speed < 0.0)
		speed = -speed;
	if (hstep->_maxSpeed != speed) {
		hstep->_maxSpeed = speed;
		hstep->_cmin = 1000000.0 / speed;
		// Recompute _n from current speed and adjust speed if accelerating or cruising
		if (hstep->_n > 0) {
			hstep->_n = (long) ((hstep->_speed * hstep->_speed)
					/ (2.0 * hstep->_acceleration)); // Equation 16
			Stepper_computeNewSpeed(hstep);
		}
	}
}

float Stepper_maxSpeed(STEPPER_TypeDef *hstep) {
	return hstep->_maxSpeed;
}

void Stepper_setAcceleration(STEPPER_TypeDef *hstep, float acceleration) {
	if (acceleration == 0.0)
		return;
	if (acceleration < 0.0)
		acceleration = -acceleration;
	if (hstep->_acceleration != acceleration) {
		// Recompute _n per Equation 17
		hstep->_n = hstep->_n * (hstep->_acceleration / acceleration);
		// New c0 per Equation 7, with correction per Equation 15
		hstep->_c0 = 0.676 * sqrt(2.0 / acceleration) * 1000000.0; // Equation 15
		hstep->_acceleration = acceleration;
		Stepper_computeNewSpeed(hstep);
	}
}

void Stepper_setSpeed(STEPPER_TypeDef *hstep, float speed) {
	if (speed == hstep->_speed)
		return;
	speed = constrain(speed, -hstep->_maxSpeed, hstep->_maxSpeed);
	if (speed == 0.0)
		hstep->_stepInterval = 0;
	else {
		hstep->_stepInterval = fabs(1000000.0 / speed);
		hstep->_direction = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
	}
	hstep->_speed = speed;
}

float Stepper_speed(STEPPER_TypeDef *hstep) {
	return hstep->_speed;
}

// Subclasses can override
void Stepper_step(STEPPER_TypeDef *hstep, long step) {
	Stepper_step1(hstep, step);

}

// You might want to override this to implement eg serial output
// bit 0 of the mask corresponds to _pin[0]
// bit 1 of the mask corresponds to _pin[1]
// ....
void Stepper_setOutputPins(STEPPER_TypeDef *hstep, unsigned int mask) {
	unsigned int numpins = 2;
	unsigned int i;
	for (i = 0; i < numpins; i++)
		HAL_GPIO_WritePin(hstep->GPIO_Port, hstep->_pin[i],
				hstep->_pinInverted[i] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// 1 pin step function (ie for stepper drivers)
// This is passed the current step number (0 to 7)
// Subclasses can override
void Stepper_step1(STEPPER_TypeDef *hstep, long step) {
	(void) (step); // Unused

	// _pin[0] is step, _pin[1] is direction
	Stepper_setOutputPins(hstep, hstep->_direction ? 0b10 : 0b00); // Set direction first else get rogue pulses
	Stepper_setOutputPins(hstep, hstep->_direction ? 0b11 : 0b01); // step HIGH
	// Caution 200ns setup time
	// Delay the minimum allowed pulse width
	delay_us(hstep->_minPulseWidth);
	Stepper_setOutputPins(hstep, hstep->_direction ? 0b10 : 0b00); // step LOW
}

// Prevents power consumption on the outputs
void Stepper_disableOutputs(STEPPER_TypeDef *hstep) {

	Stepper_setOutputPins(hstep, 0); // Handles inversion automatically
	if (hstep->_enablePin != 0xff) {
		HAL_GPIO_WritePin(hstep->GPIO_Port, hstep->_enablePin,
				hstep->_enableInverted ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
}

void Stepper_enableOutputs(STEPPER_TypeDef *hstep) {

	if (hstep->_enablePin != 0xff) {
		HAL_GPIO_WritePin(hstep->GPIO_Port, hstep->_enablePin,
				hstep->_enableInverted ? GPIO_PIN_RESET : GPIO_PIN_SET);
	}
}

void Stepper_setMinPulseWidth(STEPPER_TypeDef *hstep, unsigned int minWidth) {
	hstep->_minPulseWidth = minWidth;
}

void Stepper_setEnablePin(STEPPER_TypeDef *hstep, unsigned int enablePin) {
	hstep->_enablePin = enablePin;

	// This happens after construction, so init pin now.
	if (hstep->_enablePin != 0xff) {
		HAL_GPIO_WritePin(hstep->GPIO_Port, hstep->_enablePin,
				hstep->_enableInverted ? GPIO_PIN_RESET : GPIO_PIN_SET);
	}
}

void Stepper_setPinsInverted(STEPPER_TypeDef *hstep, bool directionInvert,
		bool stepInvert,
		bool enableInvert) {
	hstep->_pinInverted[0] = stepInvert;
	hstep->_pinInverted[1] = directionInvert;
	hstep->_enableInverted = enableInvert;
}

// Blocks until the target position is reached and stopped
void Stepper_runToPosition(STEPPER_TypeDef *hstep) {
	while (Stepper_run(hstep))
		YIELD; // Let system housekeeping occur
}

bool Stepper_runSpeedToPosition(STEPPER_TypeDef *hstep) {
	if (hstep->_targetPos == hstep->_currentPos)
		return false;
	if (hstep->_targetPos > hstep->_currentPos)
		hstep->_direction = DIRECTION_CW;
	else
		hstep->_direction = DIRECTION_CCW;
	return Stepper_runSpeed(hstep);
}

// Blocks until the new target position is reached
void Stepper_runToNewPosition(STEPPER_TypeDef *hstep, long position) {
	Stepper_moveTo(hstep, position);
	Stepper_runToPosition(hstep);
}

void Stepper_stop(STEPPER_TypeDef *hstep) {
	if (hstep->_speed != 0.0) {
		long stepsToStop = (long) ((hstep->_speed * hstep->_speed)
				/ (2.0 * hstep->_acceleration)) + 1; // Equation 16 (+integer rounding)
		if (hstep->_speed > 0)
			Stepper_move(hstep, stepsToStop);
		else
			Stepper_move(hstep, -stepsToStop);
	}
}

bool Stepper_isRunning(STEPPER_TypeDef *hstep) {
	return !(hstep->_speed == 0.0 && hstep->_targetPos == hstep->_currentPos);
}

long mm2steps(float mm) {
	return round(mm * STEPS_PER_MILIMETER);

}

bool Stepper_goToEndstop(STEPPER_TypeDef *hstep, GPIO_TypeDef *port, uint16_t pin, uint16_t max_distance) {
	Stepper_move(hstep, max_distance);
	while(HAL_GPIO_ReadPin(port, pin) != GPIO_PIN_SET && Stepper_run(hstep) == true) {
		if (   HAL_GPIO_ReadPin(Endstop_L_GPIO_Port, Endstop_L_Pin) == GPIO_PIN_SET
			|| HAL_GPIO_ReadPin(Endstop_R_GPIO_Port, Endstop_R_Pin) == GPIO_PIN_SET) {
			Stepper_stop(hstep);
			Stepper_runToPosition(hstep);
			return false;
		}

	};
	if(HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET){
		Stepper_stop(hstep);
		Stepper_runToPosition(hstep);
		return true;
	}

	return false;

}

