/*
 * Stepper.h
 *
 *  Created on: Aug 29, 2022
 *      Author: mprzybyl
 */
#pragma once
#ifndef Stepper_h
#define lStepper_h

#include "main.h"
#include <stdint.h>
#include <math.h>
#include "micros.h"

// Use the system yield() whenever possoible, since some platforms require it for housekeeping, especially
// ESP8266
#if (defined(ARDUINO) && ARDUINO >= 155) || defined(ESP8266)
 #define YIELD yield();
#else
#define YIELD
#endif

#define STEPS_PER_MILIMETER 160

typedef enum {
	DRIVER = 1, ///< Stepper Driver, 2 driver pins required
} MotorInterfaceType;

typedef struct {

	/// Current direction motor is spinning in
	/// Protected because some peoples subclasses need it to be so
	bool _direction; // 1 == CW

	GPIO_TypeDef *GPIO_Port;

	/// Arduino pin number assignments for the 2 or 4 pins required to interface to the
	/// stepper motor or driver
	unsigned int _pin[2];

	/// Whether the _pins is inverted or not
	unsigned int _pinInverted[2];

	/// The current absolution position in steps.
	long _currentPos;    // Steps

	/// The target position in steps. The Stepper library will move the
	/// motor from the _currentPos to the _targetPos, taking into account the
	/// max speed, acceleration and deceleration
	long _targetPos;     // Steps

	/// The current motos speed in steps per second
	/// Positive is clockwise
	float _speed;         // Steps per second

	/// The maximum permitted speed in steps per second. Must be > 0.
	float _maxSpeed;

	/// The acceleration to use to accelerate or decelerate the motor in steps
	/// per second per second. Must be > 0
	float _acceleration;
	float _sqrt_twoa; // Precomputed sqrt(2*_acceleration)

	/// The current interval between steps in microseconds.
	/// 0 means the motor is currently stopped with _speed == 0
	unsigned long _stepInterval;

	/// The last step time in microseconds
	unsigned long _lastStepTime;

	/// The minimum allowed pulse width in microseconds
	unsigned int _minPulseWidth;

	/// Is the enable pin inverted?
	bool _enableInverted;

	/// Enable pin for stepper driver, or 0xFF if unused.
	unsigned int _enablePin;

	/// The pointer to a forward-step procedure
	void (*_forward)();

	/// The pointer to a backward-step procedure
	void (*_backward)();

	/// The step counter for speed calculations
	long _n;

	/// Initial step size in microseconds
	float _c0;

	/// Last step size in microseconds
	float _cn;

	/// Min step size in microseconds based on maxSpeed
	float _cmin; // at max speed
} STEPPER_TypeDef;

///
void Stepper_Init(STEPPER_TypeDef *hstep, unsigned int pin1, unsigned int pin2,
		bool enable);

/// Set the target position. The run() function will try to move the motor (at most one step per call)
/// from the current position to the target position set by the most
/// recent call to this function. Caution: moveTo() also recalculates the speed for the next step.
/// If you are trying to use constant speed movements, you should call setSpeed() after calling moveTo().
/// \param[in] absolute The desired absolute position. Negative is
/// anticlockwise from the 0 position.
void Stepper_moveTo(STEPPER_TypeDef *hstep, long absolute);

/// Set the target position relative to the current position.
/// \param[in] relative The desired position relative to the current position. Negative is
/// anticlockwise from the current position.
void Stepper_move(STEPPER_TypeDef *hstep, long relative);

/// Poll the motor and step it if a step is due, implementing
/// accelerations and decelerations to achieve the target position. You must call this as
/// frequently as possible, but at least once per minimum step time interval,
/// preferably in your main loop. Note that each call to run() will make at most one step, and then only when a step is due,
/// based on the current speed and the time since the last step.
/// \return true if the motor is still running to the target position.
bool Stepper_run(STEPPER_TypeDef *hstep);

/// Poll the motor and step it if a step is due, implementing a constant
/// speed as set by the most recent call to setSpeed(). You must call this as
/// frequently as possible, but at least once per step interval,
/// \return true if the motor was stepped.
bool Stepper_runSpeed(STEPPER_TypeDef *hstep);

/// Sets the maximum permitted speed. The run() function will accelerate
/// up to the speed set by this function.
/// Caution: the maximum speed achievable depends on your processor and clock speed.
/// The default maxSpeed is 1.0 steps per second.
/// \param[in] speed The desired maximum speed in steps per second. Must
/// be > 0. Caution: Speeds that exceed the maximum speed supported by the processor may
/// Result in non-linear accelerations and decelerations.
void Stepper_setMaxSpeed(STEPPER_TypeDef *hstep, float speed);

/// Returns the maximum speed configured for this stepper
/// that was previously set by setMaxSpeed();
/// \return The currently configured maximum speed
float Stepper_maxSpeed(STEPPER_TypeDef *hstep);

/// Sets the acceleration/deceleration rate.
/// \param[in] acceleration The desired acceleration in steps per second
/// per second. Must be > 0.0. This is an expensive call since it requires a square
/// root to be calculated. Dont call more ofthen than needed
void Stepper_setAcceleration(STEPPER_TypeDef *hstep, float acceleration);

/// Sets the desired constant speed for use with runSpeed().
/// \param[in] speed The desired constant speed in steps per
/// second. Positive is clockwise. Speeds of more than 1000 steps per
/// second are unreliable. Very slow speeds may be set (eg 0.00027777 for
/// once per hour, approximately. Speed accuracy depends on the Arduino
/// crystal. Jitter depends on how frequently you call the runSpeed() function.
/// The speed will be limited by the current value of setMaxSpeed()
void Stepper_setSpeed(STEPPER_TypeDef *hstep, float speed);

/// The most recently set speed.
/// \return the most recent speed in steps per second
float Stepper_speed(STEPPER_TypeDef *hstep);

/// The distance from the current position to the target position.
/// \return the distance from the current position to the target position
/// in steps. Positive is clockwise from the current position.
long Stepper_distanceToGo(STEPPER_TypeDef *hstep);

/// The most recently set target position.
/// \return the target position
/// in steps. Positive is clockwise from the 0 position.
long Stepper_targetPosition(STEPPER_TypeDef *hstep);

/// The current motor position.
/// \return the current motor position
/// in steps. Positive is clockwise from the 0 position.
long Stepper_currentPosition(STEPPER_TypeDef *hstep);

/// Resets the current position of the motor, so that wherever the motor
/// happens to be right now is considered to be the new 0 position. Useful
/// for setting a zero position on a stepper after an initial hardware
/// positioning move.
/// Has the side effect of setting the current motor speed to 0.
/// \param[in] position The position in steps of wherever the motor
/// happens to be right now.
void Stepper_setCurrentPosition(STEPPER_TypeDef *hstep, long position);

/// Moves the motor (with acceleration/deceleration)
/// to the target position and blocks until it is at
/// position. Dont use this in event loops, since it blocks.
void Stepper_runToPosition(STEPPER_TypeDef *hstep);

/// Runs at the currently selected speed until the target position is reached.
/// Does not implement accelerations.
/// \return true if it stepped
bool Stepper_runSpeedToPosition(STEPPER_TypeDef *hstep);

/// Moves the motor (with acceleration/deceleration)
/// to the new target position and blocks until it is at
/// position. Dont use this in event loops, since it blocks.
/// \param[in] position The new target position.
void Stepper_runToNewPosition(STEPPER_TypeDef *hstep, long position);

/// Sets a new target position that causes the stepper
/// to stop as quickly as possible, using the current speed and acceleration parameters.
void Stepper_stop(STEPPER_TypeDef *hstep);

/// Disable motor pin outputs by setting them all LOW
/// Depending on the design of your electronics this may turn off
/// the power to the motor coils, saving power.
/// This is useful to support Arduino low power modes: disable the outputs
/// during sleep and then reenable with enableOutputs() before stepping
/// again.
/// If the enable Pin is defined, sets it to OUTPUT mode and clears the pin to disabled.
void Stepper_disableOutputs(STEPPER_TypeDef *hstep);

/// Enable motor pin outputs by setting the motor pins to OUTPUT
/// mode. Called automatically by the constructor.
/// If the enable Pin is defined, sets it to OUTPUT mode and sets the pin to enabled.
void Stepper_enableOutputs(STEPPER_TypeDef *hstep);

/// Sets the minimum pulse width allowed by the stepper driver. The minimum practical pulse width is
/// approximately 20 microseconds. Times less than 20 microseconds
/// will usually result in 20 microseconds or so.
/// \param[in] minWidth The minimum pulse width in microseconds.
void Stepper_setMinPulseWidth(STEPPER_TypeDef *hstep, unsigned int minWidth);

/// Sets the enable pin number for stepper drivers.
/// 0xFF indicates unused (default).
/// Otherwise, if a pin is set, the pin will be turned on when
/// enableOutputs() is called and switched off when disableOutputs()
/// is called.
/// \param[in] enablePin Arduino digital pin number for motor enable
/// \sa setPinsInverted
void Stepper_setEnablePin(STEPPER_TypeDef *hstep, unsigned int enablePin);

/// Sets the inversion for stepper driver pins
/// \param[in] directionInvert True for inverted direction pin, false for non-inverted
/// \param[in] stepInvert      True for inverted step pin, false for non-inverted
/// \param[in] enableInvert    True for inverted enable pin, false (default) for non-inverted
void Stepper_setPinsInverted(STEPPER_TypeDef *hstep, bool directionInvert,
		bool stepInvert,
		bool enableInvert);

/// Checks to see if the motor is currently running to a target
/// \return true if the speed is not zero or not at the target position
bool Stepper_isRunning(STEPPER_TypeDef *hstep);

/// \brief Direction indicator
/// Symbolic names for the direction the motor is turning
typedef enum {
	DIRECTION_CCW = 0,  ///< Counter-Clockwise
	DIRECTION_CW = 1   ///< Clockwise
} Stepper_Direction;

/// Forces the library to compute a new instantaneous speed and set that as
/// the current speed. It is called by
/// the library:
/// \li  after each step
/// \li  after change to maxSpeed through setMaxSpeed()
/// \li  after change to acceleration through setAcceleration()
/// \li  after change to target position (relative or absolute) through
/// move() or moveTo()
void Stepper_computeNewSpeed(STEPPER_TypeDef *hstep);

/// Low level function to set the motor output pins
/// bit 0 of the mask corresponds to _pin[0]
/// bit 1 of the mask corresponds to _pin[1]
/// You can override this to impment, for example serial chip output insted of using the
/// output pins directly
void Stepper_setOutputPins(STEPPER_TypeDef *hstep, unsigned int mask);

/// Called to execute a step. Only called when a new step is
/// required. Subclasses may override to implement new stepping
/// interfaces. The default calls step1(), step2(), step4() or step8() depending on the
/// number of pins defined for the stepper.
/// \param[in] step The current step phase number (0 to 7)
void Stepper_step(STEPPER_TypeDef *hstep, long step);

/// Called to execute a step on a stepper driver (ie where pins == 1). Only called when a new step is
/// required. Subclasses may override to implement new stepping
/// interfaces. The default sets or clears the outputs of Step pin1 to step,
/// and sets the output of _pin2 to the desired direction. The Step pin (_pin1) is pulsed for 1 microsecond
/// which is the minimum STEP pulse width for the 3967 driver.
/// \param[in] step The current step phase number (0 to 7)
void Stepper_step1(STEPPER_TypeDef *hstep, long step);

long mm2steps(float);

bool Stepper_goToEndstop(STEPPER_TypeDef *hstep, GPIO_TypeDef *port, uint16_t pin, uint16_t max_distance);

#endif
