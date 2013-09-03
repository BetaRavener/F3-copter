/*
* F3-copter - STM32-F3 Discovery based tricopter
* Copyright (c) 2013 Ivan Sevcik - ivan-sevcik@hotmail.com
*
* This software is provided 'as-is', without any express or
* implied warranty. In no event will the authors be held
* liable for any damages arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute
* it freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented;
*    you must not claim that you wrote the original software.
*    If you use this software in a product, an acknowledgment
*    in the product documentation would be appreciated but
*    is not required.
*
* 2. Altered source versions must be plainly marked as such,
*    and must not be misrepresented as being the original software.
*
* 3. This notice may not be removed or altered from any
*    source distribution.
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

/*
 * CONTROLLER THEORY - PID CONTROLLER
 *
 * Standard form:
 *
 * MV(t) = K_p * ( e(t) + 1/T_i * INT[0->t](e(Tau))[dTau] + T_d * D[d/dt](e(t)) )
 *
 * e = SP - PV 	: Error
 * t 		   	: Present time
 * Tau		   	: Integration variable; values from time 0 to present t
 * SP			: Set point (desired state)
 * PV			: Process variable (sensor output)
 * MV			: Manipulated variable (process input - changes system state eg. engine)
 *
 * In this standard form, the parameters have a clear physical meaning.
 * In particular, the inner summation produces a new single error value
 * which is compensated for future and past errors. The addition of the proportional
 * and derivative components effectively predicts the error value at T_d seconds (or samples)
 * in the future, assuming that the loop control remains unchanged.
 * The integral component adjusts the error value to compensate for the sum of all past errors,
 * with the intention of completely eliminating them in T_i seconds (or samples).
 * The resulting compensated single error value is scaled by the single gain K_p.
 *
 * http://en.wikipedia.org/wiki/PID_controller#Ideal_versus_standard_PID_form
 * -------------------------------------------------------------------------------------------
 * The derivative of e(t) is mathematically identical to the negative of the derivative of PV
 * everywhere except when set point changes. And when set point changes, derivative on error
 * results in an undesirable control action called derivative kick.
 *
 * http://www.controlguru.com/wp/p76.html
 * -------------------------------------------------------------------------------------------
 * Constant PID sample time -- no need to integrate / derive constants, they can be applied
 * to controller time constants directly and only once.
 *
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-sample-time/
 * -------------------------------------------------------------------------------------------
 * Integral windup prevention
 * Comment on clamping by Will:
 * The difference is subtle but reduces the windup to zero instead of the really small error
 * that still exists with clamping the iTerm to the limit instead of clamping the output to
 * the limit.
 *
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-reset-windup/
 */

class Controller
{
public:
	Controller(float proportional, float integral, float derivative, float deltaT);

	float setpoint();
	void setpoint(float setpoint);

	void limitOutput(bool limit, float minLimit = 0, float maxLimit = 0);

	float process(float input, float(*interpolate)(float, float) = nullptr);

private:
	float _setpoint;
	float _manipulated;

	float _integralTerm;
	float _previousInput;

	// Controller gain constant K_p
	float _proportional;

	// Controller time constants 1/T_i and T_d
	float _integralRecip;
	float _derivative;

	// Support for output limiting
	bool _outputLimited;
	float _minLimit;
	float _maxLimit;
};

#endif
