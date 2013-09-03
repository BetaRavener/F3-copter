
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

#include "controller.h"

Controller::Controller(float proportional, float integral, float derivative, float deltaT) :
_setpoint(0),
_manipulated(0),
_integralTerm(0),
_previousInput(0),
_proportional(proportional),
_integralRecip(deltaT / integral),
_derivative(derivative / deltaT),
_outputLimited(false),
_minLimit(0),
_maxLimit(0)
{

}

float Controller::setpoint()
{
	return _setpoint;
}

void Controller::setpoint(float setpoint)
{
	_setpoint = setpoint;
}

void Controller::limitOutput(bool limit, float minLimit, float maxLimit)
{
	_outputLimited = limit;
	_minLimit = minLimit;
	_maxLimit = maxLimit;
}

float Controller::process(float input, float(*interpolate)(float, float))
{
	float sum, error, inputDif;

	if(interpolate == nullptr){
		error = _setpoint - input;
		inputDif = _previousInput - input;
	}
	else{
		error = interpolate(input, _setpoint);
		inputDif = interpolate(input, _previousInput);
	}

	// Accumulated error adjusted by integral constant
	_integralTerm += error * _integralRecip;

	// Error derivative subsidized by input derivative -- negative of error derivative
	sum = error + _integralTerm + inputDif * _derivative;
	_previousInput = input;

	_manipulated = sum * _proportional;

	// Windup prevention on both integral term and output
	if(_outputLimited){
		if(_manipulated > _maxLimit){
			_integralTerm -= _manipulated - _maxLimit;
			_manipulated = _maxLimit;
		}
		else if(_manipulated < _minLimit){
			_integralTerm += _minLimit - _manipulated;
			_manipulated = _minLimit;
		}
	}

	return _manipulated;
}
