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

#include "servo.h"

#include "math3d.h"

Servo::Servo(TIM_TypeDef* timer, uint8_t channel, float minPulseWidth, float maxPulseWidth, float maxAngle, float angle) :
_pwm(timer, channel),
_maxAngle(maxAngle),
_minPulseWidth(minPulseWidth),
_maxPulseWidth(maxPulseWidth)
{
	if(maxAngle > 0)
		this->angle(angle);
	else
		this->normalizedAngle(0);
}

float Servo::angle()
{
	// assert(_maxAngle > 0);
	return (_pwm.pulseWidth() - _minPulseWidth) / (_maxPulseWidth - _minPulseWidth) * (2 * _maxAngle) - _maxAngle;
}

void Servo::angle(float angle)
{
	// assert(_maxAngle > 0);
	_pwm.pulseWidth(_minPulseWidth + (angle + _maxAngle) / (2 * _maxAngle) * (_maxPulseWidth - _minPulseWidth));
}

float Servo::normalizedAngle()
{
	return (_pwm.pulseWidth() - _minPulseWidth) / (_maxPulseWidth - _minPulseWidth);
}

void Servo::normalizedAngle(float angle)
{
	_pwm.pulseWidth(_minPulseWidth + angle * (_maxPulseWidth - _minPulseWidth));
}

float Servo::minPulseWidth()
{
	return _minPulseWidth;
}

void Servo::minPulseWidth(float minPulseWidth)
{
	_minPulseWidth = minPulseWidth;
}

float Servo::maxPulseWidth()
{
	return _maxPulseWidth;
}

void Servo::maxPulseWidth(float maxPulseWidth)
{
	_maxPulseWidth = maxPulseWidth;
}

void Servo::connect(GPIO_TypeDef* port, uint16_t pin, uint8_t altFunction)
{
	_pwm.connect(port, pin, altFunction);
}
