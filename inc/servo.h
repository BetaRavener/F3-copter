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

#ifndef SERVO_H
#define SERVO_H

#include "pwm.h"

class Servo
{
public:
	Servo(TIM_TypeDef* timer, uint8_t channel, float minPulseWidth = 1e-3, float maxPulseWidth = 2e-3, float maxAngle = 0, float angle = 0);

	float angle();
	// Angle in range <-Pi/2, Pi/2>, depends on maxAngle constant
	void angle(float angle);

	float normalizedAngle();
	// Angle in range <0, 1>, doesn't depend on maxAngle constant
	void normalizedAngle(float angle);

	float minPulseWidth();
	void minPulseWidth(float minPulseWidth);

	float maxPulseWidth();
	void maxPulseWidth(float maxPulseWidth);

	void connect(GPIO_TypeDef* port, uint16_t pin, uint8_t altFunction);

private:
	Pwm _pwm;
	float _maxAngle;
	float _minPulseWidth;
	float _maxPulseWidth;
};

#endif
