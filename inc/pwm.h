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

#ifndef PWM_H
#define PWM_H

#include <stdint.h>

#include <stm32f30x.h>

// http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00058181.pdf

class Pwm
{
public:
	Pwm(TIM_TypeDef* timer, uint8_t channel);

	float dutyCycle();
	void dutyCycle(float dc);

	float pulseWidth();
	void pulseWidth(float pulseWidth);

	// Port must be enabled
	void connect(GPIO_TypeDef* port, uint16_t pin, uint8_t altFunction);

	// Timer periphery must be enabled first e.g.
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	static void configureTimer(TIM_TypeDef* timer, uint32_t pwmFrequency);

private:
	float _dutyCycle;
	TIM_TypeDef* _timer;
	uint8_t _channel;
};

#endif
