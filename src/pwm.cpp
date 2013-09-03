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

#include "pwm.h"

#include <stm32f30x_tim.h>

// 2MHz Base frequency of the pwm timer
const uint32_t pwmTimerFrequency = 2e6;

Pwm::Pwm(TIM_TypeDef* timer, uint8_t channel) :
_timer(timer),
_channel(channel),
_dutyCycle(0)
{
	TIM_OCInitTypeDef channelConfig;
	TIM_OCStructInit(&channelConfig);

	channelConfig.TIM_OCMode = TIM_OCMode_PWM1;
	channelConfig.TIM_OutputState = TIM_OutputState_Enable;
	channelConfig.TIM_Pulse = 0;
	channelConfig.TIM_OCPolarity = TIM_OCPolarity_High; // Pulse polarity
	channelConfig.TIM_OCIdleState = TIM_OCIdleState_Set;

	switch(_channel){
	case 1:
		TIM_OC1Init(_timer, &channelConfig);
		TIM_OC1PreloadConfig(_timer, TIM_OCPreload_Enable);
		break;
	case 2:
		TIM_OC2Init(_timer, &channelConfig);
		TIM_OC2PreloadConfig(_timer, TIM_OCPreload_Enable);
		break;
	case 3:
		TIM_OC3Init(_timer, &channelConfig);
		TIM_OC3PreloadConfig(_timer, TIM_OCPreload_Enable);
		break;
	case 4:
		TIM_OC4Init(_timer, &channelConfig);
		TIM_OC4PreloadConfig(_timer, TIM_OCPreload_Enable);
		break;
	}
}

float Pwm::dutyCycle()
{
	return _dutyCycle;
}

void Pwm::dutyCycle(float dc)
{
	_dutyCycle = dc;
	// TODO: create timer class -> period() will return (_timer->ARR + 1)
	// pwmConfig() instead Pwm::configureTimer..
	uint32_t ticks = (_timer->ARR + 1) * _dutyCycle;

	switch(_channel)
	{
	case 1:
		TIM_SetCompare1(_timer, ticks);
		break;
	case 2:
		TIM_SetCompare2(_timer, ticks);
		break;
	case 3:
		TIM_SetCompare3(_timer, ticks);
		break;
	case 4:
		TIM_SetCompare4(_timer, ticks);
		break;
	}
}

float Pwm::pulseWidth()
{
	return ((_dutyCycle * (_timer->ARR + 1)) / (float)pwmTimerFrequency);
}

void Pwm::pulseWidth(float pulseWidth)
{
	// Set dutyCycle to equivalent of pulseWidth
	// First determine number of pwmTimer ticks in pulseWidth
	// and then divide by pwmTimer ticks in one pwm period
	dutyCycle((pwmTimerFrequency * pulseWidth) / (float)(_timer->ARR + 1));
}

void Pwm::connect(GPIO_TypeDef* port, uint16_t pin, uint8_t altFunction)
{
	// Configure port pin
	GPIO_InitTypeDef gpioConfig;
	GPIO_StructInit(&gpioConfig);

	// Pin selection mask
	gpioConfig.GPIO_Pin = 1 << pin;
	gpioConfig.GPIO_Mode = GPIO_Mode_AF; // Use the alternative pin functions
	gpioConfig.GPIO_Speed = GPIO_Speed_50MHz; // GPIO speed - has nothing to do with the timer timing
	gpioConfig.GPIO_OType = GPIO_OType_PP; // Push-pull
	gpioConfig.GPIO_PuPd = GPIO_PuPd_UP; // Setup pull-up resistors
	GPIO_Init(port, &gpioConfig);

	// Connect timer output to the pin
	GPIO_PinAFConfig(port, pin, altFunction);
}

void Pwm::configureTimer(TIM_TypeDef* timer, uint32_t pwmFrequency)
{
	// Clock divider
	uint32_t prescaler = ((SystemCoreClock / pwmTimerFrequency) - 1);

	// Calculate the period for a given pwm frequency
	// For 200 Hz: 2MHz / 200Hz = 10000 ticks == 1 / 200 Hz = 5 milliseconds
	uint16_t pwmPeriod = pwmTimerFrequency / pwmFrequency;

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
	TIM_TimeBaseStructure.TIM_Period = pwmPeriod - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);

	TIM_ARRPreloadConfig(timer, DISABLE);

	if(IS_TIM_LIST6_PERIPH(timer))
		TIM_CtrlPWMOutputs(timer, ENABLE);

	TIM_Cmd(timer, ENABLE);
}
