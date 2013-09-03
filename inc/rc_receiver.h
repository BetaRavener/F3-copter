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

#ifndef RC_RECEIVER_H
#define RC_RECEIVER_H

// Max 16 channels
#define CHANNEL_N 8

#include <stdint.h>
#include <stm32f30x_tim.h>

class RcChannel
{
public:
	RcChannel(TIM_TypeDef* timer, uint8_t timerChannel);
	~RcChannel();

	void connect(GPIO_TypeDef* port, uint16_t pin, uint8_t altFunction);
private:
	TIM_TypeDef* _timer;
	uint8_t _timerChannel;
};

class RcReceiver
{
public:
	RcReceiver();
	~RcReceiver();

	// Timer must have enabled interrupts
	void addChannel(uint8_t channel, TIM_TypeDef* timer, uint8_t timerChannel, GPIO_TypeDef* port, uint16_t pin, uint8_t altFunction);
	void removeChannel(uint8_t channel);

	float pulseWidth(uint8_t channel);

	// Returns normalized reading of channel in range <0, 1>
	float normalizedReading(uint8_t channel);

	// Timer periphery must be enabled first e.g.
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	static void configureTimer(TIM_TypeDef* timer);

	static void handleInterrupt(uint8_t channel, TIM_TypeDef* timer, uint8_t timerChannel);

private:
	RcChannel* _channels[CHANNEL_N];
};

// Interrupt handlers
#ifdef __cplusplus
extern "C" {
#endif

void TIM3_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
