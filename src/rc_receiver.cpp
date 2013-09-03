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

#include "rc_receiver.h"

#include "systime.h"
#include "common.h"

// Maximum expected pulse width is 2 milliseconds
// with 2MHz capture timer, 1 tick is 0.5 microseconds
// and maximum expected tick count is 4000. Maximum
// allowed tick count is 0x10000 = 65536 ~ 33 milliseconds
static const uint32_t captureTimerFrequency = 2e6;

// Maximum accepted pulse width is 4 milliseconds
static const uint32_t maxCaptureTicks = 0.004 * captureTimerFrequency;

// Maximum time for remembering pulse width is 50 milliseconds
static const uint32_t maxTimeDifference = 0.05 * SYSTEM_TIME_RESOLUTION;

// Minimum number of continuous samples for signal to be valid
static const uint8_t minContinuousSamples = 5;

static const float minPulseWidth = 1e-3;

static const float maxPulseWidth = 2e-3;

uint64_t channelTimeStamps[CHANNEL_N] = {0};
uint32_t channelCaptures[CHANNEL_N] = {0};
uint16_t channelStarts[CHANNEL_N] = {0};
uint8_t channelContinuous[CHANNEL_N] = {0};
uint16_t channelFlags = 0;

RcChannel::RcChannel(TIM_TypeDef* timer, uint8_t timerChannel) :
_timer(timer),
_timerChannel(timerChannel)
{
	uint16_t timChannel, timInterrupt;
	switch(timerChannel){
	case 1:
		timChannel = TIM_Channel_1;
		timInterrupt = TIM_IT_CC1;
		break;
	case 2:
		timChannel = TIM_Channel_2;
		timInterrupt = TIM_IT_CC2;
		break;
	case 3:
		timChannel = TIM_Channel_3;
		timInterrupt = TIM_IT_CC3;
		break;
	case 4:
		timChannel = TIM_Channel_4;
		timInterrupt = TIM_IT_CC4;
		break;
	}

	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = timChannel;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x3; // Filters noise
	TIM_ICInit(timer, &TIM_ICInitStructure);

	// Enable interrupt on capture/compare register
	TIM_ClearITPendingBit(timer, timInterrupt);
	TIM_ITConfig(timer, timInterrupt, ENABLE);
}

RcChannel::~RcChannel()
{
	// Disable interrupt
	uint16_t timInterrupt;
	switch(_timerChannel){
	case 1: timInterrupt = TIM_IT_CC1; break;
	case 2: timInterrupt = TIM_IT_CC2; break;
	case 3:	timInterrupt = TIM_IT_CC3; break;
	case 4:	timInterrupt = TIM_IT_CC4; break;
	}
	TIM_ITConfig(_timer, timInterrupt, DISABLE);
	TIM_ClearITPendingBit(_timer, timInterrupt);
}

// TODO: crate Pin class and derive connect from it
// TODO: AF is not consistent (look at AF12-15)
void RcChannel::connect(GPIO_TypeDef* port, uint16_t pin, uint8_t altFunction)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//GPIO_StructInit (&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = 1 << pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(port, &GPIO_InitStructure);
	GPIO_PinAFConfig (port, pin, altFunction);
}

RcReceiver::RcReceiver()
{
	for(int i = 0; i < CHANNEL_N; i++)
		_channels[i] = nullptr;
}

RcReceiver::~RcReceiver()
{
	for(int i = 0; i < CHANNEL_N; i++){
		delete _channels[i];
		_channels[i] = nullptr;
	}
}

void RcReceiver::addChannel(uint8_t channel, TIM_TypeDef* timer, uint8_t timerChannel, GPIO_TypeDef* port, uint16_t pin, uint8_t altFunction)
{
	if(channel >= CHANNEL_N || _channels[channel] != nullptr)
		return;

	_channels[channel] = new RcChannel(timer, timerChannel);
	_channels[channel]->connect(port, pin, altFunction);

	channelTimeStamps[channel] = getSystemTime();
	channelCaptures[channel] = 0;
	channelStarts[channel] = 0;
	channelContinuous[channel] = 0;
	channelFlags &= ~(1 << channel);
}

void RcReceiver::removeChannel(uint8_t channel)
{
	if(channel >= CHANNEL_N && _channels[channel] != nullptr)
		return;

	delete _channels[channel];
	_channels[channel] = nullptr;
}

float RcReceiver::pulseWidth(uint8_t channel)
{
	if(channel >= CHANNEL_N && _channels[channel] != nullptr)
		return 0;

	// If signal was lost, return zero pulse width
	if(getSystemTime() - channelTimeStamps[channel] > maxTimeDifference)
		return 0;

	return ((float)channelCaptures[channel]) / captureTimerFrequency;
}

float RcReceiver::normalizedReading(uint8_t channel)
{
	float pw = pulseWidth(channel);

	return limit((pw - minPulseWidth) / (maxPulseWidth - minPulseWidth), 0, 1);
}

void RcReceiver::configureTimer(TIM_TypeDef* timer)
{
	TIM_DeInit(timer);

	// Clock divider
	uint32_t prescaler = ((SystemCoreClock / captureTimerFrequency) - 1);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF; // Use full period
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);

	TIM_ARRPreloadConfig(timer, DISABLE);

	TIM_Cmd(timer, ENABLE);
}

void RcReceiver::handleInterrupt(uint8_t channel, TIM_TypeDef* timer, uint8_t timerChannel)
{
	uint16_t capture;
	switch(timerChannel)
	{
	case 1: capture = TIM_GetCapture1(timer); break;
	case 2: capture = TIM_GetCapture2(timer); break;
	case 3: capture = TIM_GetCapture3(timer); break;
	case 4: capture = TIM_GetCapture4(timer); break;
	default: return;
	}

	if((channelFlags & 1 << channel) == 0){
		channelStarts[channel] = capture;
		channelFlags ^= 1 << channel;
	}
	else{
		uint32_t width;

		// Compute capture value
		if (capture > channelStarts[channel])
			width = (uint32_t)capture - (uint32_t)channelStarts[channel];
		else
			width = (uint32_t)channelStarts[channel] + 0x10000 - (uint32_t)capture;

		// If pulse is wider than 4ms, don't update value and save this capture as
		// pulse start (pulse can't be wider than 4ms, if it is, we are measuring
		// idle part of pulse).
		if(width < maxCaptureTicks){

			if(getSystemTime() - channelTimeStamps[channel] < maxTimeDifference){
				if(channelContinuous[channel] > minContinuousSamples)
					channelCaptures[channel] = width;
				else
					channelContinuous[channel]++;
			}
			else
				channelContinuous[channel] = 0;

			channelTimeStamps[channel] = getSystemTime();
			channelFlags ^= 1 << channel;
		}
		else
			channelStarts[channel] = capture;
	}
}

// Interrupt handlers
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_CC1)){
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		RcReceiver::handleInterrupt(0, TIM3, 1);
	}

	if(TIM_GetITStatus(TIM3, TIM_IT_CC2)){
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		RcReceiver::handleInterrupt(1, TIM3, 2);
	}

	if(TIM_GetITStatus(TIM3, TIM_IT_CC3)){
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		RcReceiver::handleInterrupt(2, TIM3, 3);
	}

	if(TIM_GetITStatus(TIM3, TIM_IT_CC4)){
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
		RcReceiver::handleInterrupt(3, TIM3, 4);
	}
}

