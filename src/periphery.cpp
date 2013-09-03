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

#include "periphery.h"

#include "stm32f30x_rcc.h"

void Periphery::enable(Peripheries periphery)
{
	generalRccClockCmd(periphery, true);
}

void Periphery::disable(Peripheries periphery)
{
	generalRccClockCmd(periphery, false);
}

void Periphery::generalRccClockCmd(Peripheries periphery, bool enabled)
{
	FunctionalState state = enabled ? ENABLE : DISABLE;
	uint32_t pa;

	switch(periphery){
	case TIM2_P:   pa = RCC_APB1Periph_TIM2;   break;
	case TIM3_P:   pa = RCC_APB1Periph_TIM3;   break;
	case TIM4_P:   pa = RCC_APB1Periph_TIM4;   break;
	case TIM6_P:   pa = RCC_APB1Periph_TIM6;   break;
	case TIM7_P:   pa = RCC_APB1Periph_TIM7;   break;
	case WWDG_P:   pa = RCC_APB1Periph_WWDG;   break;
	case SPI2_P:   pa = RCC_APB1Periph_SPI2;   break;
	case SPI3_P:   pa = RCC_APB1Periph_SPI3;   break;
	case USART2_P: pa = RCC_APB1Periph_USART2; break;
	case USART3_P: pa = RCC_APB1Periph_USART3; break;
	case UART4_P:  pa = RCC_APB1Periph_UART4;  break;
	case UART5_P:  pa = RCC_APB1Periph_UART5;  break;
	case I2C1_P:   pa = RCC_APB1Periph_I2C1;   break;
	case I2C2_P:   pa = RCC_APB1Periph_I2C2;   break;
	case USB_P:    pa = RCC_APB1Periph_USB;    break;
	case CAN1_P:   pa = RCC_APB1Periph_CAN1;   break;
	case PWR_P:    pa = RCC_APB1Periph_PWR;    break;
	case DAC_P:    pa = RCC_APB1Periph_DAC;    break;

	case SYSCFG_P: pa = RCC_APB2Periph_SYSCFG; break;
	case TIM1_P:   pa = RCC_APB2Periph_TIM1;   break;
	case SPI1_P:   pa = RCC_APB2Periph_SPI1;   break;
	case TIM8_P:   pa = RCC_APB2Periph_TIM8;   break;
	case USART1_P: pa = RCC_APB2Periph_USART1; break;
	case TIM15_P:  pa = RCC_APB2Periph_TIM15;  break;
	case TIM16_P:  pa = RCC_APB2Periph_TIM16;  break;
	case TIM17_P:  pa = RCC_APB2Periph_TIM17;  break;

	case ADC34_P:  pa = RCC_AHBPeriph_ADC34;   break;
	case ADC12_P:  pa = RCC_AHBPeriph_ADC12;   break;
	case GPIOA_P:  pa = RCC_AHBPeriph_GPIOA;   break;
	case GPIOB_P:  pa = RCC_AHBPeriph_GPIOB;   break;
	case GPIOC_P:  pa = RCC_AHBPeriph_GPIOC;   break;
	case GPIOD_P:  pa = RCC_AHBPeriph_GPIOD;   break;
	case GPIOE_P:  pa = RCC_AHBPeriph_GPIOE;   break;
	case GPIOF_P:  pa = RCC_AHBPeriph_GPIOF;   break;
	case TS_P:     pa = RCC_AHBPeriph_TS;      break;
	case CRC_P:    pa = RCC_AHBPeriph_CRC;     break;
	case FLITF_P:  pa = RCC_AHBPeriph_FLITF;   break;
	case SRAM_P:   pa = RCC_AHBPeriph_SRAM;    break;
	case DMA2_P:   pa = RCC_AHBPeriph_DMA2;    break;
	case DMA1_P:   pa = RCC_AHBPeriph_DMA1;    break;

	default:
		return;
	}

	if(periphery >= TIM2_P && periphery <= DAC_P)
		RCC_APB1PeriphClockCmd(pa, state);
	else if(periphery >= SYSCFG_P && periphery <= TIM17_P)
		RCC_APB2PeriphClockCmd(pa, state);
	else if(periphery >= ADC34_P && periphery <= DMA1_P)
		RCC_AHBPeriphClockCmd(pa, state);
}
