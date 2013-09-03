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

#ifndef PERIPHERY_H
#define PERIPHERY_H

#include <stdint.h>

class Periphery
{
public:
	typedef enum{ TIM2_P, TIM3_P, TIM4_P, TIM6_P, TIM7_P, WWDG_P, SPI2_P,
				  SPI3_P, USART2_P, USART3_P, UART4_P, UART5_P, I2C1_P,
				  I2C2_P, USB_P, CAN1_P, PWR_P, DAC_P, SYSCFG_P, TIM1_P,
				  SPI1_P, TIM8_P, USART1_P, TIM15_P, TIM16_P, TIM17_P,
				  ADC34_P, ADC12_P, GPIOA_P, GPIOB_P, GPIOC_P, GPIOD_P,
				  GPIOE_P, GPIOF_P, TS_P, CRC_P, FLITF_P, SRAM_P, DMA2_P,
				  DMA1_P } Peripheries;

	static void enable(Peripheries periphery);
	static void disable(Peripheries periphery);

private:
	static void generalRccClockCmd(Peripheries periphery, bool enabled);
};

#endif
