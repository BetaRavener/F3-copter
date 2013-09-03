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

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <deque>
#include <vector>

#include "stm32f30x_usart.h"

class Uart
{
public:
	Uart(USART_TypeDef* uart, uint32_t baudRate, uint32_t parity = USART_Parity_No);
	~Uart();

	void connect(GPIO_TypeDef* txPort, uint16_t txPin, uint8_t txAltFunction,
			     GPIO_TypeDef* rxPort, uint16_t rxPin, uint8_t rxAltFunction);

	bool empty();

	uint8_t get();
	void put(uint8_t byte);

	std::vector<uint8_t> read(int num = 1);
	int write(const std::vector<uint8_t>& vec);

	// Internal service function for interrupt handling
	void send();
	// Internal service function for interrupt handling
	void receive();

private:
	USART_TypeDef* _uart;

	std::deque<uint8_t> _txBuffer;
	std::deque<uint8_t> _rxBuffer;

	bool _canSend;
};

#ifdef __cplusplus
extern "C" {
#endif

uint32_t USART1_IRQHandler(void);
uint32_t USART2_IRQHandler(void);
uint32_t USART3_IRQHandler(void);
uint32_t UART4_IRQHandler(void);
uint32_t UART5_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
