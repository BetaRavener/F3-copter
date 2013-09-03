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

#include "uart.h"
#include "interrupt.h"
#include "systime.h"

#include <stm32f30x.h>

//TODO: needs __IO?
Uart* uart1Reg = nullptr;
Uart* uart2Reg = nullptr;
Uart* uart3Reg = nullptr;
Uart* uart4Reg = nullptr;
Uart* uart5Reg = nullptr;

Uart::Uart(USART_TypeDef* uart, uint32_t baudRate, uint32_t parity) :
_uart(uart),
_txBuffer(),
_rxBuffer(),
_canSend(true)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = baudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = parity;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	// Configure USART1
	USART_Init(uart, &USART_InitStructure);

	// Enable the USART1
	USART_Cmd(uart, ENABLE);

	// Register Uart
	uint8_t channel;
	if(_uart == USART1){
		uart1Reg = this;
		channel = USART1_IRQn;
	}
	else if(_uart == USART2){
		uart2Reg = this;
		channel = USART2_IRQn;
	}
	else if(_uart == USART3){
		uart3Reg = this;
		channel = USART3_IRQn;
	}
	else if(_uart == UART4){
		uart4Reg = this;
		channel = UART4_IRQn;
	}
	else if(_uart == UART5){
		uart5Reg = this;
		channel = UART5_IRQn;
	}

	// Enable interrupts
	// Enable interrupt on data received
	USART_ITConfig(_uart, USART_IT_RXNE, ENABLE);

	// Disable interrupt on data transfered (no data to send)
	USART_ITConfig(_uart, USART_IT_TXE, DISABLE);

	// Use low priority
	uint8_t priority = 1;
	uint8_t subPriority = 0;
	Interrupt::enable(channel, priority, subPriority);
}

Uart::~Uart()
{
	uint8_t channel;
	if(_uart == USART1){
		uart1Reg = nullptr;
		channel = USART1_IRQn;
	}
	else if(_uart == USART2){
		uart2Reg = nullptr;
		channel = USART2_IRQn;
	}
	else if(_uart == USART3){
		uart3Reg = nullptr;
		channel = USART3_IRQn;
	}
	else if(_uart == UART4){
		uart4Reg = nullptr;
		channel = UART4_IRQn;
	}
	else if(_uart == UART5){
		uart5Reg = nullptr;
		channel = UART5_IRQn;
	}

	USART_ITConfig(_uart, USART_IT_RXNE, DISABLE);
	USART_ITConfig(_uart, USART_IT_TXE, DISABLE);

	// Disable interrupt
	Interrupt::disable(channel);
}

void Uart::connect(GPIO_TypeDef* txPort, uint16_t txPin, uint8_t txAltFunction,
	     	 	   GPIO_TypeDef* rxPort, uint16_t rxPin, uint8_t rxAltFunction)
{
	// Configure port pin
	GPIO_InitTypeDef gpioConfig;
	GPIO_StructInit(&gpioConfig);

	// Pin selection mask
	gpioConfig.GPIO_Pin = 1 << txPin;
	gpioConfig.GPIO_Mode = GPIO_Mode_AF; // Use the alternative pin functions
	gpioConfig.GPIO_Speed = GPIO_Speed_50MHz; // GPIO speed - has nothing to do with the timer timing
	gpioConfig.GPIO_OType = GPIO_OType_PP; // Push-pull
	gpioConfig.GPIO_PuPd = GPIO_PuPd_UP; // Setup pull-up resistors
	GPIO_Init(txPort, &gpioConfig);
	GPIO_PinAFConfig(txPort, txPin, txAltFunction);

	// Pin selection mask
	gpioConfig.GPIO_Pin = 1 << rxPin;
	gpioConfig.GPIO_Mode = GPIO_Mode_AF; // Use the alternative pin functions
	gpioConfig.GPIO_Speed = GPIO_Speed_50MHz; // GPIO speed - has nothing to do with the timer timing
	gpioConfig.GPIO_OType = GPIO_OType_PP; // Push-pull
	gpioConfig.GPIO_PuPd = GPIO_PuPd_UP; // Setup pull-up resistors
	GPIO_Init(rxPort, &gpioConfig);
	GPIO_PinAFConfig(rxPort, rxPin, rxAltFunction);
}

bool Uart::empty()
{
	return _rxBuffer.empty();
}

uint8_t Uart::get()
{
	// TODO: If empty, wait for timeout, then return 0 (or false if redesigned)
	if(_rxBuffer.empty()) return 0;
	uint8_t byte = _rxBuffer.front();
	_rxBuffer.pop_front();
	return byte;
}

void Uart::put(uint8_t byte)
{
	_txBuffer.push_back(byte);
}

std::vector<uint8_t> Uart::read(int num)
{
	if(num <= 0)
		return std::vector<uint8_t>();

	std::vector<uint8_t> vec;
	int read = 0;

	while(read < num){
		// TODO: timeouts
		while(_rxBuffer.empty());
		vec.push_back(_rxBuffer.front());
		_rxBuffer.pop_front();
		read++;
	}
	return vec;
}

int Uart::write(const std::vector<uint8_t>& vec)
{
	if(vec.empty())
		return 0;

	for(auto it = vec.begin(); it != vec.end(); it++)
		_txBuffer.push_back(*it);

	if(_canSend){
		// Enable interrupt on transmit to let UART
		// send the data
		USART_ITConfig(_uart, USART_IT_TXE, ENABLE);
	}

	return vec.size();
}

void Uart::send()
{
	if(_txBuffer.empty()){
		_canSend = true;
		// Disable transmit interrupt if there's nothing else to send
		USART_ITConfig(_uart, USART_IT_TXE, DISABLE);
		return;
	}

	uint8_t byte = _txBuffer.front();
	_txBuffer.pop_front();
	USART_SendData(_uart, (uint16_t)byte);
	_canSend = false;

	// Give small time window to process values on the other side
	sleep(1, microsecond);
}

void Uart::receive()
{
	_rxBuffer.push_back(USART_ReceiveData(_uart));
}

// Interrupt handlers
uint32_t USART1_IRQHandler(void)
{
	while(uart1Reg == nullptr);
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
    	uart1Reg->receive();
    }
    else if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET){
    	uart1Reg->send();
    }
    return 0;
}

uint32_t USART2_IRQHandler(void)
{
	while(uart2Reg == nullptr);
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){
		uart2Reg->receive();
	}
	else if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET){
		uart2Reg->send();
	}
	return 0;
}

uint32_t USART3_IRQHandler(void)
{
	while(uart3Reg == nullptr);
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET){
		uart3Reg->receive();
	}
	else if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET){
		uart3Reg->send();
	}
	return 0;
}

uint32_t UART4_IRQHandler(void)
{
	while(uart4Reg == nullptr);
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET){
		uart4Reg->receive();
	}
	else if(USART_GetITStatus(UART4, USART_IT_TXE) != RESET){
		uart4Reg->send();
	}
	return 0;
}

uint32_t UART5_IRQHandler(void)
{
	while(uart5Reg == nullptr);
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET){
		uart1Reg->receive();
	}
	else if(USART_GetITStatus(UART5, USART_IT_TXE) != RESET){
		uart5Reg->send();
	}
	return 0;
}

