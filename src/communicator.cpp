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

#include "communicator.h"

#include "stm32f30x.h"

#define START_BYTE 	0xAA
#define STRING_COM	0x80
#define UINT32_COM 	0x81
#define FLOAT_COM 	0x82
#define COMMAND_COM 0x83

#define FIRST_BYTE

Communicator::Communicator(Source source) :
_source(source),
_uart(USART1, 115200),
_commandByte(0),
_atStart(false)

{
	// Connectors must be crossed TX->RX and RX->TX
	_uart.connect(GPIOA, 9, 7, GPIOA, 10, 7);
}

void Communicator::send(std::string s)
{
	std::vector<uint8_t> vec;
	vec.reserve(s.size()+3);
	vec.push_back(START_BYTE);
	vec.push_back(STRING_COM);
	vec.push_back((uint8_t)s.size());
	for(auto it = s.begin(); it != s.end(); it++)
		vec.push_back((uint8_t)*it);
	_uart.write(vec);
}

// TODO: CheckSum
void Communicator::send(uint32_t ui)
{
	uint8_t *c = (uint8_t*)(&ui);
	std::vector<uint8_t> vec;
	vec.reserve(4+2);
	vec.push_back(START_BYTE);
	vec.push_back(UINT32_COM);
	vec.push_back(c[0]);
	vec.push_back(c[1]);
	vec.push_back(c[2]);
	vec.push_back(c[3]);
	_uart.write(vec);
}

void Communicator::send(float f)
{
	uint8_t *c = (uint8_t*)(&f);
	std::vector<uint8_t> vec;
	vec.reserve(4+2);
	vec.push_back(START_BYTE);
	vec.push_back(FLOAT_COM);
	vec.push_back(c[0]);
	vec.push_back(c[1]);
	vec.push_back(c[2]);
	vec.push_back(c[3]);
	_uart.write(vec);
}

void Communicator::sendRaw(std::string s)
{
	std::vector<uint8_t> vec;
	vec.reserve(s.size());
	for(auto it = s.begin(); it != s.end(); it++)
		vec.push_back((uint8_t)*it);
	_uart.write(vec);
}

void Communicator::discard()
{
	_atStart = false;
	discardUntilStart();
}

bool Communicator::empty()
{
	return (!_atStart) && _uart.empty();
}

bool Communicator::receive(CommandId& commandId)
{
	if(!discardUntilStart() || _commandByte != COMMAND_COM)
		return false;

	_atStart = false;
	// Get 2 bytes of uint16_t (command ID)
	std::vector<uint8_t> vec = _uart.read(2);

	// Get checksum
	// uint8_t checkSum = _uart.get();

	// TODO: if checkSum is correct..

	// Map memory and rebuild variable
	uint8_t *c = (uint8_t*)(&commandId);
	c[0] = vec[0];
	c[1] = vec[1];

	return true;
}

bool Communicator::receive(std::string& s)
{
	if(!discardUntilStart() || _commandByte != STRING_COM)
		return false;

	_atStart = false;
	s.clear();
	uint8_t length = _uart.get();
	uint8_t ch;
	for(int i = 0; i < length; i++){
		ch = _uart.get();
		if(ch < 0x80)
			s += ch;
		else if(ch == START_BYTE){
			// Reinitialize communication state when some bytes were lost
			_atStart = true;
			_commandByte = _uart.get();
			break;
		}
		else
			break;
	}

	return true;
}

bool Communicator::receive(uint32_t& ui)
{
	if(!discardUntilStart() || _commandByte != UINT32_COM)
		return false;

	_atStart = false;
	// Get 4 bytes of uint32_t
	std::vector<uint8_t> vec = _uart.read(4);

	// Get checksum
	// uint8_t checkSum = _uart.get();

	// TODO: if checkSum is correct..
	// Map memory and rebuild variable
	uint8_t *c = (uint8_t*)(&ui);
	c[0] = vec[0];
	c[1] = vec[1];
	c[2] = vec[2];
	c[3] = vec[3];

	return true;
}

bool Communicator::receive(float& f)
{
	if(!discardUntilStart() || _commandByte != FLOAT_COM)
		return false;

	_atStart = false;
	// Get 4 bytes of float
	std::vector<uint8_t> vec = _uart.read(4);

	// Get checksum
	// uint8_t checkSum = _uart.get();

	// TODO: if checkSum is correct..
	// Map memory and rebuild variable
	uint8_t *c = (uint8_t*)(&f);
	c[0] = vec[0];
	c[1] = vec[1];
	c[2] = vec[2];
	c[3] = vec[3];

	return true;
}

bool Communicator::discardUntilStart()
{
	while(!_atStart)
	{
		if(_uart.empty())
			return false;
		_atStart = _uart.get() == START_BYTE;
	}
	// TODO: timeout
	while(_uart.empty());
	_commandByte = _uart.get();
	return true;
}
