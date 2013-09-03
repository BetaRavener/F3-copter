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

#include "stopwatch.h"

extern const uint64_t systemTime;

Stopwatch::Stopwatch() :
_begin(0),
_end(0),
_running(false)
{
}

void Stopwatch::start()
{
	if(_running) return;

	_begin = systemTime;
	_running = true;
}

void Stopwatch::stop()
{
	if(!_running) return;

	_end = systemTime;
	_running = false;
}

void Stopwatch::reset()
{
	_begin = _end = systemTime;
	_running = false;
}

void Stopwatch::restart()
{
	_begin = _end = systemTime;
	_running = true;
}

uint64_t Stopwatch::elapsed(TimeUnit unit) const
{
	return ((_running ? systemTime : _end) - _begin) / (SYSTEM_TIME_RESOLUTION / unit);
}

bool Stopwatch::isRunning() const
{
	return _running;
}
