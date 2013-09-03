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

#include "complementaryFilter2.h"

ComplementaryFilter2::ComplementaryFilter2(float deltaT, float timeConstant) :
_state(math3d::ZeroVector),
_factor(timeConstant / (timeConstant + deltaT))
{
}

void ComplementaryFilter2::reset()
{
	_state = math3d::ZeroVector;
}

math3d::Vector3<float> ComplementaryFilter2::addSample(math3d::Vector3<float> lowPassSample, math3d::Vector3<float> highPassSample)
{
	_state = (_state + highPassSample) * _factor + lowPassSample * (1 - _factor);
	return _state;
}

math3d::Vector3<float> ComplementaryFilter2::readState()
{
	return _state;
}
