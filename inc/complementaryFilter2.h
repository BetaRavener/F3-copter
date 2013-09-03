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

#ifndef COMPLEMENTARY_FILTER_2_H
#define COMPLEMENTARY_FILTER_2_H

#include "math3d.h"

class ComplementaryFilter2
{
public:
	ComplementaryFilter2(float deltaT, float timeConstant);

	void reset();

	math3d::Vector3<float> addSample(math3d::Vector3<float> lowPassSample, math3d::Vector3<float> highPassSample);
	math3d::Vector3<float> readState();
private:
	math3d::Vector3<float> _state;
	float _factor;
};

#endif
