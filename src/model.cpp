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

#include "model.h"
#include "periphery.h"
#include "common.h"

#include <stm32f30x_rcc.h>
#include "math3d.h"

#include <cmath>

#define ROBBE_FS_500_MIN_PW 0.9e-3
#define ROBBE_FS_500_MAX_PW	2.1e-3

// Fraction of throttle can be used for maneuvering purposes in one axis
static float maneuverFraction = 0.25;

Model::Model() :
engines{Engine(TIM1, 1),
	    Engine(TIM1, 2),
	    Engine(TIM1, 3)},
servo(TIM1, 4, ROBBE_FS_500_MIN_PW, ROBBE_FS_500_MAX_PW)
{
	engines[Rear].throttle(1.);
	engines[Right].throttle(.5);
	engines[Left].throttle(0);
	servo.normalizedAngle(0);

    // Connect PWM outputs for engines to pins
    engines[Rear].connect(GPIOE, 9, 2);
    engines[Right].connect(GPIOE, 11, 2);
    engines[Left].connect(GPIOE, 13, 2);
    servo.connect(GPIOE, 14, 2);
}

void Model::update(float throttle, math3d::Vector3<float> rotation)
{
	float rearAdjustment, rightAdjustment, leftAdjustment;

	// Initialize adjustments values
	rearAdjustment = rightAdjustment = leftAdjustment = 0.0f;

	// Adjust for each engine, based on mathematical model and specified rotation
	// Rotation is in range <-1, 1>
	if(rotation[0] > 0.0f)
	{
		rearAdjustment += rotation[0];
	}
	else if(rotation[0] < 0.0f)
	{
		rightAdjustment -= rotation[0];
		leftAdjustment -= rotation[0];
	}

	if(rotation[1] > 0.0f)
	{
		rearAdjustment += rotation[1] * 0.5f;
		rightAdjustment += rotation[1];
	}
	else if(rotation[1] < 0.0f)
	{
		rearAdjustment -= rotation[1] * 0.5f;
		leftAdjustment -= rotation[1];
	}

	// Apply adjustments to engine throttle, limit to allowed range and change it
	engines[Rear].throttle(limit(throttle * (1 - maneuverFraction * rearAdjustment), 0.0f, 1.0f));
	engines[Right].throttle(limit(throttle * (1 - maneuverFraction * rightAdjustment), 0.0f, 1.0f));
	engines[Left].throttle(limit(throttle * (1 - maneuverFraction * leftAdjustment), 0.0f, 1.0f));

	// Servo is controlled directly as there is no math behind yaw mechanism
	// TODO: fix by negating if servo "polarity" is different
	servo.normalizedAngle(rotation[2]);
}
