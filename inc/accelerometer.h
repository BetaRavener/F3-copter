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

#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "math3d.h"

#include <deque>
#include <utility>

#include <stm32f3_discovery_lsm303dlhc.h>

class Accelerometer
{
public:
	enum Mode{BypassMode, FifoMode, StreamMode};

	/* Gyro is in bypass mode by default, filter enabled */
	Accelerometer(LSM303DLHCAcc_InitTypeDef& accInit, LSM303DLHCAcc_FilterConfigTypeDef& filterConfig, uint16_t maxBufferSize);

	int test();

	/* Read single value vector */
	math3d::Vector3<float> readValue();

	/* Discard first or all values in buffers */
	void discard(bool all = false);

	/* Select data storage mode */
	void selectMode(Mode mode);

	/* Change scale of sensor */
	void changeScale(uint8_t scale);

	/* Enable or disable high pass filter */
	void useHighPassFilter(bool use);

private:

	/* Retrieve all values stored in L3GD20 and store in buffers */
	void retrieveValues();

	// Clear all items from FIFO
	void clearFifo();

	/* Reset FIFO to re-enable data collection */
	void resetFifo();

	/* Buffer for storing raw data */
	std::deque<math3d::Vector3<int16_t> > _dataBuffer;

	/* Buffer for storing scale of _dataBuffer items - pair <scale, counter> */
	std::deque<std::pair<uint8_t, uint16_t> > _scaleBuffer;

	// Maximum buffer size
	uint16_t _maxBufferSize;

	// Sample time
	float _deltaT;
};

#endif
