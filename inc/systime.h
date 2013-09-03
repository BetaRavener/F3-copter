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

#ifndef TIME_H
#define TIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Number of units in second
#define SYSTEM_TIME_RESOLUTION 1000000

typedef enum{second = 1, millisecond = 1000, microsecond = 1000000} TimeUnit;

void restartSystemTime();
uint64_t getSystemTime();
void sleep(uint64_t duration, TimeUnit unit = millisecond);

#ifdef __cplusplus
}
#endif

#endif
