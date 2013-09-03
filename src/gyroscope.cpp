
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

#include "gyroscope.h"
#include "systime.h"
#include "stopwatch.h"

/* sensitivity = 1 / (dps/digit) */ 
#define L3G_Sensitivity_250dps      114.28571428571428571428571428571   /*!< gyroscope sensitivity is 8.75 mdps/digit with 250 dps full scale */
#define L3G_Sensitivity_500dps      57.142857142857142857142857142857   /*!< gyroscope sensitivity is 17.5 mdps/digit with 500 dps full scale */
#define L3G_Sensitivity_2000dps     14.285714285714285714285714285714   /*!< gyroscope sensitivity is 70 mdps/digit with 2000 dps full scale */

// Output data rate in Hz
#define L3G_ODR_95					95
#define L3G_ODR_190					190
#define L3G_ODR_380					380
#define L3G_ODR_760					760


#define CHANGE_DELAY                5

#define BYPASS_MODE					0x00
#define FIFO_MODE					0x20
#define STREAM_MODE					0x40

#define FIFO_ENABLED				0x40
#define FIFO_DISABLED				0x00

Gyroscope::Gyroscope(L3GD20_InitTypeDef& gyroInit, L3GD20_FilterConfigTypeDef& filterConfig, uint16_t maxBufferSize) :
_maxBufferSize(maxBufferSize)
{
	switch(gyroInit.Output_DataRate)
	{
	case L3GD20_OUTPUT_DATARATE_1:
		_deltaT = 1.0 / L3G_ODR_95;
		break;
	case L3GD20_OUTPUT_DATARATE_2:
		_deltaT = 1.0 / L3G_ODR_190;
		break;
	case L3GD20_OUTPUT_DATARATE_3:
		_deltaT = 1.0 / L3G_ODR_380;
		break;
	case L3GD20_OUTPUT_DATARATE_4:
		_deltaT = 1.0 / L3G_ODR_760;
		break;
	default:
		return;
	}

    /* Configure Mems L3GD20 */ 
    L3GD20_Init(&gyroInit);
    L3GD20_FilterConfig(&filterConfig);
    selectMode(BypassMode);
    useHighPassFilter(true);
    
    /* Initialize scale buffer */
    _scaleBuffer.push_back(std::make_pair(gyroInit.Full_Scale, 0));
}

int Gyroscope::test()
{
	Stopwatch sw;
	selectMode(Gyroscope::FifoMode);
	discard(true);
	clearFifo();
	sw.start();
	while(sw.elapsed() < 1000)
	{
		retrieveValues();
	}

	// TODO: Data rate je 2x vyssi nez by mal byt

	return _dataBuffer.size();
}
    
math3d::Vector3<float> Gyroscope::readValue()
{
    /* First retrieve new values from L3GD20 */
     retrieveValues();
    
    if (_dataBuffer.size() <= 0)
        return math3d::ZeroVector;
        
    float sensitivity;

    /* Switch the sensitivity value */
    switch(_scaleBuffer.front().first & 0x30)
    {
    case L3GD20_FULLSCALE_250:
        sensitivity = L3G_Sensitivity_250dps;
        break;

    case L3GD20_FULLSCALE_500:
        sensitivity = L3G_Sensitivity_500dps;
        break;

    case L3GD20_FULLSCALE_2000:
        sensitivity = L3G_Sensitivity_2000dps;
        break;
        
    default:
        sensitivity = 0;
        break;
    }
    
    /* Divide by sensitivity and convert to radians*/
    math3d::Vector3<float> ret = math3d::Vector3<float>(_dataBuffer.front()) * (math3d::radiansInDegree / sensitivity);
    
    discard();
    return ret;
}

void Gyroscope::discard(bool all)
{
    if (_dataBuffer.size() <= 0)
        return;

    if (all == false)
    {
        _dataBuffer.pop_front();
        if(_scaleBuffer.front().second > 0 && --_scaleBuffer.front().second == 0 && _scaleBuffer.size() > 1)
            _scaleBuffer.pop_front();
    }
    else
    {
        _dataBuffer.clear();
        int scale = _scaleBuffer.back().first;
        _scaleBuffer.clear();
        _scaleBuffer.push_back(std::make_pair(scale, 0));
    }
}

void Gyroscope::selectMode(Mode mode)
{
    uint8_t ctrl5, fifoCtrl;
    uint8_t fifoEn, fifoMode;
    
    switch(mode)
    {
    case BypassMode:
        fifoEn = FIFO_DISABLED;
        fifoMode = BYPASS_MODE;
        break;
    case FifoMode:
        fifoEn = FIFO_ENABLED;
        fifoMode = FIFO_MODE;
        break;
    case StreamMode:
        fifoEn = FIFO_ENABLED;
        fifoMode = STREAM_MODE;
        break;
    default:
        return;
    }
    
    L3GD20_Read(&ctrl5, L3GD20_CTRL_REG5_ADDR, 1);
    ctrl5 = (ctrl5 & (~0x40)) | fifoEn;
    L3GD20_Write(&ctrl5, L3GD20_CTRL_REG5_ADDR, 1);
    
    L3GD20_Read(&fifoCtrl, L3GD20_FIFO_CTRL_REG_ADDR, 1);
    fifoCtrl = (fifoCtrl & (~0xE0)) | fifoMode;
    L3GD20_Write(&fifoCtrl, L3GD20_FIFO_CTRL_REG_ADDR, 1);

    // Clear FIFO from previously stored data
    if(fifoEn == FIFO_ENABLED)
    	clearFifo();
}

void Gyroscope::changeScale(uint8_t scale)
{
    uint8_t ctrl4, fifoCtrl;

    scale &= 0x30;

    L3GD20_Read(&fifoCtrl, L3GD20_FIFO_CTRL_REG_ADDR, 1);
    bool fifoMode = (fifoCtrl & 0x70) != 0;

    /* Retrieve stored values before changing scale */
    retrieveValues();
    
    /* Read current value from CTRL_REG4 register */
    L3GD20_Read(&ctrl4, L3GD20_CTRL_REG4_ADDR, 1);

    /* Change scale */
    ctrl4 = (ctrl4 & ~0x30) | scale;
                    
    /* Write new value to CTRL_REG4 regsister */
    L3GD20_Write(&ctrl4, L3GD20_CTRL_REG4_ADDR, 1);

    /* Let L3GD20 perform changes */
    sleep(CHANGE_DELAY);

    /* Discard values recorded during scale change */
    if(fifoMode)
    	clearFifo();

    if (_scaleBuffer.back().second > 0)
        _scaleBuffer.push_back(std::make_pair(scale, 0));
    else
        _scaleBuffer.back().first = scale;
}

void Gyroscope::useHighPassFilter(bool use)
{
    L3GD20_FilterCmd(use ? L3GD20_HIGHPASSFILTER_ENABLE : L3GD20_HIGHPASSFILTER_DISABLE);
    
    /* Let L3GD20 perform changes */
    sleep(CHANGE_DELAY);
}


// TODO: Bypass to stream support
// Stream to FIFO support
void Gyroscope::retrieveValues()
{
    uint8_t tmpbuffer[6] = {0};
    math3d::Vector3<int16_t> vec;
    uint8_t ctrl4, fifoCtrl, fifoSrc;
    int i = 0;

    L3GD20_Read(&fifoCtrl, L3GD20_FIFO_CTRL_REG_ADDR, 1);
    bool fifoMode = (fifoCtrl & 0x70) != 0;
    bool fifoFull = false;    
    
    if (fifoMode){
        L3GD20_Read(&fifoSrc, L3GD20_FIFO_SRC_REG_ADDR, 1);
        
        /* Test FIFO empty bit */
        if ((fifoSrc & 0x20) != 0)
            return; 
    }
    
    if(_scaleBuffer.empty())
        return;

    L3GD20_Read(&ctrl4, L3GD20_CTRL_REG4_ADDR, 1);    
    do{
    	// FIFO overrun test
    	if (fifoMode){
    		L3GD20_Read(&fifoSrc, L3GD20_FIFO_SRC_REG_ADDR, 1);
    		fifoFull = fifoFull || (fifoSrc & 0x40) != 0;
    	}

        L3GD20_Read(tmpbuffer, L3GD20_OUT_X_L_ADDR, 6);

        /* Check in the control register 4 the data alignment (Big Endian or Little Endian) */
        if(ctrl4 & 0x40){
            for(i = 0; i < 3; i++){
              vec[i] = (int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
            }
        }
        else{
            for(i = 0; i < 3; i++){
              vec[i] = (int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
            }
        }
        
        // Check if buffer size isn't larger than maximum
        if(_maxBufferSize > 0 && _dataBuffer.size() >= _maxBufferSize)
        	discard();

        /* Place retrieved values in local buffer */
        _dataBuffer.push_back(vec);
        _scaleBuffer.back().second++;
        
        // FIFO empty test
        if (fifoMode){
        	// Let FIFO update
        	sleep(10, microsecond);
			L3GD20_Read(&fifoSrc, L3GD20_FIFO_SRC_REG_ADDR, 1);
        }

    }while (fifoMode && (fifoSrc & 0x20) == 0);
    
    /* FIFO needs reset after being full */
    if (fifoMode && fifoFull)
        resetFifo();
}

void Gyroscope::clearFifo()
{
	uint8_t tmpbuffer[6] = {0};
	uint8_t fifoSrc;
	bool fifoFull = false;

	do{
		// FIFO overrun test
		L3GD20_Read(&fifoSrc, L3GD20_FIFO_SRC_REG_ADDR, 1);
		fifoFull = fifoFull || (fifoSrc & 0x40) != 0;

		L3GD20_Read(tmpbuffer, L3GD20_OUT_X_L_ADDR, 6);

		// FIFO empty test
		L3GD20_Read(&fifoSrc, L3GD20_FIFO_SRC_REG_ADDR, 1);

	}while ((fifoSrc & 0x20) == 0);

	/* FIFO needs reset after being full */
	if (fifoFull)
		resetFifo();
	else
		sleep(CHANGE_DELAY);
}

void Gyroscope::resetFifo()
{
	uint8_t fifoCtrl;
	L3GD20_Read(&fifoCtrl, L3GD20_FIFO_CTRL_REG_ADDR, 1);

	// Set to bypass mode to restart data collection
	fifoCtrl = (fifoCtrl & (~0xE0));
	L3GD20_Write(&fifoCtrl, L3GD20_FIFO_CTRL_REG_ADDR, 1);
	sleep(CHANGE_DELAY);

	// Change back to FIFO mode
	fifoCtrl = fifoCtrl | FIFO_MODE;
	L3GD20_Write(&fifoCtrl, L3GD20_FIFO_CTRL_REG_ADDR, 1);
	sleep(CHANGE_DELAY);
}

// TODO: interupt if FIFO full or Bypass mode value changes
