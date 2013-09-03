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

#include "accelerometer.h"
#include "systime.h"
#include "stopwatch.h"

// CTRL_REG1 related //
// Output data rate in Hz
#define LSM_ACC_ODR_1				1
#define LSM_ACC_ODR_10				10
#define LSM_ACC_ODR_25				25
#define LSM_ACC_ODR_50				50
#define LSM_ACC_ODR_100				100
#define LSM_ACC_ODR_200				200
#define LSM_ACC_ODR_400				400
#define LSM_ACC_ODR_1344			1344

// CTRL_REG4 related //
// Sensitivity
#define LSM_Acc_Sensitivity_2g      1.0         /*!< accelerometer sensitivity is 1 mg/LSB with 2 g full scale */
#define LSM_Acc_Sensitivity_4g      0.5         /*!< accelerometer sensitivity is 2 mg/LSB with 4 g full scale */
#define LSM_Acc_Sensitivity_8g      0.25        /*!< accelerometer sensitivity is 4 mg/LSB with 8 g full scale */
#define LSM_Acc_Sensitivity_16g     0.08333333  /*!< accelerometer sensitivity is 12 mg/LSB with 16 g full scale */

#define SCALE_BITS					0x30

// CTRL_REG5 related //
#define FIFO_ENABLED				0x40
#define FIFO_DISABLED				0x00

// FIFO_CTRL_REG related //
#define MODE_BITS					0xC0
#define BYPASS_MODE					0x00
#define FIFO_MODE					0x40
#define STREAM_MODE					0x80
#define IS_FIFO						0xC0

// FIFO_SRC_REG related //
#define FIFO_EMPTY					0x20
#define FIFO_OVERRUN				0x40

// Others //
#define CHANGE_DELAY                5

Accelerometer::Accelerometer(LSM303DLHCAcc_InitTypeDef& accInit, LSM303DLHCAcc_FilterConfigTypeDef& filterConfig, uint16_t maxBufferSize) :
_maxBufferSize(maxBufferSize)
{
	switch(accInit.AccOutput_DataRate)
	{
	case LSM303DLHC_ODR_1_HZ:
		_deltaT = 1.0 / LSM_ACC_ODR_1;
		break;
	case LSM303DLHC_ODR_10_HZ:
		_deltaT = 1.0 / LSM_ACC_ODR_10;
		break;
	case LSM303DLHC_ODR_25_HZ:
		_deltaT = 1.0 / LSM_ACC_ODR_25;
		break;
	case LSM303DLHC_ODR_50_HZ:
		_deltaT = 1.0 / LSM_ACC_ODR_50;
		break;
	case LSM303DLHC_ODR_100_HZ:
		_deltaT = 1.0 / LSM_ACC_ODR_100;
		break;
	case LSM303DLHC_ODR_200_HZ:
		_deltaT = 1.0 / LSM_ACC_ODR_200;
		break;
	case LSM303DLHC_ODR_400_HZ:
		_deltaT = 1.0 / LSM_ACC_ODR_400;
		break;
	case LSM303DLHC_ODR_1344_HZ:
		_deltaT = 1.0 / LSM_ACC_ODR_1344;
		break;
	default:
		return;
	}

    /* Configure Mems LSM303DLHC Accelerometer */
	LSM303DLHC_AccInit(&accInit);
    LSM303DLHC_AccFilterConfig(&filterConfig);
    selectMode(BypassMode);
    // TODO: filter defaultne vypnuty, po zapnuti nepodava spravne hodnoty (preco? Lebo high pass filtruje dlhodobu gravitaciu?)
    // useHighPassFilter(true);
    // uint8_t aaa;
    // LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &aaa, 1);

    /* Initialize scale buffer */
    _scaleBuffer.push_back(std::make_pair(accInit.AccFull_Scale, 0));
}

int Accelerometer::test()
{
	Stopwatch sw;
	selectMode(Accelerometer::FifoMode);
	discard(true);
	clearFifo();
	sw.start();
	while(sw.elapsed() < 1000)
	{
		retrieveValues();
	}

	// TODO: Data rate je 2x vyssi nez by mal byt
	// TODO: 400 a 1344 Hz nefunguje spravne
	return _dataBuffer.size();
}

math3d::Vector3<float> Accelerometer::readValue()
{
    /* First retrieve new values from LSM303DLHC */
    retrieveValues();

    if (_dataBuffer.size() <= 0)
        return math3d::ZeroVector;

    float sensitivity;

    /* Switch the sensitivity value */
    switch(_scaleBuffer.front().first & SCALE_BITS)
    {
    case LSM303DLHC_FULLSCALE_2G:
        sensitivity = LSM_Acc_Sensitivity_2g;
        break;

    case LSM303DLHC_FULLSCALE_4G:
        sensitivity = LSM_Acc_Sensitivity_4g;
        break;

    case LSM303DLHC_FULLSCALE_8G:
        sensitivity = LSM_Acc_Sensitivity_8g;
        break;

    case LSM303DLHC_FULLSCALE_16G:
		sensitivity = LSM_Acc_Sensitivity_16g;
		break;

    default:
        sensitivity = 0;
        break;
    }

    // Divide by sensitivity and scale from miliG to G
    math3d::Vector3<float> ret = math3d::Vector3<float>(_dataBuffer.front()) / (sensitivity * 1000);

    discard();
    return ret;
}

void Accelerometer::discard(bool all)
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

void Accelerometer::selectMode(Mode mode)
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

    LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG5_A, &ctrl5, 1);
    ctrl5 = (ctrl5 & (~FIFO_ENABLED)) | fifoEn;
    LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG5_A, &ctrl5);

    LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_FIFO_CTRL_REG_A, &fifoCtrl, 1);
    fifoCtrl = (fifoCtrl & (~MODE_BITS)) | fifoMode;
    LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_FIFO_CTRL_REG_A, &fifoCtrl);

    // Clear FIFO from previously stored data
    if(fifoEn == FIFO_ENABLED)
    	clearFifo();
}

void Accelerometer::changeScale(uint8_t scale)
{
    uint8_t ctrl4, fifoCtrl;

    scale &= SCALE_BITS;

    LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_FIFO_CTRL_REG_A, &fifoCtrl, 1);
    bool fifoMode = (fifoCtrl & IS_FIFO) != 0;

    /* Retrieve stored values before changing scale */
    retrieveValues();

    /* Read current value from CTRL_REG4 register */
    LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, &ctrl4, 1);

    /* Change scale */
    ctrl4 = (ctrl4 & ~SCALE_BITS) | scale;

    /* Write new value to CTRL_REG4 regsister */
    LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, &ctrl4);

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

void Accelerometer::useHighPassFilter(bool use)
{
	LSM303DLHC_AccFilterCmd(use ? LSM303DLHC_HIGHPASSFILTER_ENABLE : LSM303DLHC_HIGHPASSFILTER_DISABLE);

    /* Let L3GD20 perform changes */
    sleep(CHANGE_DELAY);
}

void Accelerometer::retrieveValues()
{
	const uint16_t shift = 16;
    uint8_t tmpBuffer[6] = {0};
    math3d::Vector3<int16_t> vec;
    uint8_t ctrl4, fifoCtrl, fifoSrc;
    int i = 0;

    LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_FIFO_CTRL_REG_A, &fifoCtrl, 1);
    bool fifoMode = (fifoCtrl & IS_FIFO) != 0;
    bool fifoFull = false;

    if (fifoMode){
    	LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_FIFO_SRC_REG_A, &fifoSrc, 1);

        /* Test FIFO empty bit */
        if ((fifoSrc & FIFO_EMPTY) != 0)
            return;
    }

    if(_scaleBuffer.empty())
        return;

    LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, &ctrl4, 1);
    do{
    	// FIFO overrun test
    	if (fifoMode){
    		LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_FIFO_SRC_REG_A, &fifoSrc, 1);
    		fifoFull = fifoFull || (fifoSrc & FIFO_OVERRUN) != 0;
    	}

    	LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, tmpBuffer, 6);

        /* Check in the control register 4 the data alignment (Big Endian or Little Endian) */
        if(ctrl4 & LSM303DLHC_BLE_MSB){
            for(i = 0; i < 3; i++){
              vec[i] = (int16_t)(((uint16_t)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]);
            }
        }
        else{
            for(i = 0; i < 3; i++){
              vec[i] = (int16_t)(((uint16_t)tmpBuffer[2*i+1] << 8) + tmpBuffer[2*i]);
            }
        }

        // Check if buffer size isn't larger than maximum
        if(_maxBufferSize > 0 && _dataBuffer.size() >= _maxBufferSize)
        	discard();

        /* Place retrieved values in local buffer */
        _dataBuffer.push_back(vec / shift);
        _scaleBuffer.back().second++;

        // FIFO empty test
        if (fifoMode){
        	// Let FIFO update
        	sleep(10, microsecond);
        	LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_FIFO_SRC_REG_A, &fifoSrc, 1);
        }

    }while (fifoMode && (fifoSrc & FIFO_EMPTY) == 0);

    /* FIFO needs reset after being full */
    if (fifoMode && fifoFull)
        resetFifo();
}

void Accelerometer::clearFifo()
{
	uint8_t tmpBuffer[6] = {0};
	uint8_t fifoSrc;
	bool fifoFull = false;

	do{
		// FIFO overrun test
		LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_FIFO_SRC_REG_A, &fifoSrc, 1);
		fifoFull = fifoFull || (fifoSrc & FIFO_OVERRUN) != 0;

		LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, tmpBuffer, 6);

		// FIFO empty test
		LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_FIFO_SRC_REG_A, &fifoSrc, 1);
	}while ((fifoSrc & FIFO_EMPTY) == 0);

	/* FIFO needs reset after being full */
	if (fifoFull)
		resetFifo();
	else
		sleep(CHANGE_DELAY);
}

void Accelerometer::resetFifo()
{
	uint8_t fifoCtrl;
	LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_FIFO_CTRL_REG_A, &fifoCtrl, 1);

	// Set to bypass mode to restart data collection
	fifoCtrl = (fifoCtrl & (~MODE_BITS));
	LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_FIFO_CTRL_REG_A, &fifoCtrl);
	sleep(CHANGE_DELAY);

	// Change back to FIFO mode
	fifoCtrl = fifoCtrl | FIFO_MODE;
	LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_FIFO_CTRL_REG_A, &fifoCtrl);
	sleep(CHANGE_DELAY);
}

// TODO: interupt if FIFO full or Bypass mode value changes
