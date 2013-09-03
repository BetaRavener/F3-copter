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

#include "main.h"
#include "common.h"
#include "gyroscope.h"
#include "accelerometer.h"
#include "math3d.h"
#include "complementaryFilter2.h"
#include "controller.h"
#include "stopwatch.h"
#include "model.h"
#include "systime.h"
#include "communicator.h"
#include "rc_receiver.h"
#include "periphery.h"
#include "interrupt.h"

#include <cmath>
#include <cstdio>

#include <stm32f30x.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_tim.h>

static const float sensorUpdateTime = 0.01;
static const float filterTimeConst = 0.49;

static float pitchProportional = 0.3f;
static float pitchIntegral = 0.01f;
static float pitchDerivative = 0.0f;
static float rollProportional = 0.3f;
static float rollIntegral = 0.01f;
static float rollDerivative = 0.0f;
static float yawProportional = 0.3f;
static float yawIntegral = 0.01f;
static float yawDerivative = 0.0f;

// Maximum angles in which tricopter may fly
// Approx. 40 degrees
static float maxPitchAngle = 0.7f;
static float maxRollAngle = 0.7f;
static float maxYawAngularSpeed = math3d::Pi;

enum class CommandIds{pitchProportional = 1, pitchIntegral, pitchDerivative,
				      rollProportional, rollIntegral, rollDerivative};

enum ProgramState {ProgramRunning, ProgramEnded, StateN}; 
ProgramState programState;

//#define PWM_TEST
//#define ANGLE_TEST
#define CTRL_TEST

// Input angles from range <0, 2*Pi)
// Output from range <-Pi, Pi>
float interpolateAngle(float start, float end)
{
	float dif = end - start;
	return dif >= 0 ? dif <= math3d::Pi ? dif : dif - 2 * math3d::Pi
			        : dif >= -math3d::Pi ? dif : 2 * math3d::Pi + dif;
}

// Normalizes angle into <0, 2*Pi) interval
float normalizeAngle(float angle)
{
	// Remove extra rounds
	angle = fmod(angle, 2 * math3d::Pi);

	// Return positive angle
	return angle >= 0 ? angle : 2 * math3d::Pi + angle;
}

int main(void)
{
    {
        RCC_ClocksTypeDef RCC_Clocks;
        RCC_GetClocksFreq(&RCC_Clocks);
        SysTick_Config(RCC_Clocks.HCLK_Frequency / SYSTEM_TIME_RESOLUTION);
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
        NVIC_SetPriority(SysTick_IRQn, 0);
    }

    // Restart system time
    restartSystemTime();

    // Initialize User Button available on STM32F3-Discovery board
    STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI); 

    // Sensors Test
    L3GD20_InitTypeDef gyroInit;
    L3GD20_FilterConfigTypeDef gyroFilterConfig;
    LSM303DLHCAcc_InitTypeDef accInit;
    LSM303DLHCAcc_FilterConfigTypeDef accFilterConfig;

    gyroInit.Power_Mode         = L3GD20_MODE_ACTIVE;
    gyroInit.Output_DataRate    = L3GD20_OUTPUT_DATARATE_4;
    gyroInit.Axes_Enable        = L3GD20_AXES_ENABLE;
    gyroInit.Band_Width         = L3GD20_BANDWIDTH_4;
    gyroInit.BlockData_Update   = L3GD20_BlockDataUpdate_Continous;
    gyroInit.Endianness         = L3GD20_BLE_LSB;
    gyroInit.Full_Scale         = L3GD20_FULLSCALE_500; 

    gyroFilterConfig.HighPassFilter_Mode_Selection      = L3GD20_HPM_NORMAL_MODE_RES;
    gyroFilterConfig.HighPassFilter_CutOff_Frequency    = L3GD20_HPFCF_0;
    
    accInit.Power_Mode 			= LSM303DLHC_NORMAL_MODE;
    accInit.AccOutput_DataRate 	= LSM303DLHC_ODR_50_HZ;
    accInit.Axes_Enable 		= LSM303DLHC_AXES_ENABLE;
    accInit.AccFull_Scale 		= LSM303DLHC_FULLSCALE_2G;
    accInit.BlockData_Update 	= LSM303DLHC_BlockUpdate_Continous;
    accInit.Endianness 			= LSM303DLHC_BLE_LSB;
    accInit.High_Resolution 	= LSM303DLHC_HR_ENABLE;

	// Fill the accelerometer LPF structure
    accFilterConfig.HighPassFilter_Mode_Selection 	= LSM303DLHC_HPM_NORMAL_MODE;
    accFilterConfig.HighPassFilter_CutOff_Frequency	= LSM303DLHC_HPFCF_16;
    accFilterConfig.HighPassFilter_AOI1 			= LSM303DLHC_HPF_AOI1_DISABLE;
    accFilterConfig.HighPassFilter_AOI2 			= LSM303DLHC_HPF_AOI2_DISABLE;

    Gyroscope gyro(gyroInit, gyroFilterConfig, 0);
    Accelerometer acc(accInit, accFilterConfig, 0);
    ComplementaryFilter2 cmplFilter(sensorUpdateTime, filterTimeConst);

    // --- PID CONTROLLER SETUP ---
    Controller pitchController(pitchProportional, pitchIntegral, pitchDerivative, sensorUpdateTime);
    pitchController.limitOutput(true, -1, 1);
    Controller rollController(rollProportional, rollIntegral, rollDerivative, sensorUpdateTime);
    rollController.limitOutput(true, -1, 1);
    Controller yawController(yawProportional, yawIntegral, yawDerivative, sensorUpdateTime);
    yawController.limitOutput(true, -1, 1);

    math3d::Vector3<float> accReading, accAngle, gyroAngle, gyroAngleOut, angle, controllerOuttput;

#ifdef PWM_TEST
    // Enable interface clock on timer 1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Configure timer 1 for PWM output
	Pwm::configureTimer(TIM1, 50);

	// Enable interface clock on IO port E
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);

	// Connect PWM outputs to pins
	Pwm a(TIM1, 1), b(TIM1, 2), c(TIM1, 3);
	a.connect(GPIOE, 9, 2);
	b.connect(GPIOE, 11, 2);
	c.connect(GPIOE, 13, 2);

	bool up = true;
	float dc = 1e-3;
#else
	// --- MODEL SETUP ---
	Periphery::enable(Periphery::TIM1_P);
	Periphery::enable(Periphery::GPIOE_P);
	Pwm::configureTimer(TIM1, 50);

	Model model;
#endif

    // --- COMMUNICATION SETUP ---
	Periphery::enable(Periphery::USART1_P);
	Periphery::enable(Periphery::GPIOA_P);
	Communicator comm(Communicator::UartSource);

	// --- RC RECEIVER SETUP ---
	RcReceiver rc;
	Periphery::enable(Periphery::TIM3_P);
	Periphery::enable(Periphery::GPIOC_P);
	rc.configureTimer(TIM3);
	rc.addChannel(0, TIM3, 1, GPIOC, 6, 2);
	rc.addChannel(1, TIM3, 2, GPIOC, 7, 2);
	rc.addChannel(2, TIM3, 3, GPIOC, 8, 2);
	rc.addChannel(3, TIM3, 4, GPIOC, 9, 2);
	Interrupt::enable(TIM3_IRQn, 3, 1);

	// --- LOOP TIME CONTROL ---
	Stopwatch watch;
	uint64_t elapsed;
	uint64_t spareTime = 0;
	uint64_t totalSpareTime = 0;
	int iter = 0;

#ifdef ANGLE_TEST
	accReading = acc.readValue();
	accAngle = math3d::Vector3<float>(std::atan2(-accReading[0], std::sqrt(accReading[1] * accReading[1] + accReading[2] * accReading[2])),
									  -std::atan2(accReading[1], std::sqrt(accReading[0] * accReading[0] + accReading[2] * accReading[2])),
									  angle[2]);
	gyroAngleOut = accAngle;
#endif

    programState = ProgramRunning;
    while(programState == ProgramRunning){
        watch.restart();

        // Process incoming communication
        Communicator::CommandId commandId;
        while(!comm.empty())
        {
        	if(comm.receive(commandId)){
        		switch((CommandIds)commandId){
        		case CommandIds::pitchProportional:{
        			float pp;
        			if(comm.receive(pp))
        				pitchProportional = pp;
        			break;}

        		case CommandIds::pitchIntegral:{
        			float pi;
					if(comm.receive(pi))
						pitchIntegral = pi;
        			break;}

        		case CommandIds::pitchDerivative:{
        			float pd;
					if(comm.receive(pd))
						pitchDerivative = pd;
        			break;}

        		case CommandIds::rollProportional:{
					float rp;
					if(comm.receive(rp))
						rollProportional = rp;
					break;}

				case CommandIds::rollIntegral:{
					float ri;
					if(comm.receive(ri))
						rollIntegral = ri;
					break;}

				case CommandIds::rollDerivative:{
					float rd;
					if(comm.receive(rd))
						rollDerivative = rd;
					break;}
				}
        	}
        	else
        		comm.discard();
        }

        // Integrate gyroscope output
        gyroAngle = gyro.readValue() * sensorUpdateTime;

        // TODO: ak by mala trikoptera naklon viac ako +-90 stupnov v roll a pitch, treba riesit
        // aliasing, prevadzat uhly do intervalu <0, 2*PI) a nejak osetrit gimbal lock. V tom pripade
        // by bolo mozno vyhodnejsie pouzit quaterniony a prevadzat uhly priamo do nich
        // TODO: prerobit triedy na uchovavanie stavu - zrychlenie
        accReading = acc.readValue();

        // Transform accelerometer reading into board space and calculate angles
        // Pitch (X rot), Roll (Y rot), Yaw (Z rot)
		accAngle = math3d::Vector3<float>(std::atan2(-accReading[0], std::sqrt(accReading[1] * accReading[1] + accReading[2] * accReading[2])),
										  -std::atan2(accReading[1], std::sqrt(accReading[0] * accReading[0] + accReading[2] * accReading[2])),
										  angle[2]);

        // Combine angles from two sensors
		// TODO: ak je velkost vektora accReading mimo rozumnych hodnot (okolo 1g), pouzi iba udaje z gyra?
		angle = cmplFilter.addSample(accAngle, gyroAngle);
		// Normalize Yaw angle
		// TODO: should be normalized inside the filter too
		angle[2] = normalizeAngle(angle[2]);

		// --- Proven to be working to this place ---

		// Get RC input
		float rcPitch = 0;//(rc.normalizedReading(0) - 0.5) * 2 * maxPitchAngle;
		float rcRoll = 0;//(rc.normalizedReading(1) - 0.5) * 2 * maxRollAngle;
		float rcThrottle = rc.normalizedReading(2);
		float rcYaw = 0;//(rc.normalizedReading(3) - 0.5) * 2 * maxYawAngularSpeed;

		// Received rcYaw is representing angular speed so it needs to be integrated
		float yawAngle = normalizeAngle(yawController.setpoint() + rcYaw * sensorUpdateTime);

		// Update setpoints for controllers based on RC input
		pitchController.setpoint(rcPitch);
		rollController.setpoint(rcRoll);
		yawController.setpoint(yawAngle);

		controllerOuttput = math3d::Vector3<float>(pitchController.process(angle[0]),
										   	   	   rollController.process(angle[1]),
										   	   	   yawController.process(angle[2], interpolateAngle));

		// Update model
		model.update(rcThrottle, controllerOuttput);

#ifdef PWM_TEST
		a.dutyCycle(dc);
		b.dutyCycle(1-dc);
		c.dutyCycle(dc);
		up ? dc *= 1.02  : dc *= 0.98;
		if (dc >= 1) up = false;
		if (dc <= 1e-3) up = true;
#endif

#ifdef ANGLE_TEST
		gyroAngleOut += gyroAngle;
		char buf[200];
		std::sprintf(buf, "%f,%f,%f\r\n", math3d::Degrees(gyroAngleOut[1]),
										  math3d::Degrees(accAngle[1]),
										  math3d::Degrees(angle[1]));
		comm.sendRaw(std::string(buf));
#endif

#ifdef CTRL_TEST
		char buf[200];
		std::sprintf(buf, "%f,%f,%f\r\n", math3d::Degrees(angle[2]),
										  math3d::Degrees(yawAngle),
										  controllerOuttput[2]);
		comm.sendRaw(std::string(buf));
#endif

		elapsed = watch.elapsed(microsecond);
		if (elapsed < sensorUpdateTime * microsecond){
			spareTime = sensorUpdateTime * microsecond - elapsed;
			sleep(spareTime, microsecond);

			iter++;
			totalSpareTime += spareTime;
		}
    }
    
    return 0;
}

void buttonPressed(void)
{
    switch(programState){
    default:
        programState = ProgramEnded;
    }
}

uint32_t L3GD20_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
  while (1)
  {   
  }
  return 0;
}

uint32_t LSM303DLHC_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
  while (1)
  {   
  }
  return 0;
}
