/**
 * @file	DRV8813.cpp
 * @author	Simon ROUTHIAU
 * @date	03 avr. 2017
 * @brief	DRV8813 Abstration Class
 */

#include <stddef.h>
#include "Timer.hpp"
#include "GPIO.hpp"
#include "common.h"

using namespace HAL;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/
#define STEPPER_FREQ_PWM	(1000000u)			//1MHz
#define DC_FREQ_PWM			(10000u)			//10kHz
#define FREQ_TICK_REF		(4000u)				//4kHz
#define TIMER_TICK_REF		Timer::TIMER7		//Warning: common to Servo Driver

#define USTEP_1		0
#define USTEP_2		1
#define USTEP_4		2
#define USTEP_8		3
#define USTEP_16	4

#define	GPIO_DECAY	GPIO::GPIO6
#define	GPIO_RESET	GPIO::GPIO7
#define	GPIO_SLEEP	GPIO::GPIO8

//Drv88113 1
#define	DRV1_GPIO_FAULT	GPIO::GPIO9
#define	DRV1_GPIO_PHA	GPIO::GPIO10
#define	DRV1_GPIO_PHB	GPIO::GPIO11
#define	DRV1_GPIO_ENA	PWM::PWMx
#define	DRV1_GPIO_ENB	PWM::PWMx

//Drv88113 2
#define	DRV2_GPIO_FAULT	GPIO::GPIO12
#define	DRV2_GPIO_PHA	GPIO::GPIO13
#define	DRV2_GPIO_PHB	GPIO::GPIO14
#define	DRV2_GPIO_ENA	PWM::PWMx
#define	DRV2_GPIO_ENB	PWM::PWMx

//Drv88113 3
#define	DRV3_GPIO_FAULT	GPIO::GPIO15
#define	DRV3_GPIO_PHA	GPIO::GPIO16
#define	DRV3_GPIO_PHB	GPIO::GPIO17
#define	DRV3_GPIO_ENA	PWM::PWMx
#define	DRV3_GPIO_ENB	PWM::PWMx

//Drv88113 4
#define	DRV4_GPIO_FAULT	GPIO::GPIO18
#define	DRV4_GPIO_PHA	GPIO::GPIO19
#define	DRV4_GPIO_PHB	GPIO::GPIO20
#define	DRV4_GPIO_ENA	PWM::PWMx
#define	DRV4_GPIO_ENB	PWM::PWMx

//Drv88113 5
#define	DRV5_GPIO_FAULT	GPIO::GPIO21
#define	DRV5_GPIO_PHA	GPIO::GPIO22
#define	DRV5_GPIO_PHB	GPIO::GPIO23
#define	DRV5_GPIO_ENA	PWM::PWMx
#define	DRV5_GPIO_ENB	PWM::PWMx

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

/**
 * @brief Timer instances
 */
Drv8813* _drv8813[Drv8813::DRV8813_MAX] = {NULL};



/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @brief Retrieve Drv8813 definitions from Timer ID
 * @param id : Drv8813 ID
 * @return DRV8813_DEF structure
 */
static DRV8813_DEF _getDrv8813Struct (enum Timer::ID id)
{
	DRV8813_DEF drv;

	assert(id < HAL::Drv8813::DRV8813_MAX);

	switch(id)
	{
	case HAL::Drv8813::DRV8813_1:
		drv.DRV8813.MODE				=	STEPPER_MODE;
		drv.USTEP_MODE					= 	USTEP_1;
		drv.DRV8813.PWM_FREQ			=	STEPPER_FREQ;
		drv.DRV8813.GPIO_DECAY			=	GPIO_DECAY;
		drv.DRV8813.GPIO_RESET			=	GPIO_RESET;
		drv.DRV8813.GPIO_SLEEP			=	GPIO_SLEEP;
		drv.DRV8813.GPIO_FAULT			=	DRV1_GPIO_FAULT;
		drv.DRV8813.GPIO_PHA			=	DRV1_GPIO_PHA;
		drv.DRV8813.GPIO_PHB			=	DRV1_GPIO_PHB;
		drv.DRV8813.GPIO_ENA			=	DRV1_GPIO_ENA;
		drv.DRV8813.GPIO_ENB			=	DRV1_GPIO_ENB;
		break;
	case HAL::Drv8813::DRV8813_2:
		drv.DRV8813.MODE				=	STEPPER_MODE;
		drv.USTEP_MODE					= 	USTEP_1;
		drv.DRV8813.PWM_FREQ			=	STEPPER_FREQ;
		drv.DRV8813.GPIO_DECAY			=	GPIO_DECAY;
		drv.DRV8813.GPIO_RESET			=	GPIO_RESET;
		drv.DRV8813.GPIO_SLEEP			=	GPIO_SLEEP;
		drv.DRV8813.GPIO_FAULT			=	DRV2_GPIO_FAULT;
		drv.DRV8813.GPIO_PHA			=	DRV2_GPIO_PHA;
		drv.DRV8813.GPIO_PHB			=	DRV2_GPIO_PHB;
		drv.DRV8813.GPIO_ENA			=	DRV2_GPIO_ENA;
		drv.DRV8813.GPIO_ENB			=	DRV2_GPIO_ENB;
		break;
	case HAL::Drv8813::DRV8813_3:
		drv.DRV8813.MODE				=	STEPPER_MODE;
		drv.USTEP_MODE					= 	USTEP_1;
		drv.DRV8813.PWM_FREQ			=	STEPPER_FREQ;
		drv.DRV8813.GPIO_DECAY			=	GPIO_DECAY;
		drv.DRV8813.GPIO_RESET			=	GPIO_RESET;
		drv.DRV8813.GPIO_SLEEP			=	GPIO_SLEEP;
		drv.DRV8813.GPIO_FAULT			=	DRV3_GPIO_FAULT;
		drv.DRV8813.GPIO_PHA			=	DRV3_GPIO_PHA;
		drv.DRV8813.GPIO_PHB			=	DRV3_GPIO_PHB;
		drv.DRV8813.GPIO_ENA			=	DRV3_GPIO_ENA;
		drv.DRV8813.GPIO_ENB			=	DRV3_GPIO_ENB;
		break;
	case HAL::Drv8813::DRV8813_4:
		drv.DRV8813.MODE				=	STEPPER_MODE;
		drv.USTEP_MODE					= 	USTEP_1;
		drv.DRV8813.PWM_FREQ			=	STEPPER_FREQ;
		drv.DRV8813.GPIO_DECAY			=	GPIO_DECAY;
		drv.DRV8813.GPIO_RESET			=	GPIO_RESET;
		drv.DRV8813.GPIO_SLEEP			=	GPIO_SLEEP;
		drv.DRV8813.GPIO_FAULT			=	DRV4_GPIO_FAULT;
		drv.DRV8813.GPIO_PHA			=	DRV4_GPIO_PHA;
		drv.DRV8813.GPIO_PHB			=	DRV4_GPIO_PHB;
		drv.DRV8813.GPIO_ENA			=	DRV4_GPIO_ENA;
		drv.DRV8813.GPIO_ENB			=	DRV4_GPIO_ENB;
		break;
	case HAL::Drv8813::DRV8813_5:
		drv.DRV8813.MODE				=	STEPPER_MODE;
		drv.USTEP_MODE					= 	USTEP_1;
		drv.DRV8813.PWM_FREQ			=	STEPPER_FREQ;
		drv.DRV8813.GPIO_DECAY			=	GPIO_DECAY;
		drv.DRV8813.GPIO_RESET			=	GPIO_RESET;
		drv.DRV8813.GPIO_SLEEP			=	GPIO_SLEEP;
		drv.DRV8813.GPIO_FAULT			=	DRV5_GPIO_FAULT;
		drv.DRV8813.GPIO_PHA			=	DRV5_GPIO_PHA;
		drv.DRV8813.GPIO_PHB			=	DRV5_GPIO_PHB;
		drv.DRV8813.GPIO_ENA			=	DRV5_GPIO_ENA;
		drv.DRV8813.GPIO_ENB			=	DRV5_GPIO_ENB;
		break;
	default:
		break;
	}

	return drv;
}

/**
 * @brief Initialize peripheral for a specified Drv8813
 * @param id : Drv8813 ID
 */
static void _hardwareInit (enum Drv8813::ID id)
{
	static bool TimerInitialized = false;

	assert(id < HAL::Drv8813::DRV8813_MAX);

	DRV8813_DEF drv = _getDrv8813Struct(id);

	GpioInst->DECAY 				= GPIO::GetInstance(drv.DRV8813.GPIO_DECAY);
	GpioInst->RESET 				= GPIO::GetInstance(drv.DRV8813.GPIO_RESET);
	GpioInst->SLEEP 				= GPIO::GetInstance(drv.DRV8813.GPIO_SLEEP);
	GpioInst->FAULT 				= GPIO::GetInstance(drv.DRV8813.DRV5_GPIO_FAULT);
	GpioInst->PHA	 				= GPIO::GetInstance(drv.DRV8813.DRV5_GPIO_PHA);
	GpioInst->PHB	 				= GPIO::GetInstance(drv.DRV8813.DRV5_GPIO_PHB);
	GpioInst->ENA	 				= PWM::GetInstance(drv.DRV8813.DRV5_GPIO_ENA);
	GpioInst->ENB	 				= PWM::GetInstance(drv.DRV8813.DRV5_GPIO_ENB);

	if(TimerInitialized==false)
	{
		Drv8813->tim = Timer::GetInstance (TIMER_TICK_REF);
		Drv8813->tim->SetPeriod(1000000u/FREQ_TICK_REF)
		Drv8813->tim->TimerElapsed += TickDrv8813Event;
		Drv8813->tim->Start();
	}
}

/**
 * @brief Tick event for step generation
 */
static void TickDrv8813Event (void * obj)
{
	static uint32
	for (char i =0; i<DRV8813_MAX; i++)
	{
		Drv8813* drv = _drv8813[i];
		if(_drv8813[i] != null && direction!=DISABLED)
		{


			if(nb_pulse!=0 or run)
			{

			}
		}
	}
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/
namespace HAL
{
	Drv8813* Drv8813::GetInstance (enum Drv8813::ID id)
	{
		assert(id < Drv8813::DRV8813_MAX);

		// if Driver instance already exists
		if(_drv8813[id] != NULL)
		{
			return _drv8813[id];
		}
		else
		{
			// Create Driver instance
			_drv8813[id] = new Drv8813(id);

			return _drv8813[id];
		}
	}

	Drv8813::Drv8813 (enum Drv8813::ID id)
	{
		this->id = id;
		this->def = _getDrv8813Struct(id);
		this->enable = true;

		_hardwareInit(id);
	}
}

