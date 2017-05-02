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
#define STEPPER_FREQ_PWM	(16000u)			//16kHz
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
/* Const				                                                       */
/*----------------------------------------------------------------------------*/
const PWM_STEP_DEF		STEP_DEF_16[64] = {{true,true,100,0},{true,true,100,10},{true,true,98,20},{true,true,96,29},{true,true,92,38},{true,true,88,47},{true,true,83,56},{true,true,77,63},
											{true,true,71,71},{true,true,63,77},{true,true,56,83},{true,true,47,88},{true,true,38,92},{true,true,29,96},{true,true,20,98},{true,true,10,100},
											{true,true,0,100},{false,true,10,100},{false,true,20,98},{false,true,29,96},{false,true,38,92},{false,true,47,88},{false,true,56,83},{false,true,63,77},
											{false,true,71,71},{false,true,77,63},{false,true,83,56},{false,true,88,47},{false,true,92,38},{false,true,96,29},{false,true,98,20},{false,true,100,10},
											{false,true,100,0},{false,false,100,10},{false,false,98,20},{false,false,96,29},{false,false,92,38},{false,false,88,47},{false,false,83,56},{false,false,77,63},
											{false,false,71,71},{false,false,63,77},{false,false,56,83},{false,false,47,88},{false,false,38,92},{false,false,29,96},{false,false,20,98},{false,false,10,100},
											{true,false,0,100},{true,false,10,100},{true,false,20,98},{true,false,29,96},{true,false,38,92},{true,false,47,88},{true,false,56,83},{true,false,63,77},
											{true,false,71,71},{true,false,77,63},{true,false,83,56},{true,false,88,47},{true,false,92,38},{true,false,96,29},{true,false,98,20}{true,false,100,10}};

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
		drv.DRV8813.MODE		=	STEPPER_MODE.STEPPER_MODE;
		drv.USTEP_MODE			= 	USTEP_1;
		drv.CURRENT_COEF		=	100u;
		drv.PWM_FREQ			=	STEPPER_FREQ;
		drv.GPIO_DECAY			=	GPIO_DECAY;
		drv.GPIO_RESET			=	GPIO_RESET;
		drv.GPIO_SLEEP			=	GPIO_SLEEP;
		drv.GPIO_FAULT			=	DRV1_GPIO_FAULT;
		drv.GPIO_PHA			=	DRV1_GPIO_PHA;
		drv.GPIO_PHB			=	DRV1_GPIO_PHB;
		drv.GPIO_ENA			=	DRV1_GPIO_ENA;
		drv.GPIO_ENB			=	DRV1_GPIO_ENB;
		break;
	case HAL::Drv8813::DRV8813_2:
		drv.MODE				=	STEPPER_MODE.STEPPER_MODE;
		drv.USTEP_MODE			= 	USTEP_1;
		drv.CURRENT_COEF		=	100u;
		drv.PWM_FREQ			=	STEPPER_FREQ;
		drv.GPIO_DECAY			=	GPIO_DECAY;
		drv.GPIO_RESET			=	GPIO_RESET;
		drv.GPIO_SLEEP			=	GPIO_SLEEP;
		drv.GPIO_FAULT			=	DRV2_GPIO_FAULT;
		drv.GPIO_PHA			=	DRV2_GPIO_PHA;
		drv.GPIO_PHB			=	DRV2_GPIO_PHB;
		drv.GPIO_ENA			=	DRV2_GPIO_ENA;
		drv.GPIO_ENB			=	DRV2_GPIO_ENB;
		break;
	case HAL::Drv8813::DRV8813_3:
		drv.MODE				=	STEPPER_MODE.STEPPER_MODE;
		drv.USTEP_MODE			= 	USTEP_1;
		drv.CURRENT_COEF		=	100u;			
		drv.PWM_FREQ			=	STEPPER_FREQ;
		drv.GPIO_DECAY			=	GPIO_DECAY;
		drv.GPIO_RESET			=	GPIO_RESET;
		drv.GPIO_SLEEP			=	GPIO_SLEEP;
		drv.GPIO_FAULT			=	DRV3_GPIO_FAULT;
		drv.GPIO_PHA			=	DRV3_GPIO_PHA;
		drv.GPIO_PHB			=	DRV3_GPIO_PHB;
		drv.GPIO_ENA			=	DRV3_GPIO_ENA;
		drv.GPIO_ENB			=	DRV3_GPIO_ENB;
		break;
	case HAL::Drv8813::DRV8813_4:
		drv.MODE				=	STEPPER_MODE.STEPPER_MODE;
		drv.USTEP_MODE			= 	USTEP_1;
		drv.CURRENT_COEF		=	100u;
		drv.PWM_FREQ			=	STEPPER_FREQ;
		drv.GPIO_DECAY			=	GPIO_DECAY;
		drv.GPIO_RESET			=	GPIO_RESET;
		drv.GPIO_SLEEP			=	GPIO_SLEEP;
		drv.GPIO_FAULT			=	DRV4_GPIO_FAULT;
		drv.GPIO_PHA			=	DRV4_GPIO_PHA;
		drv.GPIO_PHB			=	DRV4_GPIO_PHB;
		drv.GPIO_ENA			=	DRV4_GPIO_ENA;
		drv.GPIO_ENB			=	DRV4_GPIO_ENB;
		break;
	case HAL::Drv8813::DRV8813_5:
		drv.MODE				=	STEPPER_MODE.STEPPER_MODE;
		drv.USTEP_MODE			= 	USTEP_1;
		drv.CURRENT_COEF		=	100u;
		drv.PWM_FREQ			=	STEPPER_FREQ;
		drv.GPIO_DECAY			=	GPIO_DECAY;
		drv.GPIO_RESET			=	GPIO_RESET;
		drv.GPIO_SLEEP			=	GPIO_SLEEP;
		drv.GPIO_FAULT			=	DRV5_GPIO_FAULT;
		drv.GPIO_PHA			=	DRV5_GPIO_PHA;
		drv.GPIO_PHB			=	DRV5_GPIO_PHB;
		drv.GPIO_ENA			=	DRV5_GPIO_ENA;
		drv.GPIO_ENB			=	DRV5_GPIO_ENB;
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

	drv->GpioInst->DECAY 				= GPIO::GetInstance(drv.DRV8813.GPIO_DECAY);
	drv->GpioInst->RESET 				= GPIO::GetInstance(drv.DRV8813.GPIO_RESET);
	drv->GpioInst->SLEEP 				= GPIO::GetInstance(drv.DRV8813.GPIO_SLEEP);
	drv->GpioInst->FAULT 				= GPIO::GetInstance(drv.DRV8813.DRV5_GPIO_FAULT);
	drv->GpioInst->PHA	 				= GPIO::GetInstance(drv.DRV8813.DRV5_GPIO_PHA);
	drv->GpioInst->PHB	 				= GPIO::GetInstance(drv.DRV8813.DRV5_GPIO_PHB);
	drv->GpioInst->ENA	 				= PWM::GetInstance(drv.DRV8813.DRV5_GPIO_ENA);
	drv->GpioInst->ENB	 				= PWM::GetInstance(drv.DRV8813.DRV5_GPIO_ENB);

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
		if(_drv8813[i] != null && _drv8813[i]->def->Drv8813Mode==Drv8813Mode.STEPPER_MODE)	//if activ module and stepper mode
		{
			if((nb_pulse!=0 or run) && _drv8813[i]->direction!=DISABLED)
			{
				if(CptStep % _drv8813[i]->period_tick)
				{
					if(_drv8813[i]->direction == Drv8813State.FORWARD)				//forward 1 step
					{
						_drv8813[i]->stepIndex += _drv8813[i]->def->USTEP_MODE;
						if(_drv8813[i]->stepIndex >= MAX_USTEP)
							_drv8813[i]->stepIndex=0;
					}
					else if(_drv8813[i]->direction == Drv8813State.BACKWARD)		//backward 1 step
					{
						if(_drv8813[i]->stepIndex == 0)
							_drv8813[i]->stepIndex = MAX_USTEP;
						_drv8813[i]->stepIndex -= _drv8813[i]->def->USTEP_MODE;
					}		
				}
			}
			ManageStepper(_drv8813[i]);		//manage IO pin and PWM function of step index
		}
	}

	CptStep++;
	if(CptStep>=FREQ_TICK_REF)
		CptStep=0;
}

/**
 * @brief manage IO pin and PWM function of step index
 */
static void ManageStepper (DRV8813_DEF* def)
{
	if(def->direction==DISABLED)
	{
		def->GpioInst->ENA->SetDutyCycle(0);
		def->GpioInst->ENB->SetDutyCycle(0);
	}
	else
	{
		uint32_t pwmA = STEP_DEF_16[def->stepIndex]->PWM_A;	//current index PWM
		pwmA *= drv->CURRENT_COEF;
		pwmA /= 100u;		

		uint32_t pwmB = STEP_DEF_16[def->stepIndex]->PWM_B;	//current index PWM
		pwmB *= drv->CURRENT_COEF;
		pwmB /= 100u;

		//Set phase and PWM
		if(STEP_DEF_16[def->stepIndex]->PositivA)
			def->GpioInst->PHA->Set(GPIO::State::High);
		else
			def->GpioInst->PHA->Set(GPIO::State::Low);
		def->GpioInst->ENA->SetDutyCycle(pwmA);

		if(STEP_DEF_16[def->stepIndex]->PositivB)
			def->GpioInst->PHB->Set(GPIO::State::High);
		else
			def->GpioInst->PHB->Set(GPIO::State::Low);
		def->GpioInst->ENA->SetDutyCycle(pwmB);
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

