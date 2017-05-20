/**
 * @file	DRV8813.cpp
 * @author	Simon ROUTHIAU
 * @date	03 avr. 2017
 * @brief	DRV8813 Abstration Class
 */

#include <stddef.h>
#include "DRV8813.hpp"
#include "common.h"

using namespace HAL;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/
#define STEPPER_FREQ_PWM	(10000u)			//10kHz
#define DC_FREQ_PWM			(10000u)			//10kHz
#define FREQ_TICK_REF		(4000u)				//4kHz
#define TIMER_TICK_REF		Timer::TIMER7		//Warning: common to Servo Driver

#define USTEP_1		16
#define USTEP_2		8
#define USTEP_4		4
#define USTEP_8		2
#define USTEP_16	1
#define MAX_USTEP	64

#define	DRV_GPIO_DECAY	GPIO::GPIO6
#define	DRV_GPIO_RESET	GPIO::GPIO7
#define	DRV_GPIO_SLEEP	GPIO::GPIO8

//Drv88113 1
#define	DRV1_GPIO_FAULT	GPIO::GPIO9
#define	DRV1_GPIO_PHA	GPIO::GPIO10
#define	DRV1_GPIO_PHB	GPIO::GPIO11
#define	DRV1_GPIO_ENA	PWM::PWM0
#define	DRV1_GPIO_ENB	PWM::PWM1

//Drv88113 2
#define	DRV2_GPIO_FAULT	GPIO::GPIO12
#define	DRV2_GPIO_PHA	GPIO::GPIO13
#define	DRV2_GPIO_PHB	GPIO::GPIO14
#define	DRV2_GPIO_ENA	PWM::PWM2
#define	DRV2_GPIO_ENB	PWM::PWM3

//Drv88113 3
#define	DRV3_GPIO_FAULT	GPIO::GPIO15
#define	DRV3_GPIO_PHA	GPIO::GPIO16
#define	DRV3_GPIO_PHB	GPIO::GPIO17
#define	DRV3_GPIO_ENA	PWM::PWM8
#define	DRV3_GPIO_ENB	PWM::PWM9

//Drv88113 4
#define	DRV4_GPIO_FAULT	GPIO::GPIO18
#define	DRV4_GPIO_PHA	GPIO::GPIO19
#define	DRV4_GPIO_PHB	GPIO::GPIO20
#define	DRV4_GPIO_ENA	PWM::PWM10
#define	DRV4_GPIO_ENB	PWM::PWM11

//Drv88113 5
#define	DRV5_GPIO_FAULT	GPIO::GPIO21
#define	DRV5_GPIO_PHA	GPIO::GPIO22
#define	DRV5_GPIO_PHB	GPIO::GPIO23
#define	DRV5_GPIO_ENA	PWM::PWM12
#define	DRV5_GPIO_ENB	PWM::PWM13

/*----------------------------------------------------------------------------*/
/* Const				                                                       */
/*----------------------------------------------------------------------------*/
const PWM_STEP_DEF		STEP_DEF_16[64] = {	{true,true,71,71},{true,true,63,77},{true,true,56,83},{true,true,47,88},{true,true,38,92},{true,true,29,96},{true,true,20,98},{true,true,10,100},
											{true,true,0,100},{false,true,10,100},{false,true,20,98},{false,true,29,96},{false,true,38,92},{false,true,47,88},{false,true,56,83},{false,true,63,77},
											{false,true,71,71},{false,true,77,63},{false,true,83,56},{false,true,88,47},{false,true,92,38},{false,true,96,29},{false,true,98,20},{false,true,100,10},
											{false,true,100,0},{false,false,100,10},{false,false,98,20},{false,false,96,29},{false,false,92,38},{false,false,88,47},{false,false,83,56},{false,false,77,63},
											{false,false,71,71},{false,false,63,77},{false,false,56,83},{false,false,47,88},{false,false,38,92},{false,false,29,96},{false,false,20,98},{false,false,10,100},
											{true,false,0,100},{true,false,10,100},{true,false,20,98},{true,false,29,96},{true,false,38,92},{true,false,47,88},{true,false,56,83},{true,false,63,77},
											{true,false,71,71},{true,false,77,63},{true,false,83,56},{true,false,88,47},{true,false,92,38},{true,false,96,29},{true,false,98,20},{true,false,100,10},
											{true,true,100,0},{true,true,100,10},{true,true,98,20},{true,true,96,29},{true,true,92,38},{true,true,88,47},{true,true,83,56},{true,true,77,63}};

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

/**
 * @brief Timer instances
 */
Drv8813* _drv8813[Drv8813::DRV8813_MAX] = {NULL};

/**
 * @brief Initialisation Timer Tick Flag
 */
static bool TimerInitialized = false;


/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @brief Retrieve Drv8813 definitions from Timer ID
 * @param id : Drv8813 ID
 * @return DRV8813_DEF structure
 */
static DRV8813_DEF _getDrv8813Struct (enum Drv8813::ID id)
{
	DRV8813_DEF drv;

	assert(id < HAL::Drv8813::DRV8813_MAX);

	switch(id)
	{
	case HAL::Drv8813::DRV8813_1:
		drv.MODE				=	Drv8813Mode::STEPPER_MODE;
		drv.USTEP_MODE			= 	USTEP_1;
		drv.CURRENT_COEF		=	100u;
		drv.PWM_FREQ			=	STEPPER_FREQ_PWM;
		drv.NB_MOTOR_STEP		=	200u;
		drv.GPIO_DECAY			=	DRV_GPIO_DECAY;
		drv.GPIO_RESET			=	DRV_GPIO_RESET;
		drv.GPIO_SLEEP			=	DRV_GPIO_SLEEP;
		drv.GPIO_FAULT			=	DRV1_GPIO_FAULT;
		drv.GPIO_PHA			=	DRV1_GPIO_PHA;
		drv.GPIO_PHB			=	DRV1_GPIO_PHB;
		drv.GPIO_ENA			=	DRV1_GPIO_ENA;
		drv.GPIO_ENB			=	DRV1_GPIO_ENB;
		break;
	case HAL::Drv8813::DRV8813_2:
		drv.MODE				=	Drv8813Mode::STEPPER_MODE;
		drv.USTEP_MODE			= 	USTEP_1;
		drv.CURRENT_COEF		=	100u;
		drv.PWM_FREQ			=	STEPPER_FREQ_PWM;
		drv.NB_MOTOR_STEP		=	200u;
		drv.GPIO_DECAY			=	DRV_GPIO_DECAY;
		drv.GPIO_RESET			=	DRV_GPIO_RESET;
		drv.GPIO_SLEEP			=	DRV_GPIO_SLEEP;
		drv.GPIO_FAULT			=	DRV2_GPIO_FAULT;
		drv.GPIO_PHA			=	DRV2_GPIO_PHA;
		drv.GPIO_PHB			=	DRV2_GPIO_PHB;
		drv.GPIO_ENA			=	DRV2_GPIO_ENA;
		drv.GPIO_ENB			=	DRV2_GPIO_ENB;
		break;
	case HAL::Drv8813::DRV8813_3:
		drv.MODE				=	Drv8813Mode::STEPPER_MODE;
		drv.USTEP_MODE			= 	USTEP_1;
		drv.CURRENT_COEF		=	100u;
		drv.PWM_FREQ			=	STEPPER_FREQ_PWM;
		drv.NB_MOTOR_STEP		=	200u;
		drv.GPIO_DECAY			=	DRV_GPIO_DECAY;
		drv.GPIO_RESET			=	DRV_GPIO_RESET;
		drv.GPIO_SLEEP			=	DRV_GPIO_SLEEP;
		drv.GPIO_FAULT			=	DRV3_GPIO_FAULT;
		drv.GPIO_PHA			=	DRV3_GPIO_PHA;
		drv.GPIO_PHB			=	DRV3_GPIO_PHB;
		drv.GPIO_ENA			=	DRV3_GPIO_ENA;
		drv.GPIO_ENB			=	DRV3_GPIO_ENB;
		break;
	case HAL::Drv8813::DRV8813_4:
		drv.MODE				=	Drv8813Mode::STEPPER_MODE;
		drv.USTEP_MODE			= 	USTEP_1;
		drv.CURRENT_COEF		=	100u;
		drv.PWM_FREQ			=	STEPPER_FREQ_PWM;
		drv.NB_MOTOR_STEP		=	200u;
		drv.GPIO_DECAY			=	DRV_GPIO_DECAY;
		drv.GPIO_RESET			=	DRV_GPIO_RESET;
		drv.GPIO_SLEEP			=	DRV_GPIO_SLEEP;
		drv.GPIO_FAULT			=	DRV4_GPIO_FAULT;
		drv.GPIO_PHA			=	DRV4_GPIO_PHA;
		drv.GPIO_PHB			=	DRV4_GPIO_PHB;
		drv.GPIO_ENA			=	DRV4_GPIO_ENA;
		drv.GPIO_ENB			=	DRV4_GPIO_ENB;
		break;
	case HAL::Drv8813::DRV8813_5:
		drv.MODE				=	Drv8813Mode::STEPPER_MODE;
		drv.USTEP_MODE			= 	USTEP_1;
		drv.CURRENT_COEF		=	100u;
		drv.PWM_FREQ			=	STEPPER_FREQ_PWM;
		drv.NB_MOTOR_STEP		=	200u;
		drv.GPIO_DECAY			=	DRV_GPIO_DECAY;
		drv.GPIO_RESET			=	DRV_GPIO_RESET;
		drv.GPIO_SLEEP			=	DRV_GPIO_SLEEP;
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
 * @brief manage IO pin and PWM function of step index
 */
static void ManageStepper (Drv8813* drv)
{
	static uint32_t pwmA;
	static uint32_t pwmB;

	if(drv->direction==DISABLED)
	{
		drv->GpioInst.ENA->SetDutyCycle(0);
		drv->GpioInst.ENB->SetDutyCycle(0);
	}
	else
	{
		pwmA = STEP_DEF_16[drv->stepIndex].PWM_A;	//current index PWM
		pwmA *= drv->def.CURRENT_COEF;
		pwmA /= 100u;

		pwmB = STEP_DEF_16[drv->stepIndex].PWM_B;	//current index PWM
		pwmB *= drv->def.CURRENT_COEF;
		pwmB /= 100u;

		//////// TO DELETE, RESOLVE TORQUE PB ONLY ///////
		/*if(drv->period_tick>(FREQ_TICK_REF/100u))
		{
			if(pwmA>0) pwmA=45;
			if(pwmB>0) pwmB=45;
		}*/
		if(drv->period_tick<(FREQ_TICK_REF/500u))
		{
			if(pwmA>0) pwmA=100;
			if(pwmB>0) pwmB=100;
		}

		//Set phase and PWM
		if(STEP_DEF_16[drv->stepIndex].PositivA)
			drv->GpioInst.PHA->Set(GPIO::State::High);
		else
			drv->GpioInst.PHA->Set(GPIO::State::Low);
		drv->GpioInst.ENA->SetDutyCycle(pwmA/100.0f);

		if(STEP_DEF_16[drv->stepIndex].PositivB)
			drv->GpioInst.PHB->Set(GPIO::State::High);
		else
			drv->GpioInst.PHB->Set(GPIO::State::Low);
		drv->GpioInst.ENB->SetDutyCycle(pwmB/100.0f);
	}
}

/**
 * @brief Tick event for step generation
 */
static void TickDrv8813Event (void * obj)
{
	static uint32_t CptStep=0;
	static uint32_t i=0;

	for (i =0; i<Drv8813::ID::DRV8813_MAX; i++)
	{
		Drv8813* drv = _drv8813[i];
		if(drv != NULL && drv->def.MODE==Drv8813Mode_t::STEPPER_MODE)	//if activ module and stepper mode
		{
			if((drv->nb_pulse!=0 or drv->run) && drv->direction!=DISABLED)
			{
				if(drv->period_tick != 0 && (CptStep % drv->period_tick) == 0)
				{
					if(drv->direction == Drv8813State_t::FORWARD)				//forward 1 step
					{
						drv->stepIndex += drv->def.USTEP_MODE;
						if(drv->stepIndex >= MAX_USTEP)
							drv->stepIndex=0;

						drv->position += 1;
						if(drv->position >= drv->def.NB_MOTOR_STEP)
							drv->position=0;
					}
					else if(drv->direction == Drv8813State_t::BACKWARD)		//backward 1 step
					{
						if(drv->stepIndex == 0)
							drv->stepIndex = MAX_USTEP;
						drv->stepIndex -= drv->def.USTEP_MODE;

						if(drv->position == 0)
							drv->position=drv->def.NB_MOTOR_STEP;
						drv->position -= 1;
					}

					if(drv->nb_pulse > 0)
						drv->nb_pulse--;

					ManageStepper(drv);		//manage IO pin and PWM function of step index
				}
			}
			else if(drv->direction==DISABLED)
			{
			    ManageStepper(drv);
			}
		}
	}

	CptStep++;
	//if(CptStep>=FREQ_TICK_REF)
	//	CptStep=0;
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

		this->period_tick = FREQ_TICK_REF;
		this->direction = Drv8813State_t::FORWARD;
		this->position = 0;
		this->run = false;
		this->stepIndex = 0;
		this->nb_pulse = 0;

		this->pwm1 = 0;
		this->pwm2 = 0;

		//GPIO init
		this->GpioInst.DECAY 				= GPIO::GetInstance(def.GPIO_DECAY);
		this->GpioInst.RESET 				= GPIO::GetInstance(def.GPIO_RESET);
		this->GpioInst.SLEEP 				= GPIO::GetInstance(def.GPIO_SLEEP);
		this->GpioInst.FAULT 				= GPIO::GetInstance(def.GPIO_FAULT);
		this->GpioInst.PHA	 			= GPIO::GetInstance(def.GPIO_PHA);
		this->GpioInst.PHB	 			= GPIO::GetInstance(def.GPIO_PHB);
		this->GpioInst.ENA	 			= PWM::GetInstance(def.GPIO_ENA);
		this->GpioInst.ENB	 			= PWM::GetInstance(def.GPIO_ENB);

		this->GpioInst.RESET->Set(GPIO::State::High);
		this->GpioInst.SLEEP->Set(GPIO::State::High);
		this->GpioInst.DECAY->Set(GPIO::State::High);
		this->GpioInst.ENA->SetState(PWM::State::ENABLED);
		this->GpioInst.ENB->SetState(PWM::State::ENABLED);

		if(TimerInitialized==false)
		{
			this->tim = Timer::GetInstance (TIMER_TICK_REF);
			this->tim->SetPeriod(1000000u/FREQ_TICK_REF);
			this->tim->TimerElapsed += TickDrv8813Event;
			this->tim->Start();

			TimerInitialized = true;
		}

		//_hardwareInit(id);
	}

	uint32_t Drv8813::SetSpeedStep (uint32_t speed)
	{
		if(speed>FREQ_TICK_REF)		// Speed out of range
			return ERROR_GENERAL;

		if(speed==0)
			this->period_tick = 0;
		else
			this->period_tick = FREQ_TICK_REF/speed + 1u;
		return 0;
	}

	uint32_t Drv8813::SetSpeedRPM (float32_t speed)
	{
		return this->SetSpeedRPS(speed/60.0);
	}

	uint32_t Drv8813::SetSpeedRPS (float32_t speed)
	{
		speed = speed * (float32_t)this->def.NB_MOTOR_STEP;

		if(this->def.USTEP_MODE == USTEP_2)
			speed = speed * 2u;
		if(this->def.USTEP_MODE == USTEP_4)
					speed = speed * 4u;
		if(this->def.USTEP_MODE == USTEP_8)
					speed = speed * 8u;
		if(this->def.USTEP_MODE == USTEP_16)
					speed = speed * 16u;

		if(abs(speed)>FREQ_TICK_REF)		// Speed out of range
			return ERROR_GENERAL;


		if(speed>0)
		{
			this->period_tick = FREQ_TICK_REF/abs(speed) + 1u;
			SetDirection(Drv8813State_t::FORWARD);
			this->Start();
		}
		else if(speed<0)
		{
			this->period_tick = FREQ_TICK_REF/abs(speed) + 1u;
			SetDirection(Drv8813State_t::BACKWARD);
			this->Start();
		}
		else
			this->Stop();


		return 0;
	}

	void Drv8813::SetDirection (Drv8813State dir)
	{
		this->direction=dir;
	}

	void Drv8813::Start (void)
	{
		this->run = true;
	}

	void Drv8813::Stop (void)
	{
		this->run = false;
	}

	void Drv8813::PulseRotation (uint32_t pulse)
	{
		this->nb_pulse=pulse;
	}

	uint32_t Drv8813::ReadPosition (void)
	{
		return this->position;
	}

	uint32_t Drv8813::SetPosition (uint32_t pos)
	{
		if(pos >= this->def.NB_MOTOR_STEP)
			return ERROR_GENERAL;

		if(this->def.NB_MOTOR_STEP == 0)
			return ERROR_GENERAL;

		this->position = pos;

		return 0;
	}

	bool Drv8813::IsMoving()
    {
        if((this->nb_pulse!=0 or this->run) && this->direction!=DISABLED)
            return true;
        else
            return false;
    }

}
