/**
 * @file	PWM.cpp
 * @author	Kevin WYSOCKI
 * @date	14 nov. 2016
 * @brief	PWM Abstraction class
 */

#include <stddef.h>
#include "PWM.hpp"
#include "common.h"

using namespace HAL;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

// TIM1_CH1
#define PWM0_IO_PORT		(GPIOE)
#define PWM0_IO_PIN			(GPIO_Pin_9)
#define PWM0_IO_PINSOURCE	(GPIO_PinSource9)
#define PWM0_IO_AF			(GPIO_AF_TIM1)
#define PWM0_FREQ			(100000u)
#define PWM0_DUTYCYCLE		(0.5f)
#define PWM0_TIMER			(TIM1)
#define PWM0_TIMER_CHANNEL	(TIM_Channel_1)
#define PWM0_TIMER_FREQ		(SystemCoreClock / 2)	// TIM2 clock is derivated from APB2 clock

// TIM1_CH2
#define PWM1_IO_PORT		(GPIOE)
#define PWM1_IO_PIN			(GPIO_Pin_11)
#define PWM1_IO_PINSOURCE	(GPIO_PinSource1)
#define PWM1_IO_AF			(GPIO_AF_TIM1)
#define PWM1_FREQ			(100000u)
#define PWM1_DUTYCYCLE		(0.5f)
#define PWM1_TIMER			(TIM1)
#define PWM1_TIMER_CHANNEL	(TIM_Channel_2)
#define PWM1_TIMER_FREQ		(SystemCoreClock / 2)	// TIM2 clock is derivated from APB2 clock

// TIM1_CH3
#define PWM2_IO_PORT		(GPIOE)
#define PWM2_IO_PIN			(GPIO_Pin_13)
#define PWM2_IO_PINSOURCE	(GPIO_PinSource13)
#define PWM2_IO_AF			(GPIO_AF_TIM1)
#define PWM2_FREQ			(100000u)
#define PWM2_DUTYCYCLE		(0.5f)
#define PWM2_TIMER			(TIM1)
#define PWM2_TIMER_CHANNEL	(TIM_Channel_3)
#define PWM2_TIMER_FREQ		(SystemCoreClock / 2)	// TIM2 clock is derivated from APB2 clock

// TIM1_CH4
#define PWM3_IO_PORT		(GPIOE)
#define PWM3_IO_PIN			(GPIO_Pin_14)
#define PWM3_IO_PINSOURCE	(GPIO_PinSource14)
#define PWM3_IO_AF			(GPIO_AF_TIM1)
#define PWM3_FREQ			(100000u)
#define PWM3_DUTYCYCLE		(0.5f)
#define PWM3_TIMER			(TIM1)
#define PWM3_TIMER_CHANNEL	(TIM_Channel_4)
#define PWM3_TIMER_FREQ		(SystemCoreClock / 2)	// TIM2 clock is derivated from APB2 clock

// TIM2_CH1
#define PWM4_IO_PORT		(GPIOA)
#define PWM4_IO_PIN			(GPIO_Pin_15
#define PWM4_IO_PINSOURCE	(GPIO_PinSource15)
#define PWM4_IO_AF			(GPIO_AF_TIM2)
#define PWM4_FREQ			(100000u)
#define PWM4_DUTYCYCLE		(0.5f)
#define PWM4_TIMER			(TIM2)
#define PWM4_TIMER_CHANNEL	(TIM_Channel_1)
#define PWM4_TIMER_FREQ		(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM2_CH2
#define PWM5_IO_PORT		(GPIOB)
#define PWM5_IO_PIN			(GPIO_Pin_3)
#define PWM5_IO_PINSOURCE	(GPIO_PinSource3)
#define PWM5_IO_AF			(GPIO_AF_TIM2)
#define PWM5_FREQ			(100000u)
#define PWM5_DUTYCYCLE		(0.5f)
#define PWM5_TIMER			(TIM2)
#define PWM5_TIMER_CHANNEL	(TIM_Channel_2)
#define PWM5_TIMER_FREQ		(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM2_CH3
#define PWM6_IO_PORT		(GPIOB)
#define PWM6_IO_PIN			(GPIO_Pin_10)
#define PWM6_IO_PINSOURCE	(GPIO_PinSource10)
#define PWM6_IO_AF			(GPIO_AF_TIM2)
#define PWM6_FREQ			(100000u)
#define PWM6_DUTYCYCLE		(0.5f)
#define PWM6_TIMER			(TIM2)
#define PWM6_TIMER_CHANNEL	(TIM_Channel_3)
#define PWM6_TIMER_FREQ		(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM2_CH4
#define PWM7_IO_PORT		(GPIOB)
#define PWM7_IO_PIN			(GPIO_Pin_2)
#define PWM7_IO_PINSOURCE	(GPIO_PinSource2)
#define PWM7_IO_AF			(GPIO_AF_TIM2)
#define PWM7_FREQ			(100000u)
#define PWM7_DUTYCYCLE		(0.5f)
#define PWM7_TIMER			(TIM2)
#define PWM7_TIMER_CHANNEL	(TIM_Channel_4)
#define PWM7_TIMER_FREQ		(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM3_CH1
#define PWM8_IO_PORT		(GPIOC)
#define PWM8_IO_PIN			(GPIO_Pin_6)
#define PWM8_IO_PINSOURCE	(GPIO_PinSource6)
#define PWM8_IO_AF			(GPIO_AF_TIM3)
#define PWM8_FREQ			(100000u)
#define PWM8_DUTYCYCLE		(0.5f)
#define PWM8_TIMER			(TIM3)
#define PWM8_TIMER_CHANNEL	(TIM_Channel_1)
#define PWM8_TIMER_FREQ		(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM3_CH2
#define PWM9_IO_PORT		(GPIOC)
#define PWM9_IO_PIN			(GPIO_Pin_7)
#define PWM9_IO_PINSOURCE	(GPIO_PinSource7)
#define PWM9_IO_AF			(GPIO_AF_TIM3)
#define PWM9_FREQ			(100000u)
#define PWM9_DUTYCYCLE		(0.5f)
#define PWM9_TIMER			(TIM3)
#define PWM9_TIMER_CHANNEL	(TIM_Channel_2)
#define PWM9_TIMER_FREQ		(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM3_CH3
#define PWM10_IO_PORT		(GPIOC)
#define PWM10_IO_PIN		(GPIO_Pin_8)
#define PWM10_IO_PINSOURCE	(GPIO_PinSource8)
#define PWM10_IO_AF			(GPIO_AF_TIM3)
#define PWM10_FREQ			(100000u)
#define PWM10_DUTYCYCLE		(0.5f)
#define PWM10_TIMER			(TIM3)
#define PWM10_TIMER_CHANNEL	(TIM_Channel_3)
#define PWM10_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM3_CH4
#define PWM11_IO_PORT		(GPIOC)
#define PWM11_IO_PIN		(GPIO_Pin_9)
#define PWM11_IO_PINSOURCE	(GPIO_PinSource9)
#define PWM11_IO_AF			(GPIO_AF_TIM3)
#define PWM11_FREQ			(100000u)
#define PWM11_DUTYCYCLE		(0.5f)
#define PWM11_TIMER			(TIM3)
#define PWM11_TIMER_CHANNEL	(TIM_Channel_4)
#define PWM11_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM4_CH1
#define PWM12_IO_PORT		(GPIOD)
#define PWM12_IO_PIN		(GPIO_Pin_12)
#define PWM12_IO_PINSOURCE	(GPIO_PinSource12)
#define PWM12_IO_AF			(GPIO_AF_TIM4)
#define PWM12_FREQ			(100000u)
#define PWM12_DUTYCYCLE		(0.5f)
#define PWM12_TIMER			(TIM4)
#define PWM12_TIMER_CHANNEL	(TIM_Channel_1)
#define PWM12_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM4_CH2
#define PWM13_IO_PORT		(GPIOD)
#define PWM13_IO_PIN		(GPIO_Pin_13)
#define PWM13_IO_PINSOURCE	(GPIO_PinSource13)
#define PWM13_IO_AF			(GPIO_AF_TIM4)
#define PWM13_FREQ			(100000u)
#define PWM13_DUTYCYCLE		(0.5f)
#define PWM13_TIMER			(TIM4)
#define PWM13_TIMER_CHANNEL	(TIM_Channel_2)
#define PWM13_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM4_CH3
#define PWM14_IO_PORT		(GPIOD)
#define PWM14_IO_PIN		(GPIO_Pin_14)
#define PWM14_IO_PINSOURCE	(GPIO_PinSource14)
#define PWM14_IO_AF			(GPIO_AF_TIM4)
#define PWM14_FREQ			(100000u)
#define PWM14_DUTYCYCLE		(0.5f)
#define PWM14_TIMER			(TIM4)
#define PWM14_TIMER_CHANNEL	(TIM_Channel_3)
#define PWM14_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM4_CH4
#define PWM15_IO_PORT		(GPIOD)
#define PWM15_IO_PIN		(GPIO_Pin_15)
#define PWM15_IO_PINSOURCE	(GPIO_PinSource15)
#define PWM15_IO_AF			(GPIO_AF_TIM4)
#define PWM15_FREQ			(100000u)
#define PWM15_DUTYCYCLE		(0.5f)
#define PWM15_TIMER			(TIM4)
#define PWM15_TIMER_CHANNEL	(TIM_Channel_4)
#define PWM15_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM5_CH1
#define PWM16_IO_PORT		(GPIOA)
#define PWM16_IO_PIN		(GPIO_Pin_0)
#define PWM16_IO_PINSOURCE	(GPIO_PinSource0)
#define PWM16_IO_AF			(GPIO_AF_TIM5)
#define PWM16_FREQ			(100000u)
#define PWM16_DUTYCYCLE		(0.5f)
#define PWM16_TIMER			(TIM5)
#define PWM16_TIMER_CHANNEL	(TIM_Channel_1)
#define PWM16_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM5_CH2
#define PWM17_IO_PORT		(GPIOA)
#define PWM17_IO_PIN		(GPIO_Pin_1)
#define PWM17_IO_PINSOURCE	(GPIO_PinSource1)
#define PWM17_IO_AF			(GPIO_AF_TIM5)
#define PWM17_FREQ			(100000u)
#define PWM17_DUTYCYCLE		(0.5f)
#define PWM17_TIMER			(TIM5)
#define PWM17_TIMER_CHANNEL	(TIM_Channel_2)
#define PWM17_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM5_CH3
#define PWM18_IO_PORT		(GPIOA)
#define PWM18_IO_PIN		(GPIO_Pin_2)
#define PWM18_IO_PINSOURCE	(GPIO_PinSource2)
#define PWM18_IO_AF			(GPIO_AF_TIM5)
#define PWM18_FREQ			(100000u)
#define PWM18_DUTYCYCLE		(0.5f)
#define PWM18_TIMER			(TIM5)
#define PWM18_TIMER_CHANNEL	(TIM_Channel_3)
#define PWM18_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM5_CH4
#define PWM19_IO_PORT		(GPIOA)
#define PWM19_IO_PIN		(GPIO_Pin_3)
#define PWM19_IO_PINSOURCE	(GPIO_PinSource3)
#define PWM19_IO_AF			(GPIO_AF_TIM5)
#define PWM19_FREQ			(100000u)
#define PWM19_DUTYCYCLE		(0.5f)
#define PWM19_TIMER			(TIM5)
#define PWM19_TIMER_CHANNEL	(TIM_Channel_4)
#define PWM19_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM9_CH1
#define PWM20_IO_PORT		(GPIOE)
#define PWM20_IO_PIN		(GPIO_Pin_5)
#define PWM20_IO_PINSOURCE	(GPIO_PinSource5)
#define PWM20_IO_AF			(GPIO_AF_TIM9)
#define PWM20_FREQ			(100000u)
#define PWM20_DUTYCYCLE		(0.5f)
#define PWM20_TIMER			(TIM9)
#define PWM20_TIMER_CHANNEL	(TIM_Channel_1)
#define PWM20_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB2 clock

// TIM9_CH2
#define PWM21_IO_PORT		(GPIOE)
#define PWM21_IO_PIN		(GPIO_Pin_6)
#define PWM21_IO_PINSOURCE	(GPIO_PinSource6)
#define PWM21_IO_AF			(GPIO_AF_TIM9)
#define PWM21_FREQ			(100000u)
#define PWM21_DUTYCYCLE		(0.5f)
#define PWM21_TIMER			(TIM9)
#define PWM21_TIMER_CHANNEL	(TIM_Channel_2)
#define PWM21_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB2 clock

// TIM10_CH1
#define PWM22_IO_PORT		(GPIOF)
#define PWM22_IO_PIN		(GPIO_Pin_6)
#define PWM22_IO_PINSOURCE	(GPIO_PinSource6)
#define PWM22_IO_AF			(GPIO_AF_TIM10)
#define PWM22_FREQ			(100000u)
#define PWM22_DUTYCYCLE		(0.5f)
#define PWM22_TIMER			(TIM10)
#define PWM22_TIMER_CHANNEL	(TIM_Channel_1)
#define PWM22_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB2 clock

// TIM11_CH1
#define PWM23_IO_PORT		(GPIOF)
#define PWM23_IO_PIN		(GPIO_Pin_7)
#define PWM23_IO_PINSOURCE	(GPIO_PinSource7)
#define PWM23_IO_AF			(GPIO_AF_TIM11)
#define PWM23_FREQ			(100000u)
#define PWM23_DUTYCYCLE		(0.5f)
#define PWM23_TIMER			(TIM11)
#define PWM23_TIMER_CHANNEL	(TIM_Channel_1)
#define PWM23_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB2 clock

// TIM12_CH1
#define PWM24_IO_PORT		(GPIOB)
#define PWM24_IO_PIN		(GPIO_Pin_15)
#define PWM24_IO_PINSOURCE	(GPIO_PinSource15)
#define PWM24_IO_AF			(GPIO_AF_TIM12)
#define PWM24_FREQ			(100000u)
#define PWM24_DUTYCYCLE		(0.5f)
#define PWM24_TIMER			(TIM12)
#define PWM24_TIMER_CHANNEL	(TIM_Channel_1)
#define PWM24_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM12_CH2
#define PWM25_IO_PORT		(GPIOB)
#define PWM25_IO_PIN		(GPIO_Pin_14)
#define PWM25_IO_PINSOURCE	(GPIO_PinSource14)
#define PWM25_IO_AF			(GPIO_AF_TIM12)
#define PWM25_FREQ			(100000u)
#define PWM25_DUTYCYCLE		(0.5f)
#define PWM25_TIMER			(TIM12)
#define PWM25_TIMER_CHANNEL	(TIM_Channel_2)
#define PWM25_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock


// TIM13_CH1
#define PWM26_IO_PORT		(GPIOF)
#define PWM26_IO_PIN		(GPIO_Pin_8)
#define PWM26_IO_PINSOURCE	(GPIO_PinSource8)
#define PWM26_IO_AF			(GPIO_AF_TIM13)
#define PWM26_FREQ			(100000u)
#define PWM26_DUTYCYCLE		(0.5f)
#define PWM26_TIMER			(TIM13)
#define PWM26_TIMER_CHANNEL	(TIM_Channel_1)
#define PWM26_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock

// TIM14_CH1
#define PWM27_IO_PORT		(GPIOF)
#define PWM27_IO_PIN		(GPIO_Pin_9)
#define PWM27_IO_PINSOURCE	(GPIO_PinSource9)
#define PWM27_IO_AF			(GPIO_AF_TIM14)
#define PWM27_FREQ			(100000u)
#define PWM27_DUTYCYCLE		(0.5f)
#define PWM27_TIMER			(TIM14)
#define PWM27_TIMER_CHANNEL	(TIM_Channel_1)
#define PWM27_TIMER_FREQ	(SystemCoreClock / 2)	// TIM2 clock is derivated from APB1 clock



/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

/**
 * @brief PWM instances
 */
static PWM* _pwm[PWM::PWM_MAX] = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @brief Retrieve PWM definitions from PWM ID
 * @param id : PWM ID
 * @return PWM_DEF structure
 */
static PWM_DEF _getPWMStruct (enum PWM::ID id)
{
	PWM_DEF pwm;

	assert(id < HAL::PWM::PWM_MAX);

	switch(id)
	{
	case PWM::PWM0:
		pwm.IO.PORT					=	PWM0_IO_PORT;
		pwm.IO.PIN					=	PWM0_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM0_IO_PINSOURCE;
		pwm.IO.AF					=	PWM0_IO_AF;
		pwm.TIMER.TIMER				=	PWM0_TIMER;
		pwm.TIMER.CHANNEL			=	PWM0_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM0_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM0_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM0_DUTYCYCLE;
		break;
	case PWM::PWM1:
		pwm.IO.PORT					=	PWM1_IO_PORT;
		pwm.IO.PIN					=	PWM1_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM1_IO_PINSOURCE;
		pwm.IO.AF					=	PWM1_IO_AF;
		pwm.TIMER.TIMER				=	PWM1_TIMER;
		pwm.TIMER.CHANNEL			=	PWM1_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM1_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM1_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM1_DUTYCYCLE;
		break;
	case PWM::PWM2:
		pwm.IO.PORT					=	PWM2_IO_PORT;
		pwm.IO.PIN					=	PWM2_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM2_IO_PINSOURCE;
		pwm.IO.AF					=	PWM2_IO_AF;
		pwm.TIMER.TIMER				=	PWM2_TIMER;
		pwm.TIMER.CHANNEL			=	PWM2_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM2_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM2_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM2_DUTYCYCLE;
		break;
	case PWM::PWM3:
		pwm.IO.PORT					=	PWM3_IO_PORT;
		pwm.IO.PIN					=	PWM3_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM3_IO_PINSOURCE;
		pwm.IO.AF					=	PWM3_IO_AF;
		pwm.TIMER.TIMER				=	PWM3_TIMER;
		pwm.TIMER.CHANNEL			=	PWM3_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM3_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM3_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM3_DUTYCYCLE;
		break;
	case PWM::PWM4:
		pwm.IO.PORT					=	PWM4_IO_PORT;
		pwm.IO.PIN					=	PWM4_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM4_IO_PINSOURCE;
		pwm.IO.AF					=	PWM4_IO_AF;
		pwm.TIMER.TIMER				=	PWM4_TIMER;
		pwm.TIMER.CHANNEL			=	PWM4_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM4_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM4_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM4_DUTYCYCLE;
		break;
	case PWM::PWM5:
		pwm.IO.PORT					=	PWM5_IO_PORT;
		pwm.IO.PIN					=	PWM5_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM5_IO_PINSOURCE;
		pwm.IO.AF					=	PWM5_IO_AF;
		pwm.TIMER.TIMER				=	PWM5_TIMER;
		pwm.TIMER.CHANNEL			=	PWM5_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM5_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM5_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM5_DUTYCYCLE;
		break;
	case PWM::PWM6:
		pwm.IO.PORT					=	PWM6_IO_PORT;
		pwm.IO.PIN					=	PWM6_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM6_IO_PINSOURCE;
		pwm.IO.AF					=	PWM6_IO_AF;
		pwm.TIMER.TIMER				=	PWM6_TIMER;
		pwm.TIMER.CHANNEL			=	PWM6_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM6_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM6_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM6_DUTYCYCLE;
		break;
	case PWM::PWM7:
		pwm.IO.PORT					=	PWM7_IO_PORT;
		pwm.IO.PIN					=	PWM7_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM7_IO_PINSOURCE;
		pwm.IO.AF					=	PWM7_IO_AF;
		pwm.TIMER.TIMER				=	PWM7_TIMER;
		pwm.TIMER.CHANNEL			=	PWM7_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM7_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM7_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM7_DUTYCYCLE;
		break;	
	case PWM::PWM8:
		pwm.IO.PORT					=	PWM8_IO_PORT;
		pwm.IO.PIN					=	PWM8_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM8_IO_PINSOURCE;
		pwm.IO.AF					=	PWM8_IO_AF;
		pwm.TIMER.TIMER				=	PWM8_TIMER;
		pwm.TIMER.CHANNEL			=	PWM8_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM8_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM8_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM8_DUTYCYCLE;
		break;
	case PWM::PWM9:
		pwm.IO.PORT					=	PWM9_IO_PORT;
		pwm.IO.PIN					=	PWM9_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM9_IO_PINSOURCE;
		pwm.IO.AF					=	PWM9_IO_AF;
		pwm.TIMER.TIMER				=	PWM9_TIMER;
		pwm.TIMER.CHANNEL			=	PWM9_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM9_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM9_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM9_DUTYCYCLE;
		break;
	case PWM::PWM10:
		pwm.IO.PORT					=	PWM10_IO_PORT;
		pwm.IO.PIN					=	PWM10_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM10_IO_PINSOURCE;
		pwm.IO.AF					=	PWM10_IO_AF;
		pwm.TIMER.TIMER				=	PWM10_TIMER;
		pwm.TIMER.CHANNEL			=	PWM10_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM10_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM10_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM10_DUTYCYCLE;
		break;
	case PWM::PWM11:
		pwm.IO.PORT					=	PWM11_IO_PORT;
		pwm.IO.PIN					=	PWM11_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM11_IO_PINSOURCE;
		pwm.IO.AF					=	PWM11_IO_AF;
		pwm.TIMER.TIMER				=	PWM11_TIMER;
		pwm.TIMER.CHANNEL			=	PWM11_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM11_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM11_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM11_DUTYCYCLE;
		break;
	case PWM::PWM12:
		pwm.IO.PORT					=	PWM12_IO_PORT;
		pwm.IO.PIN					=	PWM12_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM12_IO_PINSOURCE;
		pwm.IO.AF					=	PWM12_IO_AF;
		pwm.TIMER.TIMER				=	PWM12_TIMER;
		pwm.TIMER.CHANNEL			=	PWM12_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM12_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM12_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM12_DUTYCYCLE;
		break;
	case PWM::PWM13:
		pwm.IO.PORT					=	PWM13_IO_PORT;
		pwm.IO.PIN					=	PWM13_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM13_IO_PINSOURCE;
		pwm.IO.AF					=	PWM13_IO_AF;
		pwm.TIMER.TIMER				=	PWM13_TIMER;
		pwm.TIMER.CHANNEL			=	PWM13_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM13_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM13_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM13_DUTYCYCLE;
		break;
	case PWM::PWM14:
		pwm.IO.PORT					=	PWM14_IO_PORT;
		pwm.IO.PIN					=	PWM14_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM14_IO_PINSOURCE;
		pwm.IO.AF					=	PWM14_IO_AF;
		pwm.TIMER.TIMER				=	PWM14_TIMER;
		pwm.TIMER.CHANNEL			=	PWM14_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM14_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM14_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM14_DUTYCYCLE;
		break;
	case PWM::PWM15:
		pwm.IO.PORT					=	PWM15_IO_PORT;
		pwm.IO.PIN					=	PWM15_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM15_IO_PINSOURCE;
		pwm.IO.AF					=	PWM15_IO_AF;
		pwm.TIMER.TIMER				=	PWM15_TIMER;
		pwm.TIMER.CHANNEL			=	PWM15_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM15_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM15_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM15_DUTYCYCLE;
		break;
	case PWM::PWM16:
		pwm.IO.PORT					=	PWM16_IO_PORT;
		pwm.IO.PIN					=	PWM16_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM16_IO_PINSOURCE;
		pwm.IO.AF					=	PWM16_IO_AF;
		pwm.TIMER.TIMER				=	PWM16_TIMER;
		pwm.TIMER.CHANNEL			=	PWM16_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM16_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM16_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM16_DUTYCYCLE;
		break;	
	case PWM::PWM17:
		pwm.IO.PORT					=	PWM17_IO_PORT;
		pwm.IO.PIN					=	PWM17_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM17_IO_PINSOURCE;
		pwm.IO.AF					=	PWM17_IO_AF;
		pwm.TIMER.TIMER				=	PWM17_TIMER;
		pwm.TIMER.CHANNEL			=	PWM17_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM17_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM17_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM17_DUTYCYCLE;
		break;
	case PWM::PWM18:
		pwm.IO.PORT					=	PWM18_IO_PORT;
		pwm.IO.PIN					=	PWM18_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM18_IO_PINSOURCE;
		pwm.IO.AF					=	PWM18_IO_AF;
		pwm.TIMER.TIMER				=	PWM18_TIMER;
		pwm.TIMER.CHANNEL			=	PWM18_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM18_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM18_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM18_DUTYCYCLE;
		break;
	case PWM::PWM19:
		pwm.IO.PORT					=	PWM19_IO_PORT;
		pwm.IO.PIN					=	PWM19_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM19_IO_PINSOURCE;
		pwm.IO.AF					=	PWM19_IO_AF;
		pwm.TIMER.TIMER				=	PWM19_TIMER;
		pwm.TIMER.CHANNEL			=	PWM19_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM19_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM19_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM19_DUTYCYCLE;
		break;
	case PWM::PWM20:
		pwm.IO.PORT					=	PWM20_IO_PORT;
		pwm.IO.PIN					=	PWM20_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM20_IO_PINSOURCE;
		pwm.IO.AF					=	PWM20_IO_AF;
		pwm.TIMER.TIMER				=	PWM20_TIMER;
		pwm.TIMER.CHANNEL			=	PWM20_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM20_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM20_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM20_DUTYCYCLE;
		break;
	case PWM::PWM21:
		pwm.IO.PORT					=	PWM21_IO_PORT;
		pwm.IO.PIN					=	PWM21_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM21_IO_PINSOURCE;
		pwm.IO.AF					=	PWM21_IO_AF;
		pwm.TIMER.TIMER				=	PWM21_TIMER;
		pwm.TIMER.CHANNEL			=	PWM21_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM21_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM21_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM21_DUTYCYCLE;
		break;
	case PWM::PWM22:
		pwm.IO.PORT					=	PWM22_IO_PORT;
		pwm.IO.PIN					=	PWM22_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM22_IO_PINSOURCE;
		pwm.IO.AF					=	PWM22_IO_AF;
		pwm.TIMER.TIMER				=	PWM22_TIMER;
		pwm.TIMER.CHANNEL			=	PWM22_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM22_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM22_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM22_DUTYCYCLE;
		break;
	case PWM::PWM23:
		pwm.IO.PORT					=	PWM23_IO_PORT;
		pwm.IO.PIN					=	PWM23_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM23_IO_PINSOURCE;
		pwm.IO.AF					=	PWM23_IO_AF;
		pwm.TIMER.TIMER				=	PWM23_TIMER;
		pwm.TIMER.CHANNEL			=	PWM23_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM23_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM23_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM23_DUTYCYCLE;
		break;
	case PWM::PWM24:
		pwm.IO.PORT					=	PWM24_IO_PORT;
		pwm.IO.PIN					=	PWM24_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM24_IO_PINSOURCE;
		pwm.IO.AF					=	PWM24_IO_AF;
		pwm.TIMER.TIMER				=	PWM24_TIMER;
		pwm.TIMER.CHANNEL			=	PWM24_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM24_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM24_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM24_DUTYCYCLE;
		break;
	case PWM::PWM25:
		pwm.IO.PORT					=	PWM25_IO_PORT;
		pwm.IO.PIN					=	PWM25_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM25_IO_PINSOURCE;
		pwm.IO.AF					=	PWM25_IO_AF;
		pwm.TIMER.TIMER				=	PWM25_TIMER;
		pwm.TIMER.CHANNEL			=	PWM25_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM25_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM25_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM25_DUTYCYCLE;
		break;
	case PWM::PWM26:
		pwm.IO.PORT					=	PWM26_IO_PORT;
		pwm.IO.PIN					=	PWM26_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM26_IO_PINSOURCE;
		pwm.IO.AF					=	PWM26_IO_AF;
		pwm.TIMER.TIMER				=	PWM26_TIMER;
		pwm.TIMER.CHANNEL			=	PWM26_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM26_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM26_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM26_DUTYCYCLE;
		break;
	case PWM::PWM27:
		pwm.IO.PORT					=	PWM27_IO_PORT;
		pwm.IO.PIN					=	PWM27_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM27_IO_PINSOURCE;
		pwm.IO.AF					=	PWM27_IO_AF;
		pwm.TIMER.TIMER				=	PWM27_TIMER;
		pwm.TIMER.CHANNEL			=	PWM27_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM27_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM27_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM27_DUTYCYCLE;
		break;
	default:
		break;
	}

	return pwm;
}

/**
 * @brief Initialize peripheral for a specific PWM
 * @param id : PWM ID
 */
static void _hardwareInit (enum PWM::ID id)
{
	GPIO_InitTypeDef GPIOStruct;
	TIM_TimeBaseInitTypeDef TIMBaseStruct;
	TIM_OCInitTypeDef TIMOCStruct;

	PWM_DEF pwm;

	assert(id < HAL::PWM::PWM_MAX);

	pwm = _getPWMStruct(id);

	// Init IO and TX pins
	GPIOStruct.GPIO_Mode	=	GPIO_Mode_AF;
	GPIOStruct.GPIO_OType	=	GPIO_OType_PP;
	GPIOStruct.GPIO_PuPd	=	GPIO_PuPd_NOPULL;
	GPIOStruct.GPIO_Speed	=	GPIO_High_Speed;
	GPIOStruct.GPIO_Pin		=	pwm.IO.PIN;

	GPIO_PinAFConfig(pwm.IO.PORT, pwm.IO.PINSOURCE, pwm.IO.AF);
	GPIO_Init(pwm.IO.PORT, &GPIOStruct);

	// TIMER Init
	TIMBaseStruct.TIM_ClockDivision		=	TIM_CKD_DIV1;
	TIMBaseStruct.TIM_CounterMode		=	TIM_CounterMode_Up;
	TIMBaseStruct.TIM_Prescaler			=	0u;
	TIMBaseStruct.TIM_RepetitionCounter	=	0u;
	TIMBaseStruct.TIM_Period			=	(pwm.TIMER.CLOCKFREQ / pwm.PWM.DEFAULT_FREQ) - 1u;

	TIM_Cmd(pwm.TIMER.TIMER, DISABLE);
	TIM_TimeBaseInit(pwm.TIMER.TIMER, &TIMBaseStruct);
	TIM_SetCounter(pwm.TIMER.TIMER, 0u);
	TIM_ARRPreloadConfig(pwm.TIMER.TIMER, ENABLE);

	// Output compare init
	TIMOCStruct.TIM_OCMode			=	TIM_OCMode_PWM1;
	TIMOCStruct.TIM_OCIdleState		=	TIM_OCIdleState_Reset;
	TIMOCStruct.TIM_OCNIdleState	=	TIM_OCNIdleState_Reset;
	TIMOCStruct.TIM_OCNPolarity		=	TIM_OCNPolarity_High;
	TIMOCStruct.TIM_OCPolarity		=	TIM_OCPolarity_High;
	TIMOCStruct.TIM_OutputNState	=	TIM_OutputNState_Enable;
	TIMOCStruct.TIM_OutputState		=	TIM_OutputState_Enable;
	TIMOCStruct.TIM_Pulse			=	(uint32_t)((float32_t)TIMBaseStruct.TIM_Period * pwm.PWM.DEFAULT_DUTYCYCLE);

	TIM_OCxInit(pwm.TIMER.TIMER, &TIMOCStruct, pwm.TIMER.CHANNEL);
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace HAL
{
	PWM* PWM::GetInstance (enum PWM::ID id)
	{
		assert(id < PWM::PWM_MAX);

		// if PWM instance already exists
		if(_pwm[id] != NULL)
		{
			return _pwm[id];
		}
		else
		{
			// Create PWM instance
			_pwm[id] = new PWM(id);

			return _pwm[id];
		}
	}

	PWM::PWM (enum PWM::ID id)
	{
		this->id = id;
		this->state = PWM::DISABLED;
		this->def = _getPWMStruct(id);
		this->dutyCycle = this->def.PWM.DEFAULT_DUTYCYCLE;
		this->frequency = this->def.PWM.DEFAULT_FREQ;

		_hardwareInit(id);
	}

	void PWM::SetDutyCycle (float32_t percent)
	{
		uint32_t ARR = 0u, CCR = 0u;

		if(percent > 1.0f)
		{
			percent = 1.0f;
		}
		else if(percent < 0.0f)
		{
			percent = 0.0f;
		}

		// 1. Get ARRx and CCRx value
		ARR = (PWM0_TIMER_FREQ / this->frequency) - 1u;
		CCR = (uint32_t)((float32_t)ARR * percent);

		// 2. Update ARRx register
		TIM_SetAutoreload(this->def.TIMER.TIMER, ARR);

		// 3. Update CCRx
		TIM_SetCompareX(this->def.TIMER.TIMER, CCR, this->def.TIMER.CHANNEL);

		// 4. Update instance duty cycle
		this->dutyCycle = percent;
	}

	void PWM::SetFrequency (uint32_t freq)
	{
		uint32_t ARR = 0u, CCR = 0u;

		assert(freq > 0u);

		// 1. Get ARRx and CCRx value
		ARR = (PWM0_TIMER_FREQ / freq) - 1u;
		CCR = (uint32_t)((float32_t)ARR * this->dutyCycle);

		// 2. Update ARRx register
		TIM_SetAutoreload(this->def.TIMER.TIMER, ARR);

		// 3. Update CCRx
		TIM_SetCompareX(this->def.TIMER.TIMER, CCR, this->def.TIMER.CHANNEL);

		// 4. Update instance frequency
		this->frequency = freq;
	}

	void PWM::SetState (PWM::State state)
	{
		// 1. Reset timer
		TIM_SetCounter(this->def.TIMER.TIMER, 0u);

		// 2. Set timer state
		if(state == PWM::ENABLED)
		{
			TIM_Cmd(this->def.TIMER.TIMER, ENABLE);
		}
		else
		{
			TIM_Cmd(this->def.TIMER.TIMER, DISABLE);
		}

		// 3. Update instance state
		this->state = state;
	}
}

