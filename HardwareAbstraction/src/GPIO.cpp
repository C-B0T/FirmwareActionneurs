/**
 * @file	GPIO.cpp
 * @author	Kevin WYSOCKI
 * @date	8 nov. 2016
 * @brief	GPIO Abstraction Class
 */

#include <stdio.h>
#include "GPIO.hpp"
#include "common.h"

using namespace HAL;

#define TARGET_NUCLEO	1

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/
#if TARGET_NUCLEO==1

// LED1
#define GPIO0_PORT				(GPIOB)
#define GPIO0_PIN				(GPIO_Pin_0)
#define GPIO0_MODE				(GPIO_Mode_OUT)	

// LED2
#define GPIO1_PORT				(GPIOB)
#define GPIO1_PIN				(GPIO_Pin_7)
#define GPIO1_MODE				(GPIO_Mode_OUT) 

// LED3
#define GPIO2_PORT				(GPIOB)
#define GPIO2_PIN				(GPIO_Pin_14)
#define GPIO2_MODE				(GPIO_Mode_OUT) 

#else

// LED1
#define GPIO0_PORT				(GPIOE)			//(GPIOB)
#define GPIO0_PIN				(GPIO_Pin_2)	//(GPIO_Pin_0)
#define GPIO0_MODE				(GPIO_Mode_OUT)

// LED2
#define GPIO1_PORT				(GPIOE)			//(GPIOB)
#define GPIO1_PIN				(GPIO_Pin_3)	//(GPIO_Pin_7)
#define GPIO1_MODE				(GPIO_Mode_OUT)

// LED3
#define GPIO2_PORT				(GPIOB)			//(GPIOB)
#define GPIO2_PIN				(GPIO_Pin_8)	//(GPIO_Pin_14)
#define GPIO2_MODE				(GPIO_Mode_OUT)

#endif

// LED4
#define GPIO3_PORT				(GPIOC)
#define GPIO3_PIN				(GPIO_Pin_12)
#define GPIO3_MODE				(GPIO_Mode_OUT)

// MES_BAT
#define GPIO4_PORT				(GPIOC)
#define GPIO4_PIN				(GPIO_Pin_3)
#define GPIO4_MODE				(GPIO_Mode_AN)

// EN_5V
#define GPIO5_PORT				(GPIOE)
#define GPIO5_PIN				(GPIO_Pin_4)
#define GPIO5_MODE				(GPIO_Mode_OUT)

// MOT_DECAY
#define GPIO6_PORT				(GPIOF)
#define GPIO6_PIN				(GPIO_Pin_14)
#define GPIO6_MODE				(GPIO_Mode_OUT)

// MOT_RESET
#define GPIO7_PORT				(GPIOF)
#define GPIO7_PIN				(GPIO_Pin_15)
#define GPIO7_MODE				(GPIO_Mode_OUT)

// MOT_SLEEP
#define GPIO8_PORT				(GPIOF)
#define GPIO8_PIN				(GPIO_Pin_13)
#define GPIO8_MODE				(GPIO_Mode_OUT)

// MOT1_FAULT
#define GPIO9_PORT				(GPIOG)
#define GPIO9_PIN				(GPIO_Pin_0)
#define GPIO9_MODE				(GPIO_Mode_IN)

// MOT1_APH
#define GPIO10_PORT				(GPIOG)
#define GPIO10_PIN				(GPIO_Pin_1)
#define GPIO10_MODE				(GPIO_Mode_OUT)

// MOT1_BPH
#define GPIO11_PORT				(GPIOE)
#define GPIO11_PIN				(GPIO_Pin_7)
#define GPIO11_MODE				(GPIO_Mode_OUT)

// MOT2_FAULT
#define GPIO12_PORT				(GPIOE)
#define GPIO12_PIN				(GPIO_Pin_10)
#define GPIO12_MODE				(GPIO_Mode_IN)

// MOT2_APH
#define GPIO13_PORT				(GPIOB)
#define GPIO13_PIN				(GPIO_Pin_11)
#define GPIO13_MODE				(GPIO_Mode_OUT)

// MOT2_BPH
#define GPIO14_PORT				(GPIOE)
#define GPIO14_PIN				(GPIO_Pin_15)
#define GPIO14_MODE				(GPIO_Mode_OUT)

// MOT3_FAULT
#define GPIO15_PORT				(GPIOG)
#define GPIO15_PIN				(GPIO_Pin_7)
#define GPIO15_MODE				(GPIO_Mode_IN)

// MOT3_APH
#define GPIO16_PORT				(GPIOG)
#define GPIO16_PIN				(GPIO_Pin_3)
#define GPIO16_MODE				(GPIO_Mode_OUT)

// MOT3_BPH
#define GPIO17_PORT				(GPIOG)
#define GPIO17_PIN				(GPIO_Pin_8)
#define GPIO17_MODE				(GPIO_Mode_OUT)

// MOT4_FAULT
#define GPIO18_PORT				(GPIOC)
#define GPIO18_PIN				(GPIO_Pin_10)
#define GPIO18_MODE				(GPIO_Mode_IN)

// MOT4_APH
#define GPIO19_PORT				(GPIOA)
#define GPIO19_PIN				(GPIO_Pin_11)
#define GPIO19_MODE				(GPIO_Mode_OUT)

// MOT4_BPH
#define GPIO20_PORT				(GPIOA)
#define GPIO20_PIN				(GPIO_Pin_8)
#define GPIO20_MODE				(GPIO_Mode_OUT)

// MOT5_FAULT
#define GPIO21_PORT				(GPIOG)
#define GPIO21_PIN				(GPIO_Pin_2)
#define GPIO21_MODE				(GPIO_Mode_IN)

// MOT5_APH
#define GPIO22_PORT				(GPIOD)
#define GPIO22_PIN				(GPIO_Pin_11)
#define GPIO22_MODE				(GPIO_Mode_OUT)

// MOT5_BPH
#define GPIO23_PORT				(GPIOB)
#define GPIO23_PIN				(GPIO_Pin_13)
#define GPIO23_MODE				(GPIO_Mode_OUT)

// SEL0
#define GPIO24_PORT				(GPIOF)
#define GPIO24_PIN				(GPIO_Pin_0)
#define GPIO24_MODE				(GPIO_Mode_OUT)

// SEL1
#define GPIO25_PORT				(GPIOF)
#define GPIO25_PIN				(GPIO_Pin_1)
#define GPIO25_MODE				(GPIO_Mode_OUT)

// SEL2
#define GPIO26_PORT				(GPIOF)
#define GPIO26_PIN				(GPIO_Pin_2)
#define GPIO26_MODE				(GPIO_Mode_OUT)

// SEL3
#define GPIO27_PORT				(GPIOF)
#define GPIO27_PIN				(GPIO_Pin_3)
#define GPIO27_MODE				(GPIO_Mode_OUT)

// SEN1
#define GPIO28_PORT				(GPIOC)
#define GPIO28_PIN				(GPIO_Pin_14)
#define GPIO28_MODE				(GPIO_Mode_OUT)

// SEN2
#define GPIO29_PORT				(GPIOC)
#define GPIO29_PIN				(GPIO_Pin_13)
#define GPIO29_MODE				(GPIO_Mode_OUT)

// FAULT1
#define GPIO30_PORT				(GPIOE)
#define GPIO30_PIN				(GPIO_Pin_0)
#define GPIO30_MODE				(GPIO_Mode_IN)

// FAULT2
#define GPIO31_PORT				(GPIOE)
#define GPIO31_PIN				(GPIO_Pin_1)
#define GPIO31_MODE				(GPIO_Mode_IN)

// OUT1
#define GPIO32_PORT				(GPIOD)
#define GPIO32_PIN				(GPIO_Pin_14)
#define GPIO32_MODE				(GPIO_Mode_OUT)

// OUT2
#define GPIO33_PORT				(GPIOD)
#define GPIO33_PIN				(GPIO_Pin_15)
#define GPIO33_MODE				(GPIO_Mode_OUT)

// OUT3
#define GPIO34_PORT				(GPIOE)
#define GPIO34_PIN				(GPIO_Pin_5)
#define GPIO34_MODE				(GPIO_Mode_OUT)

// OUT4
#define GPIO35_PORT				(GPIOE)
#define GPIO35_PIN				(GPIO_Pin_6)
#define GPIO35_MODE				(GPIO_Mode_OUT)

// OUT5
#define GPIO36_PORT				(GPIOB)
#define GPIO36_PIN				(GPIO_Pin_15)
#define GPIO36_MODE				(GPIO_Mode_OUT)

// OUT6
#define GPIO37_PORT				(GPIOB)
#define GPIO37_PIN				(GPIO_Pin_14)
#define GPIO37_MODE				(GPIO_Mode_OUT)

// OUT7
#define GPIO38_PORT				(GPIOB)
#define GPIO38_PIN				(GPIO_Pin_10)
#define GPIO38_MODE				(GPIO_Mode_OUT)

// OUT8
#define GPIO39_PORT				(GPIOB)
#define GPIO39_PIN				(GPIO_Pin_2)
#define GPIO39_MODE				(GPIO_Mode_OUT)

// OUT_SENS
#define GPIO40_PORT				(GPIOC)
#define GPIO40_PIN				(GPIO_Pin_2)
#define GPIO40_MODE				(GPIO_Mode_AN)

//SERVO1
#define GPIO41_PORT				(GPIOD)
#define GPIO41_PIN				(GPIO_Pin_0)
#define GPIO41_MODE				(GPIO_Mode_OUT)

//SERVO2
#define GPIO42_PORT				(GPIOD)
#define GPIO42_PIN				(GPIO_Pin_1)
#define GPIO42_MODE				(GPIO_Mode_OUT)

//SERVO3
#define GPIO43_PORT				(GPIOD)
#define GPIO43_PIN				(GPIO_Pin_2)
#define GPIO43_MODE				(GPIO_Mode_OUT)

//SERVO4
#define GPIO44_PORT				(GPIOD)
#define GPIO44_PIN				(GPIO_Pin_3)
#define GPIO44_MODE				(GPIO_Mode_OUT)

//SERVO5
#define GPIO45_PORT				(GPIOD)
#define GPIO45_PIN				(GPIO_Pin_4)
#define GPIO45_MODE				(GPIO_Mode_OUT)

//SERVO6
#define GPIO46_PORT				(GPIOD)
#define GPIO46_PIN				(GPIO_Pin_5)
#define GPIO46_MODE				(GPIO_Mode_OUT)

//SERVO7
#define GPIO47_PORT				(GPIOD)
#define GPIO47_PIN				(GPIO_Pin_6)
#define GPIO47_MODE				(GPIO_Mode_OUT)

//SERVO8
#define GPIO48_PORT				(GPIOD)
#define GPIO48_PIN				(GPIO_Pin_7)
#define GPIO48_MODE				(GPIO_Mode_OUT)

//SERVO9
#define GPIO49_PORT				(GPIOG)
#define GPIO49_PIN				(GPIO_Pin_9)
#define GPIO49_MODE				(GPIO_Mode_OUT)

//SERVO10
#define GPIO50_PORT				(GPIOG)
#define GPIO50_PIN				(GPIO_Pin_10)
#define GPIO50_MODE				(GPIO_Mode_OUT)

//SERVO11
#define GPIO51_PORT				(GPIOG)
#define GPIO51_PIN				(GPIO_Pin_11)
#define GPIO51_MODE				(GPIO_Mode_OUT)

//SERVO12
#define GPIO52_PORT				(GPIOG)
#define GPIO52_PIN				(GPIO_Pin_12)
#define GPIO52_MODE				(GPIO_Mode_OUT)

//SERVO13
#define GPIO53_PORT				(GPIOG)
#define GPIO53_PIN				(GPIO_Pin_13)
#define GPIO53_MODE				(GPIO_Mode_OUT)

//SERVO14
#define GPIO54_PORT				(GPIOG)
#define GPIO54_PIN				(GPIO_Pin_14)
#define GPIO54_MODE				(GPIO_Mode_OUT)

//SERVO15
#define GPIO55_PORT				(GPIOG)
#define GPIO55_PIN				(GPIO_Pin_15)
#define GPIO55_MODE				(GPIO_Mode_OUT)

//SERVO16
#define GPIO56_PORT				(GPIOA)
#define GPIO56_PIN				(GPIO_Pin_2)
#define GPIO56_MODE				(GPIO_Mode_OUT)

//INPUT1
#define GPIO57_PORT				(GPIOA)
#define GPIO57_PIN				(GPIO_Pin_4)
#define GPIO57_MODE				(GPIO_Mode_IN)

//INPUT2
#define GPIO58_PORT				(GPIOA)
#define GPIO58_PIN				(GPIO_Pin_5)
#define GPIO58_MODE				(GPIO_Mode_IN)

//INPUT3
#define GPIO59_PORT				(GPIOA)
#define GPIO59_PIN				(GPIO_Pin_6)
#define GPIO59_MODE				(GPIO_Mode_IN)

//INPUT4
#define GPIO60_PORT				(GPIOA)
#define GPIO60_PIN				(GPIO_Pin_7)
#define GPIO60_MODE				(GPIO_Mode_IN)

//INPUT5
#define GPIO61_PORT				(GPIOC)
#define GPIO61_PIN				(GPIO_Pin_4)
#define GPIO61_MODE				(GPIO_Mode_IN)

//INPUT6
#define GPIO62_PORT				(GPIOC)
#define GPIO62_PIN				(GPIO_Pin_5)
#define GPIO62_MODE				(GPIO_Mode_IN)

//INPUT7
#define GPIO63_PORT				(GPIOB)
#define GPIO63_PIN				(GPIO_Pin_0)
#define GPIO63_MODE				(GPIO_Mode_IN)

//INPUT8
#define GPIO64_PORT				(GPIOB)
#define GPIO64_PIN				(GPIO_Pin_1)
#define GPIO64_MODE				(GPIO_Mode_IN)

//INPUT9
#define GPIO65_PORT				(GPIOC)
#define GPIO65_PIN				(GPIO_Pin_0)
#define GPIO65_MODE				(GPIO_Mode_IN)

//INPUT10
#define GPIO66_PORT				(GPIOC)
#define GPIO66_PIN				(GPIO_Pin_1)
#define GPIO66_MODE				(GPIO_Mode_IN)

//INPUT11
#define GPIO67_PORT				(GPIOF)
#define GPIO67_PIN				(GPIO_Pin_6)
#define GPIO67_MODE				(GPIO_Mode_IN)

//INPUT12
#define GPIO68_PORT				(GPIOF)
#define GPIO68_PIN				(GPIO_Pin_7)
#define GPIO68_MODE				(GPIO_Mode_IN)

//INPUT13
#define GPIO69_PORT				(GPIOF)
#define GPIO69_PIN				(GPIO_Pin_8)
#define GPIO69_MODE				(GPIO_Mode_IN)

//INPUT14
#define GPIO70_PORT				(GPIOF)
#define GPIO70_PIN				(GPIO_Pin_9)
#define GPIO70_MODE				(GPIO_Mode_IN)

//INPUT15
#define GPIO71_PORT				(GPIOF)
#define GPIO71_PIN				(GPIO_Pin_10)
#define GPIO71_MODE				(GPIO_Mode_IN)

//INPUT16
#define GPIO72_PORT				(GPIOF)
#define GPIO72_PIN				(GPIO_Pin_11)
#define GPIO72_MODE				(GPIO_Mode_IN)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

/**
 * @brief GPIO instances
 */
GPIO* _gpio[GPIO::GPIO_MAX] = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @brief Retrieve GPIO definitions from GPIO ID
 * @param id : GPIO ID
 * @return GPIO_DEF structure
 */
static GPIO_DEF _getGPIOStruct (enum GPIO::ID id)
{
	GPIO_DEF gpio;

	assert(id < HAL::GPIO::GPIO_MAX);

	switch(id)
	{
	case HAL::GPIO::GPIO0:
		gpio.IO.PORT	=	GPIO0_PORT;
		gpio.IO.PIN		=	GPIO0_PIN;
		gpio.IO.MODE	=	GPIO0_MODE;
		break;
	case HAL::GPIO::GPIO1:
		gpio.IO.PORT	=	GPIO1_PORT;
		gpio.IO.PIN		=	GPIO1_PIN;
		gpio.IO.MODE	=	GPIO1_MODE;
		break;
	case HAL::GPIO::GPIO2:
		gpio.IO.PORT	=	GPIO2_PORT;
		gpio.IO.PIN		=	GPIO2_PIN;
		gpio.IO.MODE	=	GPIO2_MODE;
		break;
	case HAL::GPIO::GPIO3:
		gpio.IO.PORT	=	GPIO3_PORT;
		gpio.IO.PIN		=	GPIO3_PIN;
		gpio.IO.MODE	=	GPIO3_MODE;
		break;
	case HAL::GPIO::GPIO4:
		gpio.IO.PORT	=	GPIO4_PORT;
		gpio.IO.PIN		=	GPIO4_PIN;
		gpio.IO.MODE	=	GPIO4_MODE;
		break;
	case HAL::GPIO::GPIO5:
		gpio.IO.PORT	=	GPIO5_PORT;
		gpio.IO.PIN		=	GPIO5_PIN;
		gpio.IO.MODE	=	GPIO5_MODE;
		break;
	case HAL::GPIO::GPIO6:
		gpio.IO.PORT	=	GPIO6_PORT;
		gpio.IO.PIN		=	GPIO6_PIN;
		gpio.IO.MODE	=	GPIO6_MODE;
		break;
	case HAL::GPIO::GPIO7:
		gpio.IO.PORT	=	GPIO7_PORT;
		gpio.IO.PIN		=	GPIO7_PIN;
		gpio.IO.MODE	=	GPIO7_MODE;
		break;
	case HAL::GPIO::GPIO8:
		gpio.IO.PORT	=	GPIO8_PORT;
		gpio.IO.PIN		=	GPIO8_PIN;
		gpio.IO.MODE	=	GPIO8_MODE;
		break;
	case HAL::GPIO::GPIO9:
		gpio.IO.PORT	=	GPIO9_PORT;
		gpio.IO.PIN		=	GPIO9_PIN;
		gpio.IO.MODE	=	GPIO9_MODE;
		break;
	case HAL::GPIO::GPIO10:
		gpio.IO.PORT	=	GPIO10_PORT;
		gpio.IO.PIN		=	GPIO10_PIN;
		gpio.IO.MODE	=	GPIO10_MODE;
		break;
	case HAL::GPIO::GPIO11:
		gpio.IO.PORT	=	GPIO11_PORT;
		gpio.IO.PIN		=	GPIO11_PIN;
		gpio.IO.MODE	=	GPIO11_MODE;
		break;
	case HAL::GPIO::GPIO12:
		gpio.IO.PORT	=	GPIO12_PORT;
		gpio.IO.PIN		=	GPIO12_PIN;
		gpio.IO.MODE	=	GPIO12_MODE;
		break;
	case HAL::GPIO::GPIO13:
		gpio.IO.PORT	=	GPIO13_PORT;
		gpio.IO.PIN		=	GPIO13_PIN;
		gpio.IO.MODE	=	GPIO13_MODE;
		break;
	case HAL::GPIO::GPIO14:
		gpio.IO.PORT	=	GPIO14_PORT;
		gpio.IO.PIN		=	GPIO14_PIN;
		gpio.IO.MODE	=	GPIO14_MODE;
		break;
	case HAL::GPIO::GPIO15:
		gpio.IO.PORT	=	GPIO15_PORT;
		gpio.IO.PIN		=	GPIO15_PIN;
		gpio.IO.MODE	=	GPIO15_MODE;
		break;
	case HAL::GPIO::GPIO16:
		gpio.IO.PORT	=	GPIO16_PORT;
		gpio.IO.PIN		=	GPIO16_PIN;
		gpio.IO.MODE	=	GPIO16_MODE;
		break;
	case HAL::GPIO::GPIO17:
		gpio.IO.PORT	=	GPIO17_PORT;
		gpio.IO.PIN		=	GPIO17_PIN;
		gpio.IO.MODE	=	GPIO17_MODE;
		break;
	case HAL::GPIO::GPIO18:
		gpio.IO.PORT	=	GPIO18_PORT;
		gpio.IO.PIN		=	GPIO18_PIN;
		gpio.IO.MODE	=	GPIO18_MODE;
		break;
	case HAL::GPIO::GPIO19:
		gpio.IO.PORT	=	GPIO19_PORT;
		gpio.IO.PIN		=	GPIO19_PIN;
		gpio.IO.MODE	=	GPIO19_MODE;
		break;
	case HAL::GPIO::GPIO20:
		gpio.IO.PORT	=	GPIO20_PORT;
		gpio.IO.PIN		=	GPIO20_PIN;
		gpio.IO.MODE	=	GPIO20_MODE;
		break;
	case HAL::GPIO::GPIO21:
		gpio.IO.PORT	=	GPIO21_PORT;
		gpio.IO.PIN		=	GPIO21_PIN;
		gpio.IO.MODE	=	GPIO21_MODE;
		break;
	case HAL::GPIO::GPIO22:
		gpio.IO.PORT	=	GPIO22_PORT;
		gpio.IO.PIN		=	GPIO22_PIN;
		gpio.IO.MODE	=	GPIO22_MODE;
		break;
	case HAL::GPIO::GPIO23:
		gpio.IO.PORT	=	GPIO23_PORT;
		gpio.IO.PIN		=	GPIO23_PIN;
		gpio.IO.MODE	=	GPIO23_MODE;
		break;
	case HAL::GPIO::GPIO24:
		gpio.IO.PORT	=	GPIO24_PORT;
		gpio.IO.PIN		=	GPIO24_PIN;
		gpio.IO.MODE	=	GPIO24_MODE;
		break;
	case HAL::GPIO::GPIO25:
		gpio.IO.PORT	=	GPIO25_PORT;
		gpio.IO.PIN		=	GPIO25_PIN;
		gpio.IO.MODE	=	GPIO25_MODE;
		break;
	case HAL::GPIO::GPIO26:
		gpio.IO.PORT	=	GPIO26_PORT;
		gpio.IO.PIN		=	GPIO26_PIN;
		gpio.IO.MODE	=	GPIO26_MODE;
		break;
	case HAL::GPIO::GPIO27:
		gpio.IO.PORT	=	GPIO27_PORT;
		gpio.IO.PIN		=	GPIO27_PIN;
		gpio.IO.MODE	=	GPIO27_MODE;
		break;
	case HAL::GPIO::GPIO28:
		gpio.IO.PORT	=	GPIO28_PORT;
		gpio.IO.PIN		=	GPIO28_PIN;
		gpio.IO.MODE	=	GPIO28_MODE;
		break;
	case HAL::GPIO::GPIO29:
		gpio.IO.PORT	=	GPIO29_PORT;
		gpio.IO.PIN		=	GPIO29_PIN;
		gpio.IO.MODE	=	GPIO29_MODE;
		break;
	case HAL::GPIO::GPIO30:
		gpio.IO.PORT	=	GPIO30_PORT;
		gpio.IO.PIN		=	GPIO30_PIN;
		gpio.IO.MODE	=	GPIO30_MODE;
		break;
	case HAL::GPIO::GPIO31:
		gpio.IO.PORT	=	GPIO31_PORT;
		gpio.IO.PIN		=	GPIO31_PIN;
		gpio.IO.MODE	=	GPIO31_MODE;
		break;
	case HAL::GPIO::GPIO32:
		gpio.IO.PORT	=	GPIO32_PORT;
		gpio.IO.PIN		=	GPIO32_PIN;
		gpio.IO.MODE	=	GPIO32_MODE;
		break;
	case HAL::GPIO::GPIO33:
		gpio.IO.PORT	=	GPIO33_PORT;
		gpio.IO.PIN		=	GPIO33_PIN;
		gpio.IO.MODE	=	GPIO33_MODE;
		break;
	case HAL::GPIO::GPIO34:
		gpio.IO.PORT	=	GPIO34_PORT;
		gpio.IO.PIN		=	GPIO34_PIN;
		gpio.IO.MODE	=	GPIO34_MODE;
		break;
	case HAL::GPIO::GPIO35:
		gpio.IO.PORT	=	GPIO35_PORT;
		gpio.IO.PIN		=	GPIO35_PIN;
		gpio.IO.MODE	=	GPIO35_MODE;
		break;
	case HAL::GPIO::GPIO36:
		gpio.IO.PORT	=	GPIO36_PORT;
		gpio.IO.PIN		=	GPIO36_PIN;
		gpio.IO.MODE	=	GPIO36_MODE;
		break;
	case HAL::GPIO::GPIO37:
		gpio.IO.PORT	=	GPIO37_PORT;
		gpio.IO.PIN		=	GPIO37_PIN;
		gpio.IO.MODE	=	GPIO37_MODE;
		break;
	case HAL::GPIO::GPIO38:
		gpio.IO.PORT	=	GPIO38_PORT;
		gpio.IO.PIN		=	GPIO38_PIN;
		gpio.IO.MODE	=	GPIO38_MODE;
		break;
	case HAL::GPIO::GPIO39:
		gpio.IO.PORT	=	GPIO39_PORT;
		gpio.IO.PIN		=	GPIO39_PIN;
		gpio.IO.MODE	=	GPIO39_MODE;
		break;
	case HAL::GPIO::GPIO40:
		gpio.IO.PORT	=	GPIO40_PORT;
		gpio.IO.PIN		=	GPIO40_PIN;
		gpio.IO.MODE	=	GPIO40_MODE;
		break;
	case HAL::GPIO::GPIO41:
		gpio.IO.PORT	=	GPIO41_PORT;
		gpio.IO.PIN		=	GPIO41_PIN;
		gpio.IO.MODE	=	GPIO41_MODE;
		break;
	case HAL::GPIO::GPIO42:
		gpio.IO.PORT	=	GPIO42_PORT;
		gpio.IO.PIN		=	GPIO42_PIN;
		gpio.IO.MODE	=	GPIO42_MODE;
		break;
	case HAL::GPIO::GPIO43:
		gpio.IO.PORT	=	GPIO43_PORT;
		gpio.IO.PIN		=	GPIO43_PIN;
		gpio.IO.MODE	=	GPIO43_MODE;
		break;
	case HAL::GPIO::GPIO44:
		gpio.IO.PORT	=	GPIO44_PORT;
		gpio.IO.PIN		=	GPIO44_PIN;
		gpio.IO.MODE	=	GPIO44_MODE;
		break;
	case HAL::GPIO::GPIO45:
		gpio.IO.PORT	=	GPIO45_PORT;
		gpio.IO.PIN		=	GPIO45_PIN;
		gpio.IO.MODE	=	GPIO45_MODE;
		break;
	case HAL::GPIO::GPIO46:
		gpio.IO.PORT	=	GPIO46_PORT;
		gpio.IO.PIN		=	GPIO46_PIN;
		gpio.IO.MODE	=	GPIO46_MODE;
		break;
	case HAL::GPIO::GPIO47:
		gpio.IO.PORT	=	GPIO47_PORT;
		gpio.IO.PIN		=	GPIO47_PIN;
		gpio.IO.MODE	=	GPIO47_MODE;
		break;
	case HAL::GPIO::GPIO48:
		gpio.IO.PORT	=	GPIO48_PORT;
		gpio.IO.PIN		=	GPIO48_PIN;
		gpio.IO.MODE	=	GPIO48_MODE;
		break;
	case HAL::GPIO::GPIO49:
		gpio.IO.PORT	=	GPIO49_PORT;
		gpio.IO.PIN		=	GPIO49_PIN;
		gpio.IO.MODE	=	GPIO49_MODE;
		break;
	case HAL::GPIO::GPIO50:
		gpio.IO.PORT	=	GPIO50_PORT;
		gpio.IO.PIN		=	GPIO50_PIN;
		gpio.IO.MODE	=	GPIO50_MODE;
		break;
	case HAL::GPIO::GPIO51:
		gpio.IO.PORT	=	GPIO51_PORT;
		gpio.IO.PIN		=	GPIO51_PIN;
		gpio.IO.MODE	=	GPIO51_MODE;
		break;
	case HAL::GPIO::GPIO52:
		gpio.IO.PORT	=	GPIO52_PORT;
		gpio.IO.PIN		=	GPIO52_PIN;
		gpio.IO.MODE	=	GPIO52_MODE;
		break;
	case HAL::GPIO::GPIO53:
		gpio.IO.PORT	=	GPIO53_PORT;
		gpio.IO.PIN		=	GPIO53_PIN;
		gpio.IO.MODE	=	GPIO53_MODE;
		break;
	case HAL::GPIO::GPIO54:
		gpio.IO.PORT	=	GPIO54_PORT;
		gpio.IO.PIN		=	GPIO54_PIN;
		gpio.IO.MODE	=	GPIO54_MODE;
		break;
	case HAL::GPIO::GPIO55:
		gpio.IO.PORT	=	GPIO55_PORT;
		gpio.IO.PIN		=	GPIO55_PIN;
		gpio.IO.MODE	=	GPIO55_MODE;
		break;
	case HAL::GPIO::GPIO56:
		gpio.IO.PORT	=	GPIO56_PORT;
		gpio.IO.PIN		=	GPIO56_PIN;
		gpio.IO.MODE	=	GPIO56_MODE;
		break;
	case HAL::GPIO::GPIO57:
		gpio.IO.PORT	=	GPIO57_PORT;
		gpio.IO.PIN		=	GPIO57_PIN;
		gpio.IO.MODE	=	GPIO57_MODE;
		break;
	case HAL::GPIO::GPIO58:
		gpio.IO.PORT	=	GPIO58_PORT;
		gpio.IO.PIN		=	GPIO58_PIN;
		gpio.IO.MODE	=	GPIO58_MODE;
		break;
	case HAL::GPIO::GPIO59:
		gpio.IO.PORT	=	GPIO59_PORT;
		gpio.IO.PIN		=	GPIO59_PIN;
		gpio.IO.MODE	=	GPIO59_MODE;
		break;
	case HAL::GPIO::GPIO60:
		gpio.IO.PORT	=	GPIO60_PORT;
		gpio.IO.PIN		=	GPIO60_PIN;
		gpio.IO.MODE	=	GPIO60_MODE;
		break;
	case HAL::GPIO::GPIO61:
		gpio.IO.PORT	=	GPIO61_PORT;
		gpio.IO.PIN		=	GPIO61_PIN;
		gpio.IO.MODE	=	GPIO61_MODE;
		break;
	case HAL::GPIO::GPIO62:
		gpio.IO.PORT	=	GPIO62_PORT;
		gpio.IO.PIN		=	GPIO62_PIN;
		gpio.IO.MODE	=	GPIO62_MODE;
		break;
	case HAL::GPIO::GPIO63:
		gpio.IO.PORT	=	GPIO63_PORT;
		gpio.IO.PIN		=	GPIO63_PIN;
		gpio.IO.MODE	=	GPIO63_MODE;
		break;
	case HAL::GPIO::GPIO64:
		gpio.IO.PORT	=	GPIO64_PORT;
		gpio.IO.PIN		=	GPIO64_PIN;
		gpio.IO.MODE	=	GPIO64_MODE;
		break;
	case HAL::GPIO::GPIO65:
		gpio.IO.PORT	=	GPIO65_PORT;
		gpio.IO.PIN		=	GPIO65_PIN;
		gpio.IO.MODE	=	GPIO65_MODE;
		break;
	case HAL::GPIO::GPIO66:
		gpio.IO.PORT	=	GPIO66_PORT;
		gpio.IO.PIN		=	GPIO66_PIN;
		gpio.IO.MODE	=	GPIO66_MODE;
		break;
	case HAL::GPIO::GPIO67:
		gpio.IO.PORT	=	GPIO67_PORT;
		gpio.IO.PIN		=	GPIO67_PIN;
		gpio.IO.MODE	=	GPIO67_MODE;
		break;
	case HAL::GPIO::GPIO68:
		gpio.IO.PORT	=	GPIO68_PORT;
		gpio.IO.PIN		=	GPIO68_PIN;
		gpio.IO.MODE	=	GPIO68_MODE;
		break;
	case HAL::GPIO::GPIO69:
		gpio.IO.PORT	=	GPIO69_PORT;
		gpio.IO.PIN		=	GPIO69_PIN;
		gpio.IO.MODE	=	GPIO69_MODE;
		break;
	case HAL::GPIO::GPIO70:
		gpio.IO.PORT	=	GPIO70_PORT;
		gpio.IO.PIN		=	GPIO70_PIN;
		gpio.IO.MODE	=	GPIO70_MODE;
		break;
	case HAL::GPIO::GPIO71:
		gpio.IO.PORT	=	GPIO71_PORT;
		gpio.IO.PIN		=	GPIO71_PIN;
		gpio.IO.MODE	=	GPIO71_MODE;
		break;
	case HAL::GPIO::GPIO72:
		gpio.IO.PORT	=	GPIO72_PORT;
		gpio.IO.PIN		=	GPIO72_PIN;
		gpio.IO.MODE	=	GPIO72_MODE;
		break;
	default:
		break;
	}

	return gpio;
}

/**
 * @brief Initialize peripheral for a specified GPIO
 * @param id : GPIO ID
 */
static void _hardwareInit (enum GPIO::ID id)
{
	GPIO_InitTypeDef GPIOStruct;
	EXTI_InitTypeDef EXTIStruct;
	NVIC_InitTypeDef NVICStruct;

	GPIO_DEF gpio;

	assert(id < HAL::GPIO::GPIO_MAX);

	gpio = _getGPIOStruct(id);

	// GPIO Init
	GPIOStruct.GPIO_OType 	= 	GPIO_OType_PP;
	GPIOStruct.GPIO_PuPd	=	GPIO_PuPd_NOPULL;
	GPIOStruct.GPIO_Speed	=	GPIO_High_Speed;
	GPIOStruct.GPIO_Pin		=	gpio.IO.PIN;
	GPIOStruct.GPIO_Mode	=	gpio.IO.MODE;

	GPIO_Init(gpio.IO.PORT, &GPIOStruct);

	// INT Init
	if(gpio.IO.MODE == GPIO_Mode_IN)
	{
		// Connect INT Line to GPIO pin
		//SYSCFG_EXTILineConfig(gpio.INT.PORTSOURCE, gpio.INT.PINSOURCE);

		// Init INT
		EXTIStruct.EXTI_Line		= 	gpio.INT.LINE;
		EXTIStruct.EXTI_Trigger		=	(EXTITrigger_TypeDef)gpio.INT.TRIGGER;
		EXTIStruct.EXTI_Mode		=	EXTI_Mode_Interrupt;
		EXTIStruct.EXTI_LineCmd		=	ENABLE;

		//EXTI_ClearITPendingBit(gpio.INT.LINE);

		//EXTI_Init(&EXTIStruct);

		// Init NVIC
		NVICStruct.NVIC_IRQChannel						=	gpio.INT.CHANNEL;
		NVICStruct.NVIC_IRQChannelPreemptionPriority 	= 	gpio.INT.PRIORITY;
		NVICStruct.NVIC_IRQChannelSubPriority 			= 	0;
		NVICStruct.NVIC_IRQChannelCmd					=	ENABLE;

		//NVIC_Init(&NVICStruct);
	}
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/
namespace HAL
{
	GPIO* GPIO::GetInstance (enum GPIO::ID id)
	{
		assert(id < GPIO::GPIO_MAX);

		// if GPIO instance already exists
		if(_gpio[id] != NULL)
		{
			return _gpio[id];
		}
		else
		{
			// Create GPIO instance
			_gpio[id] = new GPIO(id);

			return _gpio[id];
		}
	}

	GPIO::GPIO (enum GPIO::ID id)
	{
		this->id = id;
		this->intState = false;
		this->def = _getGPIOStruct(id);

		_hardwareInit(id);
	}

	enum GPIO::State GPIO::Get ()
	{
		enum GPIO::State state = GPIO::Low;
		BitAction bit = Bit_RESET;

		bit = (BitAction)GPIO_ReadInputDataBit(this->def.IO.PORT, this->def.IO.PIN);

		if(bit == Bit_SET)
		{
			state = GPIO::High;
		}

		return state;
	}

	void GPIO::Set (enum GPIO::State state)
	{
		BitAction bit = Bit_RESET;

		if(this->def.IO.MODE == GPIO_Mode_OUT)
		{
			if(state == GPIO::High)
			{
				bit = Bit_SET;
			}

			GPIO_WriteBit(this->def.IO.PORT, this->def.IO.PIN, bit);
		}
	}

	void GPIO::Toggle ()
	{
		if(this->def.IO.MODE == GPIO_Mode_OUT)
		{
			GPIO_ToggleBits(this->def.IO.PORT, this->def.IO.PIN);
		}
	}

	void GPIO::INTERNAL_InterruptCallback()
	{
		if(this->def.IO.MODE == GPIO_Mode_IN)
		{
			this->StateChanged();
		}
	}
}

/*----------------------------------------------------------------------------*/
/* Interrupt Handler                                                          */
/*----------------------------------------------------------------------------*/

extern "C"
{
	/**
	 * @brief INT Line 9 to 5 Interrupt Handler
	 */
	void INT9_5_IRQHandler(void)
	{
/*		if(EXTI_GetFlagStatus(GPIO0_INT_LINE) == SET)
		{
			EXTI_ClearFlag(GPIO0_INT_LINE);

			GPIO* gpio = GPIO::GetInstance(GPIO::GPIO0);

			gpio->INTERNAL_InterruptCallback();
		}
		else if(EXTI_GetFlagStatus(GPIO3_INT_LINE) == SET)
		{
			EXTI_ClearFlag(GPIO3_INT_LINE);

			GPIO* gpio = GPIO::GetInstance(GPIO::GPIO3);

			gpio->INTERNAL_InterruptCallback();
		}
*/
	}
}
