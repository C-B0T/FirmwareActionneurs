/**
 * @file	GPIO.hpp
 * @author	Kevin WYSOCKI
 * @date	8 nov. 2016
 * @brief	GPIO Abstraction Class
 */

#ifndef INC_GPIO_HPP_
#define INC_GPIO_HPP_

#include "stm32f4xx.h"

#include "Event.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/**
 * @brief GPIO Definition structure
 * Used to define GPIO port, pin, mode and, in case of input,
 * specific necessary definitions to initialize external interrupt
 */
typedef struct
{
	struct Pin
	{
		GPIO_TypeDef * 		PORT;		/**< GPIO Port */
		uint16_t			PIN;		/**< GPIO Pin number */
		GPIOMode_TypeDef	MODE;		/**< GPIO Mode */
	}IO;

	// Interrupt definitions - INPUTS ONLY
	struct Interrupt
	{
		uint8_t 	PORTSOURCE;		/**< Interrupt Port */
		uint8_t		PINSOURCE;		/**< Interrupt Pin */
		uint32_t	LINE;			/**< Interrupt Line */
		uint32_t	TRIGGER;		/**< Interrupt trigger */
		uint8_t		PRIORITY;		/**< Interrupt priority, 0 to 15, 0 is the highest priority */
		uint8_t		CHANNEL;		/**< Interrupt IRQ Channel */
	}INT;
}GPIO_DEF;

/*----------------------------------------------------------------------------*/
/* Class										                              */
/*----------------------------------------------------------------------------*/
/**
 * @namespace HAL
 */
namespace HAL
{
	/**
	 * @class GPIO
	 * @brief GPIO Abstraction class
	 *
	 * HOWTO :
	 *	- Get GPIO instance with GPIO::GetInstance()
	 *	- Get() and Set() can be use to retrieve or set GPIO state
	 *	- InterruptCallback can be used to set a function called when interrupt is raised
	 */
	class GPIO
	{
	public:

		/**
		 * @brief GPIO Identifier list
		 */
		enum ID
		{
			GPIO0,		//!< LED1
			GPIO1,		//!< LED2
			GPIO2,		//!< LED3
			GPIO3,		//!< LED4
			GPIO4,		//!< MES_BAT
			GPIO5,		//!< EN_5V
			GPIO6,		//!< MOT_DECAY
			GPIO7,		//!< MOT_RESET
			GPIO8,		//!< MOT_SLEEP
			GPIO9,		//!< MOT1_FAULT
			GPIO10,		//!< MOT1_APH
			GPIO11,		//!< MOT1_BPH
			GPIO12,		//!< MOT2_FAULT
			GPIO13,		//!< MOT2_APH
			GPIO14,		//!< MOT2_BPH
			GPIO15,		//!< MOT3_FAULT
			GPIO16,		//!< MOT3_APH
			GPIO17,		//!< MOT3_BPH
			GPIO18,		//!< MOT4_FAULT
			GPIO19,		//!< MOT4_APH
			GPIO20,		//!< MOT4_BPH
			GPIO21,		//!< MOT5_FAULT
			GPIO22,		//!< MOT5_APH
			GPIO23,		//!< MOT5_BPH
			GPIO24,		//!< SEL0
			GPIO25,		//!< SEL1
			GPIO26,		//!< SEL2
			GPIO27,		//!< SEL3
			GPIO28,		//!< SEN1
			GPIO29,		//!< SEN2
			GPIO30,		//!< FAULT1
			GPIO31,		//!< FAULT2
			GPIO32,		//!< OUT1
			GPIO33,		//!< OUT2
			GPIO34,		//!< OUT3
			GPIO35,		//!< OUT4
			GPIO36,		//!< OUT5
			GPIO37,		//!< OUT6
			GPIO38,		//!< OUT7
			GPIO39,		//!< OUT8
			GPIO40,		//!< OUT_SENS
			GPIO41,		//!< SERVO1
			GPIO42,		//!< SERVO2
			GPIO43,		//!< SERVO3
			GPIO44,		//!< SERVO4
			GPIO45,		//!< SERVO5
			GPIO46,		//!< SERVO6
			GPIO47,		//!< SERVO7
			GPIO48,		//!< SERVO8
			GPIO49,		//!< SERVO9
			GPIO50,		//!< SERVO10
			GPIO51,		//!< SERVO11
			GPIO52,		//!< SERVO12
			GPIO53,		//!< SERVO13
			GPIO54,		//!< SERVO14
			GPIO55,		//!< SERVO15
			GPIO56,		//!< SERVO16
			GPIO57,		//!< INPUT1
			GPIO58,		//!< INPUT2
			GPIO59,		//!< INPUT3
			GPIO60,		//!< INPUT4
			GPIO61,		//!< INPUT5
			GPIO62,		//!< INPUT6
			GPIO63,		//!< INPUT7
			GPIO64,		//!< INPUT8
			GPIO65,		//!< INPUT9
			GPIO66,		//!< INPUT10
			GPIO67,		//!< INPUT11
			GPIO68,		//!< INPUT12
			GPIO69,		//!< INPUT13
			GPIO70,		//!< INPUT14
			GPIO71,		//!< INPUT15
			GPIO72,		//!< INPUT16
			GPIO_MAX
		};

		/**
		 * @brief GPIO State
		 */
		enum State
		{
			Low = 0,//!< Logic '0'
			High    //!< Logic '1'
		};

		/**
		 * @brief Get instance method
		 * @param id : GPIO ID
		 * @return GPIO instance
		 */
		static GPIO* GetInstance (enum ID id);

		/**
		 * @brief Return GPIO ID
		 * @return ID
		 */
		enum ID GetID()
		{
			return this->id;
		}

		/**
		 * @brief Return GPIO interrupt state
		 * @return true if interrupt is enabled, false else
		 */
		bool GetInterruptState ()
		{
			return this->intState;
		}

		/**
		 * @brief Return GPIO state
		 * @return Low if GPIO is '0' logic, High else
		 */
		enum State Get();

		/**
		 * @brief Set GPIO state (if GPIO is an output)
		 * @param state : Low if GPIO must be '0' logic, High else
		 */
		void Set(enum State state);

		/**
		 * @brief Toggle GPIO (if GPIO is an output)
		 */
		void Toggle();

		/**
		 * @brief State changed event
		 * INPUT ONLY !
		 */
		Utils::Event StateChanged;

		/**
		 * @private
		 * @brief Internal interrupt callback. DO NOT CALL !!
		 */
		void INTERNAL_InterruptCallback ();

	private:
		/**
		 * @private
		 * @brief GPIO private constructor
		 * @param id : GPIO ID
		 */
		GPIO (enum ID id);

		/**
		 * @private
		 * @brief GPIO ID
		 */
		enum ID id;

		/**
		 * @private
		 * @brief Interrupt state
		 */
		bool intState;

		/**
		 * @private
		 * @brief GPIO definition
		 */
		GPIO_DEF def;
	};
}
#endif /* INC_GPIO_HPP_ */
