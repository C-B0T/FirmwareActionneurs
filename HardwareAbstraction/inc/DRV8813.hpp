/**
 * @file	DRV8813.hpp
 * @author	Simon ROUTHIAU
 * @date	03 avr. 2017
 * @brief	DRV8813 Abstration Class
 */

#ifndef INC_DRV8813_HPP_
#define INC_DRV8813_HPP_

#include "stm32f4xx.h"
#include "common.h"

#include "Observable.hpp"
#include "Event.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/
typedef enum Drv8813Mode Drv8813Mode
enum Drv8813Mode
{
	STEPPER_MODE,
	DC_MODE
};

typedef enum Drv8813State Drv8813State
enum Drv8813State
{
	DISABLED,
	FORWARD,
	BACKWARD
};

/**
 * @brief DRV8813 Definition structure
 * Used to define peripheral definition in order to initialize them
 */
typedef struct
{
	// Drv8813 definitions
	struct Drv8813
	{
		Drv8813Mode		MODE;
		uint32_t		USTEP_MODE;
		uint32_t		PWM_FREQ;
		uint32_t		STATE;
		GPIO::ID		GPIO_DECAY;
		GPIO::ID		GPIO_RESET;
		GPIO::ID		GPIO_SLEEP;
		GPIO::ID		GPIO_FAULT;
		GPIO::ID		GPIO_PHA;
		GPIO::ID		GPIO_PHB;
		PWM::ID			GPIO_ENA;
		PWM::ID			GPIO_ENB;
	}DRV8813;
}DRV8813_DEF;

/**
 * @brief DRV8813 GPIO Instances
 * Local GPIO instance
 */
typedef struct
{
	//Drv8813 GPIO instances
	GPIO* 				DECAY;
	GPIO*				RESET;
	GPIO*				SLEEP;
	GPIO*				FAULT;
	GPIO*				PHA;
	GPIO*				PHB;
	PWM*				ENA;
	PWM*				ENB;
}DRV8813_GPIO_INST


/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @namespace HAL
 */
namespace HAL
{
	/**
	 * @class Drv8813
	 * @brief Drv8813 Abstraction Class
	 *
	 * HOWTO:
	 *  - Get a Drv8813 instance with GetInstance() method
	 *  - 
	 */
	class Drv8813
	{
	public:

		/**
		 * @brief Drv8813 identifier list
		 */
		enum ID
		{
			DRV8813_1,  //!< MOTOR1
			DRV8813_2,  //!< MOTOR2
			DRV8813_3,  //!< MOTOR3
			DRV8813_4,  //!< MOTOR4
			DRV8813_5,  //!< MOTOR5
			DRV8813_MAX//!< TIMER_MAX
		};

		/**
		 * @brief Return a Drv8813 instance
		 * @param id : Drv8813 identifier
		 * @return Drv8813 instance
		 */
		static Drv8813* GetInstance (enum ID id);

		/**
		 * @brief Return Drv8813 identifier
		 */
		enum ID GetID()
		{
			return this->id;
		}

		
	private:

		/**
			 * @private
			 * @brief pulse number to execute
			 */
		uint32_t nb_pulse

		/**
		 * @private
		 * @brief Drv8813 constructor
		 * @param id : Drv8813 identifier
		 */
		Drv8813 (enum ID id);

		/**
		 * @private
		 * @brief Drv8813 identifier
		 */
		enum ID id;

		/**
		 * @private
		 * @brief Drv8813 definitions
		 */
		DRV8813_DEF def;

		/**
		 * @private
		 * @brief Drv8813 GPIO instances
		 */
		DRV8813_GPIO_INST GpioInst;

		/**
		 * @private
		 * @brief pulse number to execute
		 */
		uint32_t nb_pulse;

	};
}


#endif /* INC_DRV8813_HPP_ */
