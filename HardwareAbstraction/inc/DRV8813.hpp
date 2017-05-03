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
 * @brief Steps PWM structure
 * Used to define PWM duty cycle for each step in the mode
 */
typedef struct
{
		bool 			PositivA		//true= positive current, false = negative current
		bool			PositivB		//true= positive current, false = negative current
		uint32_t		PWM_A;			//in %
		uint32_t		PWM_B;			//in %
}PWM_STEP_DEF;

/**
 * @brief DRV8813 Definition structure
 * Used to define peripheral definition in order to initialize them
 */
typedef struct
{
	// Drv8813 definitions
		Drv8813Mode		MODE;
		uint32_t		USTEP_MODE;
		uint32_t		PWM_FREQ;
		uint32_t		NB_MOTOR_STEP;
		GPIO::ID		GPIO_DECAY;
		GPIO::ID		GPIO_RESET;
		GPIO::ID		GPIO_SLEEP;
		GPIO::ID		GPIO_FAULT;
		GPIO::ID		GPIO_PHA;
		GPIO::ID		GPIO_PHB;
		PWM::ID			GPIO_ENA;
		PWM::ID			GPIO_ENB;
		uint32_t		CURRENT_COEF;	//current coef for pwm (0 to 100%)
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
		 * @brief Set speed
		 * @param speed: in step/s
		 * @return 0 if OK, else speed is out of range
		 */
		uint32_t SetSpeed (uint32_t speed);

		/**
		 * @brief Set direction
		 * @param dir: motor's direction DISABLE, FORWARD, BACKWARD
		 */
		void SetDirection (Drv8813State dir);

		/**
		 * @brief Start rotation
		 */
		void Start (void);

		/**
		 * @brief Stop rotation
		 */
		void Stop (void);

		/**
		 * @brief rotation of step number
		 * @param pulse: number of step
		 */
		void PulseRotation (uint32_t pulse);

		/**
		 * @brief read position in step
		 * @return position in step
		 */
		uint32_t ReadPosition (void);

		/**
		 * @brief set position in step
		 * @param pos: in step
		 * @return 0 if OK, else position is out of range
		*/
		uint32_t SetPosition (uint32_t pos);

		/**
		 * @brief Return Drv8813 identifier
		 */
		enum ID GetID()
		{
			return this->id;
		}

		/**
		 * @private
		 * @brief timer source for tick stepper
		 */
		static Timer tim;
		
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
		 * @brief enable/forward/backward status
		 */
		Drv8813State direction;

		/**
		 * @private
		 * @brief speed of movment (nb tick between two step)
		 */
		uint32_t period_tick;

		/**
		 * @private
		 * @brief index in step16
		 */
		uint32_t stepIndex;

		/**
		 * @private
		 * @brief speed of movment (PWM)
		 */
		uint32_t pwm1;

		/**
		 * @private
		 * @brief speed of movment (PWM)
		 */
		uint32_t pwm2;

		/**
		 * @private
		 * @brief pulse number to execute
		 */
		uint32_t nb_pulse;

		/**
		 * @private
		 * @brief position of the rotation in step
		 */
		uint32_t position;

		/**
		 * @private
		 * @brief run rotation motor
		 */
		uint32_t run;

	};
}


#endif /* INC_DRV8813_HPP_ */
