/**
 * @file	I2C.hpp
 * @author	Keivn WYSOCKI
 * @date	6 mars 2017
 * @brief	I2C abstraction class
 */

#ifndef INC_I2C_HPP_
#define INC_I2C_HPP_

#include "stm32f4xx.h"
#include "common.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/**
 * @brief I2C Definition structure
 * Used to define peripheral definition in order to initialize them
 */
typedef struct
{
	// IO definitions
	struct Pin
	{
		GPIO_TypeDef *	PORT;
		uint16_t		PIN;
		uint8_t			PINSOURCE;
		uint8_t			AF;
	}SDA;

	struct Pin SCL;

	// Timer definitions
	struct Timer
	{
		I2C_TypeDef *	I2C;
		uint32_t		CLOCKFREQ;
		uint8_t			SLAVE_ADDR;
	}I2C;
}I2C_DEF;

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

namespace HAL
{
	/**
	 * @brief I2C abstraction class
	 */
	class I2C
	{
	public:

		/**
		 * @brief I2C Identifier list
		 */
		enum ID
		{
			I2C0 = 0,//!< I2C0
			I2C_MAX  //!< I2C_MAX
		};

		/**
		 * @brief Get instance method
		 * @param id : I2C ID
		 * @return I2C instance
		 */
		static I2C* GetInstance (enum ID id);

		/**
		 * @brief Return instance ID
		 */
		enum ID GetID ()
		{
			return this->id;
		}

		/**
		 * @brief Return interrupt state
		 */
		bool GetInterruptState ()
		{
			return this->intState;
		}

		/**
		 * @brief Send data
		 * @param buffer : Data buffer
		 * @param nbBytes : Number of bytes to send
		 * @return Number of bytes sent
		 */
		int32_t Write (uint8_t * buffer, uint32_t nbBytes);

		/**
		 * @brief Read Data
		 * @param buffer : Data buffer
		 * @param nbBytes : Number of bytes to read
		 * @return Number of bytes read
		 */
		int32_t Read (uint8_t * buffer, uint32_t nbBytes);

		/**
		 * @brief Manage whole I2C transfer
		 * @param slaveAddr : 8-bit slave address (R/W bit masked)
		 * @param txBuffer : Transmit buffer address
		 * @param txNbBytes : Number of bytes to transmit
		 * @param rxBuffer : Receive buffer address
		 * @param rxNbBytes : Number of bytes to receive
		 * @return Error code
		 *
		 * First, sends start bit then slave address + R/W bit = '0'. If slave ack, send all
		 * data bytes. If the slave nak, function returns. Write sequence ends with stop bit.
		 * Then, sends start bit plus slave address with R/W bit = '1'. If slave ack, read all
		 * data bytes. Ends with NAK bit and returns.1
		 */
		int32_t Transfer (uint8_t slaveAddr, uint8_t * txBuffer, uint32_t txNbBytes, uint8_t * rxBuffer, uint32_t rxNbBytes);

		/**
		 * @private
		 * @brief Internal interrupt callback. DO NOT CALL !!
		 */
		void INTERNAL_InterruptCallback ();

	private:

		/**
		 * @private
		 * @brief I2C private constructor
		 * @param id : I2C identifier
		 */
		I2C (enum ID id);

		/**
		 * @private
		 * @brief Instance ID
		 */
		enum ID id;

		/**
		 * @private
		 * @brief Interrupt state
		 */
		bool intState;

		/**
		 * @private
		 * @brief Peripheral definition
		 */
		I2C_DEF def;
	};
}

#endif /* INC_I2C_HPP_ */
