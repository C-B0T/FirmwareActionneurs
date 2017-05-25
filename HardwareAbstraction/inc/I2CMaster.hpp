/**
 * @file	I2CMaster.hpp
 * @author	Kevin
 * @date	25 mai 2017
 * @brief	
 *
 *
 */

#ifndef INC_I2CMASTER_HPP_
#define INC_I2CMASTER_HPP_

#include "I2CCommon.h"
#include "Event.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

namespace HAL
{
	/**
	 * @brief I2CMaster abstraction class
	 */
	class I2CMaster
	{
	public:

		/**
		 * @brief I2C Identifier list
		 */
		enum ID
		{
			I2C_MASTER0 = 0,//!< I2C0
			I2C_MASTER_MAX  //!< I2C_MAX
		};

		/**
		 * @brief Get instance method
		 * @param id : I2C ID
		 * @return I2C instance
		 */
		static I2CMaster* GetInstance (enum ID id);

		/**
		 * @brief Return instance ID
		 */
		enum ID GetID ()
		{
			return this->id;
		}

		/**
		 * @brief Return current error (= 0 if no error, < 0 else)
		 */
		int32_t GetError ()
		{
			int32_t error = this->error;

			this->error = 0;

			return error;
		}

		/**
		 * @brief Send frame
		 * @param frame : I2C Frame
		 * @return Number of bytes sent or error code (if return value < 0)
		 */
		int32_t Write (uint8_t addr, uint8_t nbBytes, uint8_t * buffer);

		/**
		 * @brief Read incoming frame
		 * @param frame : Buffered I2C Frame
		 * @return Error code (if < 0)
		 */
		int32_t Read (uint8_t addr, uint8_t nbBytes, uint8_t * buffer);


	private:

		/**
		 * @private
		 * @brief I2C private constructor
		 * @param id : I2C identifier
		 */
		I2CMaster (enum ID id);

		/**
		 * @private
		 * @brief Instance ID
		 */
		enum ID id;

		int32_t error;

		/**
		 * @private
		 * @brief Peripheral definition
		 */
		I2C_DEF def;
	};
}

#endif /* INC_I2CMASTER_HPP_ */
