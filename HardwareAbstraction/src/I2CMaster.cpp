/**
 * @file	I2CMaster.cpp
 * @author	Kevin
 * @date	25 mai 2017
 * @brief	
 *
 *
 */

#include "I2CMaster.hpp"

#include <FreeRTOS.h>

using namespace std;
using namespace HAL;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

// I2C0
#define I2C0_SCL_PORT			(GPIOA)
#define I2C0_SCL_PIN			(GPIO_Pin_8)
#define I2C0_SCL_PINSOURCE		(GPIO_PinSource8)
#define I2C0_SDA_PORT			(GPIOC)
#define I2C0_SDA_PIN			(GPIO_Pin_9)
#define I2C0_SDA_PINSOURCE		(GPIO_PinSource9)
#define I2C0_IO_AF				(GPIO_AF_I2C3)
#define I2C0_CLOCKFREQ			(100000u)
#define I2C0_SLAVEADDR			(0x10)
#define I2C0_BUS				(I2C3)
#define I2C0_INT_EVENT_CHANNEL	(I2C3_EV_IRQn)
#define I2C0_INT_ERROR_CHANNEL	(I2C3_ER_IRQn)
#define I2C0_INT_PRIORITY		(8u)

#define I2C_TIMEOUT				(0xFFFFu)


/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static I2CMaster* _i2cMaster[I2CMaster::I2C_MASTER_MAX] = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static I2C_DEF _getI2CStruct (enum I2CMaster::ID id)
{
	I2C_DEF i2c;

	assert(id < I2CMaster::I2C_MASTER_MAX);

	switch(id)
	{
	case I2CMaster::I2C_MASTER0:
		// SCL Pin
		i2c.SCL.PORT			=	I2C0_SCL_PORT;
		i2c.SCL.PIN				=	I2C0_SCL_PIN;
		i2c.SCL.PINSOURCE		=	I2C0_SCL_PINSOURCE;
		i2c.SCL.AF				=	I2C0_IO_AF;
		// SDA Pin
		i2c.SDA.PORT			=	I2C0_SDA_PORT;
		i2c.SDA.PIN				=	I2C0_SDA_PIN;
		i2c.SDA.PINSOURCE		=	I2C0_SDA_PINSOURCE;
		i2c.SDA.AF				=	I2C0_IO_AF;
		// I2C Peripheral
		i2c.I2C.BUS				=	I2C0_BUS;
		i2c.I2C.SLAVE_ADDR		=	I2C0_SLAVEADDR;
		i2c.I2C.CLOCKFREQ		=	I2C0_CLOCKFREQ;
		break;

	default:
		break;
	}

	return i2c;
}

static void _hardwareInit (enum I2CMaster::ID id)
{
	GPIO_InitTypeDef GPIOStruct;
	I2C_InitTypeDef I2CStruct;
	NVIC_InitTypeDef NVICStruct;

	I2C_DEF i2c;

	assert(id < I2CMaster::I2C_MASTER_MAX);

	i2c = _getI2CStruct(id);

	// Init SCL and SDA pins
	GPIOStruct.GPIO_Mode	=	GPIO_Mode_AF;
	GPIOStruct.GPIO_OType	=	GPIO_OType_OD;
	GPIOStruct.GPIO_PuPd	=	GPIO_PuPd_NOPULL;
	GPIOStruct.GPIO_Speed	=	GPIO_Speed_100MHz;
	GPIOStruct.GPIO_Pin		=	i2c.SCL.PIN;

	GPIO_PinAFConfig(i2c.SCL.PORT, i2c.SCL.PINSOURCE, i2c.SCL.AF);
	GPIO_Init(i2c.SCL.PORT, &GPIOStruct);

	GPIOStruct.GPIO_Pin		=	i2c.SDA.PIN;

	GPIO_PinAFConfig(i2c.SDA.PORT, i2c.SDA.PINSOURCE, i2c.SDA.AF);
	GPIO_Init(i2c.SDA.PORT, &GPIOStruct);

	// I2C Init
	I2CStruct.I2C_Mode					=	I2C_Mode_I2C;
	I2CStruct.I2C_DutyCycle				=	I2C_DutyCycle_2;
	I2CStruct.I2C_Ack					=	I2C_Ack_Enable;
	I2CStruct.I2C_AcknowledgedAddress	=	I2C_AcknowledgedAddress_7bit;
	I2CStruct.I2C_OwnAddress1			=	i2c.I2C.SLAVE_ADDR;
	I2CStruct.I2C_ClockSpeed			=	i2c.I2C.CLOCKFREQ;

	I2C_Init(i2c.I2C.BUS, &I2CStruct);

	I2C_ITConfig(i2c.I2C.BUS, I2C_IT_EVT /*| I2C_IT_BUF*/ | I2C_IT_ERR, ENABLE);
	//I2C_GeneralCallCmd(i2c.I2C.BUS, ENABLE);
	//I2C_CalculatePEC(i2c.I2C.BUS, ENABLE);
	I2C_Cmd(i2c.I2C.BUS, ENABLE);
}

static ErrorStatus _waitEvent (I2C_TypeDef* i2c, uint32_t event)
{
	int32_t timeout = I2C_TIMEOUT;
	ErrorStatus status = ERROR;

	while(timeout > 0)
	{
		status = I2C_CheckEvent(i2c, event);

		if(status == SUCCESS)
		{
			break;
		}

		timeout--;
	}

	return status;
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace HAL
{
	I2CMaster* I2CMaster::GetInstance(enum I2CMaster::ID id)
	{
		assert(id < I2CMaster::I2C_MASTER_MAX);

		// if I2C instance already exists
		if(_i2cMaster[id] != NULL)
		{
			return _i2cMaster[id];
		}
		else
		{
			_i2cMaster[id] = new I2CMaster(id);

			return _i2cMaster[id];
		}
	}

	I2CMaster::I2CMaster(enum I2CMaster::ID id)
	{
		this->id = id;
		this->error = 0;
		this->def = _getI2CStruct(id);

		_hardwareInit(id);
	}

	int32_t I2CMaster::Write (uint8_t addr, uint8_t nbBytes, uint8_t * buffer)
	{
		uint32_t i = 0u;
		int32_t rval = 0;

		assert(buffer != NULL);
		assert(nbBytes > 0);

		portENTER_CRITICAL();

		I2C_AcknowledgeConfig(this->def.I2C.BUS, ENABLE);

		// 1. Generate Start
		I2C_GenerateSTART(this->def.I2C.BUS, ENABLE);

		// 2. Wait master mode selected event (EV5)
		if((error == NO_ERROR) &&(_waitEvent(this->def.I2C.BUS, I2C_EVENT_MASTER_MODE_SELECT)!= SUCCESS))
		{
			rval = I2C_ERROR_TIMEOUT;
		}

		// 3. Send Address + R/W bit = 0
		if(error == NO_ERROR)
		{
			I2C_Send7bitAddress(this->def.I2C.BUS, addr, I2C_Direction_Transmitter);
		}

		// 4. Wait address match event (EV6)
		if((error == NO_ERROR) &&(_waitEvent(this->def.I2C.BUS, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)!= SUCCESS))
		{
			rval = I2C_ERROR_TIMEOUT;
		}

		// 5. Send data
		if(rval == NO_ERROR)
		{
			for(i=0u; i < nbBytes; i++)
			{
				// Send a byte
				I2C_SendData(this->def.I2C.BUS, buffer[i]);

				// Wait byte transmitted event
				if(_waitEvent(this->def.I2C.BUS, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
				{
					rval = I2C_ERROR_SLAVE_SEND_DATA_FAILED;
					break;
				}
			}

			rval = i;
		}

		// 6. Generate stop
		I2C_GenerateSTOP(this->def.I2C.BUS, ENABLE);

		portEXIT_CRITICAL();

		return rval;
	}

	int32_t I2CMaster::Read (uint8_t addr, uint8_t nbBytes, uint8_t * buffer)
	{
		uint32_t i = 0u;
		int32_t rval = 0;

		assert(buffer != NULL);
		assert(nbBytes > 0);

		portENTER_CRITICAL();

		I2C_AcknowledgeConfig(this->def.I2C.BUS, ENABLE);

		// 1. Generate Start
		I2C_GenerateSTART(this->def.I2C.BUS, ENABLE);

		// 2. Wait master mode selected event (EV5)
		if((error == NO_ERROR) &&(_waitEvent(this->def.I2C.BUS, I2C_EVENT_MASTER_MODE_SELECT)!= SUCCESS))
		{
			rval = I2C_ERROR_TIMEOUT;
		}

		// 3. Send Address + R/W bit = 1
		if(error == NO_ERROR)
		{
			I2C_Send7bitAddress(this->def.I2C.BUS, addr, I2C_Direction_Receiver);
		}

		// 4. Wait address match event (EV6)
		if((error == NO_ERROR) &&(_waitEvent(this->def.I2C.BUS, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)!= SUCCESS))
		{
			rval = I2C_ERROR_TIMEOUT;
		}

		// 5. Send data
		if(rval == NO_ERROR)
		{
			for(i=0u; i < nbBytes; i++)
			{
				// Wait byte received event
				if(_waitEvent(this->def.I2C.BUS, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS)
				{
					rval = I2C_ERROR_SLAVE_SEND_DATA_FAILED;
					break;
				}

				// Read byte
				buffer[i] = I2C_ReceiveData(this->def.I2C.BUS);

				// Last byte, send NAK
				if(i == (nbBytes - 2u))
				{
					I2C_AcknowledgeConfig(this->def.I2C.BUS, DISABLE);
				}
			}

			rval = i;
		}

		// 6. Generate stop
		I2C_GenerateSTOP(this->def.I2C.BUS, ENABLE);

		portEXIT_CRITICAL();

		return rval;
	}
}
