/**
 * @file	ADConverter.hpp
 * @author	Kevin WYSOCKI
 * @date	23 mai 2017
 * @brief	A/D Converter class
 */

#ifndef INC_ADCONVERTER_HPP_
#define INC_ADCONVERTER_HPP_

#include "common.h"
#include "Event.hpp"
#include "stm32f4xx.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define ADC_ERROR_BUSY      (-1)


typedef struct
{
    struct
    {
        GPIO_TypeDef *  PORT;
        uint16_t        PIN;
    }Input;

    struct
    {
        ADC_TypeDef *   ADConverter;
        uint8_t         Channel;
    }ADConverter;
}ADC_DEF;

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

namespace HAL
{
    class ADConverter
    {
    public:

        enum Channel
        {
            ADC_Channel0    =   0,
            ADC_Channel1,
            ADC_Channel2,
            ADC_ChannelMAX

        };

        ADConverter* GetInstance (ADConverter::Channel channel);

        int32_t StartConv ();

        void WaitWhileBusy();

        uint16_t GetResult();

    private:

        ADConverter(ADConverter::Channel channel);

        uint16_t result;

        ADC_DEF def;
    };
}

#endif /* INC_ADCONVERTER_HPP_ */
