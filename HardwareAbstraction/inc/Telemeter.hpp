/**
 * @file      Telemeter.hpp
 * @author    Jeremy ROULLAND
 * @date      25 may 2017
 * @brief
 */

#ifndef INC_TELEMETER_HPP_
#define INC_TELEMETER_HPP_

/**
 * @namespace
 * @brief
 *
 *
 */

 #include "common.h"

#include "ADConverter.hpp"

 /*----------------------------------------------------------------------------*/
 /* Definitions                                                                */
 /*----------------------------------------------------------------------------*/

typedef struct
{
    HAL::ADConverter::Channel ch;
}TEL_DEF;


 /*----------------------------------------------------------------------------*/
 /* Class declaration	                                                      */
 /*----------------------------------------------------------------------------*/

/**
* @class
* @brief
*
* HOWTO :
* -
*
*/
namespace HAL {

class Telemeter
{
public:
    enum ID
    {
        TELEMETER_1 = 0,
        TELEMETER_2,
        TELEMETER_MAX
    };

    static Telemeter* GetInstance (enum ID id);

    uint16_t GetValue();

    bool Detect();

private:
    Telemeter (enum ID id);

    enum ID id;

    HAL::ADConverter* adc;

    TEL_DEF def;
};

}

#endif /* INC_TELEMETER_HPP_ */
