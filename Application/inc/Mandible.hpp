/**
 * @author    Jeremy ROULLAND
 * @file      Mandible.hpp
 * @date      24 may 2017
 * @brief
 */

#ifndef INC_MANDIBLE_HPP_
#define INC_MANDIBLE_HPP_

/**
 * @namespace
 * @brief
 *
 *
 */

 #include "common.h"

 #include "GPIO.hpp"
 #include "PWM.hpp"

 /*----------------------------------------------------------------------------*/
 /* Definitions                                                                */
 /*----------------------------------------------------------------------------*/


 #define ADC_ERROR_BUSY      (-1)

typedef struct
{
    HAL::GPIO::ID   highSideGpioId;
    HAL::GPIO::ID   lowSideGpioId;
    HAL::PWM::ID    PowerPwmId;
}MAN_DEF;


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
class Mandible
{
public:
    enum ID
    {
        MANDIBLE_1 = 0,
        MANDIBLE_MAX
    };

    enum Position
    {
        Top = 0,
        Bottom,
        Middle,
        Position_MAX
    };

    static Mandible* GetInstance (enum ID id);

    void SetPosition(enum Position pos);

private:
    Mandible (enum ID id);

    void SetPosition(float32_t percent); // 0.0=Low 1.0=High


    enum ID id;

    enum Position pos;

    HAL::GPIO* highSide;
    HAL::GPIO* lowSide;
    HAL::PWM*  power;

    MAN_DEF def;
};

#endif

