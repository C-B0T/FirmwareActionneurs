/**
 * @author    Jeremy ROULLAND
 * @file      Mandible.cpp
 * @date      24 may 2017
 * @brief
 */

#include "Mandible.hpp"

#include "FreeRTOS.h"
#include "task.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

// MANDIBLE_1
#define MANDIBLE1_HIGH_SIDE_GPIO    (HAL::GPIO::ID::GPIO36)
#define MANDIBLE1_LOW_SIDE_GPIO     (HAL::GPIO::ID::GPIO37)
#define MANDIBLE1_POWER_PWM         (HAL::PWM::ID::PWM15)


/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static Mandible* _instance[Mandible::ID::MANDIBLE_MAX] = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

 /**
  * @brief
  * @param id :  ID
  * @return MAN_DEF structure
  */
static MAN_DEF _getMANStruct (enum Mandible::ID id)
{
     MAN_DEF def;

     assert(id < Mandible::MANDIBLE_MAX);

     switch(id)
     {
     case Mandible::ID::MANDIBLE_1:
        def.highSideGpioId = MANDIBLE1_HIGH_SIDE_GPIO;
        def.lowSideGpioId  = MANDIBLE1_LOW_SIDE_GPIO;
        def.PowerPwmId     = MANDIBLE1_POWER_PWM;
        break;

     default:
         break;
     }

     return def;
}

/**
* @brief Initialize peripheral for a specific Encoder
* @param id : Encoder ID
*/
static void _hardwareInit (enum Mandible::ID id)
{

}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

Mandible* Mandible::GetInstance(Mandible::ID id)
{
    assert(id < Mandible::ID::MANDIBLE_MAX);

    if(_instance[id] == NULL)
    {
        _instance[id] = new Mandible(id);
    }

    return _instance[id];
}

Mandible::Mandible(Mandible::ID id)
{
    this->id = id;
    this->def = _getMANStruct(id);

    this->highSide = HAL::GPIO::GetInstance(this->def.highSideGpioId);
    this->lowSide  = HAL::GPIO::GetInstance(this->def.lowSideGpioId);
    this->power    = HAL::PWM::GetInstance(this->def.PowerPwmId);

    this->highSide->Set(HAL::GPIO::State::Low);
    this->lowSide->Set(HAL::GPIO::State::Low);
    this->power->SetDutyCycle(0.0f);

    this->pos = Bottom;

    //_hardwareInit(id);
}

void Mandible::SetPosition(Mandible::Position pos)
{
    if (pos == Top)
    {
        if (this->pos != Top)
        {
            this->lowSide->Set(HAL::GPIO::State::High);
            this->power->SetDutyCycle(0.1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            this->lowSide->Set(HAL::GPIO::State::Low);
            this->pos = pos;
        }
    }
    else if (pos == Bottom)
    {
        if (this->pos != Bottom)
        {
            this->highSide->Set(HAL::GPIO::State::High);
            this->power->SetDutyCycle(0.1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            this->highSide->Set(HAL::GPIO::State::Low);
            this->pos = pos;
        }
    }
    else if (pos == Middle)
    {
        if (this->pos == Bottom)
        {
            this->lowSide->Set(HAL::GPIO::State::High);
            this->power->SetDutyCycle(0.1);
            vTaskDelay(pdMS_TO_TICKS(500));
            this->lowSide->Set(HAL::GPIO::State::Low);
            this->pos = pos;
        }
        else if (this->pos == Top)
        {
            this->highSide->Set(HAL::GPIO::State::High);
            this->power->SetDutyCycle(0.1);
            vTaskDelay(pdMS_TO_TICKS(500));
            this->highSide->Set(HAL::GPIO::State::Low);
            this->pos = pos;
        }
    }
}

void Mandible::SetPosition(float32_t percent)
{
    if (percent < 0.0)
        percent = 0.0;
    else if (percent > 1.0)
        percent = 1.0;

    this->highSide->Set(HAL::GPIO::State::High);
    this->power->SetDutyCycle(0.1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    this->highSide->Set(HAL::GPIO::State::Low);

}
