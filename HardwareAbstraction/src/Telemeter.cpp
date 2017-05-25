/**
 * @author    Jeremy ROULLAND
 * @file      Telemeter.cpp
 * @date      25 may 2017
 * @brief
 */

#include "Telemeter.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

// TELEMETER_1
#define TEL1_CHANNEL    (HAL::ADConverter::ADC_Channel1)

// TELEMETER_2
#define TEL2_CHANNEL    (HAL::ADConverter::ADC_Channel2)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static HAL::Telemeter* _instance[HAL::Telemeter::ID::TELEMETER_MAX] = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @brief
 * @param id :  ID
 * @return MAN_DEF structure
 */
static TEL_DEF _getTELStruct (enum HAL::Telemeter::ID id)
{
     TEL_DEF def;

     assert(id < HAL::Telemeter::TELEMETER_MAX);

     switch(id)
     {
     case HAL::Telemeter::TELEMETER_1:
        def.ch = TEL1_CHANNEL;
        break;

     case HAL::Telemeter::TELEMETER_2:
        def.ch = TEL2_CHANNEL;
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
static void _hardwareInit (enum HAL::Telemeter::ID id)
{

}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace HAL {

    Telemeter* Telemeter::GetInstance(Telemeter::ID id)
    {
        assert(id < Telemeter::ID::TELEMETER_MAX);

        if(_instance[id] == NULL)
        {
            _instance[id] = new Telemeter(id);
        }

        return _instance[id];
    }

    Telemeter::Telemeter(Telemeter::ID id)
    {
        this->id = id;
        this->def = _getTELStruct(id);

        this->adc = ADConverter::GetInstance(this->def.ch);

        //_hardwareInit(id);
    }

    uint16_t Telemeter::GetValue()
    {
        uint16_t val;

        this->adc->StartConv();
        this->adc->WaitWhileBusy();

        val = this->adc->GetResult();

        return val;
    }

    bool Telemeter::Detect()
    {
        bool ret;
        uint16_t cpt = 0;
        uint16_t val = 0;

        for(uint16_t i=0; i < 5; i++)
        {
            val = this->GetValue();

            if(val > 2000)
                cpt++;
        }

        if(cpt >= 3)
            ret = true;
        else
            ret = false;

        return ret;
    }
}
