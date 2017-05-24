/**
 * @file    Cylinder.cpp
 * @author  Jeremy ROULLAND
 * @date    20 may 2017
 * @brief   Cylinder (Barillet) class
 */

#include <stddef.h>
#include "Cylinder.hpp"

#include "FreeRTOS.h"
#include "task.h"


/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

// CYLINDER BOTTOM
#define CYL0_MOTOR      (HAL::Drv8813::ID::DRV8813_2)
//#define CYL0_MOTOR      (HAL::Drv8813::ID::DRV8813_1)
#define CYL0_MOTORRISE  (HAL::Drv8813::ID::DRV8813_5)
#define CYL0_MOTOROPEN1 (HAL::PWM::PWM14)
#define CYL0_MOTOROPEN2 (HAL::PWM::PWM15)
#define CYL0_TOPZ       (HAL::GPIO::ID::GPIO72)
#define CYL0_SHORTPATH  (true)
#define CYL0_CANRISE    (true)
#define CYL0_INDEX_MAX  (9u)
#define CYL0_RATIO      ((5.89f*400.0f)/(CYL0_INDEX_MAX+1u))   // NbStep pour 1 tour barillet
#define CYL0_SPEED      (200u)          // Step/sec

// CYLINDER TOP
#define CYL1_MOTOR      (HAL::Drv8813::ID::DRV8813_3)
//#define CYL1_MOTOR      (HAL::Drv8813::ID::DRV8813_1)
#define CYL1_TOPZ       (HAL::GPIO::ID::GPIO58)
#define CYL1_INDEX_MAX   (5u)
#define CYL1_SHORTPATH  (false)
#define CYL1_CANRISE    (false)
#define CYL1_RATIO      ((2.49f*400.0f)/(CYL1_INDEX_MAX+1u))   // NbStep pour 1 tour barillet
#define CYL1_SPEED      (100u)      // Step/sec


/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

Cylinder* _cylinder[Cylinder::CYLINDER_MAX] = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @brief Retrieve Encoder definitions from Encoder ID
 * @param id : Encoder ID
 * @return ENC_DEF structure
 */
static CYL_DEF _getCylStruct (enum Cylinder::ID id)
{
    CYL_DEF cyl;

	assert(id < Cylinder::CYLINDER_MAX);

    switch(id)
    {
    case Cylinder::ID::CYLINDER0:
        cyl.ID_motor        = CYL0_MOTOR;
        cyl.ID_motorRise    = CYL0_MOTORRISE;
        cyl.ID_motorOpen[0] = CYL0_MOTOROPEN1;
        cyl.ID_motorOpen[1] = CYL0_MOTOROPEN2;
        cyl.ID_topz         = CYL0_TOPZ;
        cyl.indexMax        = CYL0_INDEX_MAX;
        cyl.shortPath       = CYL0_SHORTPATH;
        cyl.canRise         = CYL0_CANRISE;
        cyl.ratio           = CYL0_RATIO;
        cyl.speed           = CYL0_SPEED;
        break;

    case Cylinder::ID::CYLINDER1:
        cyl.ID_motor     = CYL1_MOTOR;
        cyl.ID_topz      = CYL1_TOPZ;
        cyl.indexMax     = CYL1_INDEX_MAX;
        cyl.shortPath    = CYL1_SHORTPATH;
        cyl.canRise      = CYL1_CANRISE;
        cyl.ratio        = CYL1_RATIO;
        cyl.speed        = CYL1_SPEED;
        break;

    default:
        break;
    }

    return cyl;
}


/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

Cylinder* Cylinder::GetInstance (Cylinder::ID id)
{
    assert(id < Cylinder::ID::CYLINDER_MAX);

    // if cylinder instance already exists
    if(_cylinder[id] != NULL)
    {
        return _cylinder[id];
    }
    else
    {
        // Create cylinder instance
        _cylinder[id] = new Cylinder(id);
        return _cylinder[id];
    }
}

Cylinder::Cylinder (Cylinder::ID id)
{
    this->def = _getCylStruct(id);

    this->id = id;

    this->motor = HAL::Drv8813::GetInstance(this->def.ID_motor);
    this->topz  = HAL::GPIO::GetInstance(this->def.ID_topz);
    this->index = 0;

    if(id == Cylinder::ID::CYLINDER0)
    {
        this->motorOpen[0] = HAL::PWM::GetInstance(this->def.ID_motorOpen[0]);
        this->motorOpen[1] = HAL::PWM::GetInstance(this->def.ID_motorOpen[1]);
        this->motorOpen[0]->SetState(HAL::PWM::State::ENABLED);
        this->motorOpen[1]->SetState(HAL::PWM::State::ENABLED);

        if(this->def.canRise == true)
            this->motorRise = HAL::Drv8813::GetInstance(this->def.ID_motorRise);
    }
}

int8_t Cylinder::Open()
{
    if(this->id == Cylinder::CYLINDER0)
    {
        this->motorOpen[0]->SetDutyCycle(0);
        this->motorOpen[1]->SetDutyCycle(0);

        vTaskDelay(1000);

        this->motorOpen[0]->SetDutyCycle(0);
        this->motorOpen[1]->SetDutyCycle(0);
    }
    return 0;
}

int8_t Cylinder::Close()
{
    if(this->id == Cylinder::CYLINDER0)
    {

    }
    return 0;
}

int8_t Cylinder::Raise()
{
    if(this->def.canRise == false)
        return -1;

    this->motorRise->SetDirection(HAL::Drv8813State_t::BACKWARD);
    this->motorRise->SetSpeedStep(200);
    this->motorRise->PulseRotation(10*200);

    while(this->motorRise->IsMoving()) vTaskDelay(200);

    this->motorRise->SetDirection(HAL::Drv8813State_t::DISABLED);

    return 0;

}

int8_t Cylinder::Lower()
{
    if(this->def.canRise == false)
        return -1;

    this->motorRise->SetDirection(HAL::Drv8813State_t::FORWARD);
    this->motorRise->SetSpeedStep(200);
    this->motorRise->PulseRotation(10*200);

    while(this->motorRise->IsMoving()) vTaskDelay(200);

    this->motorRise->SetDirection(HAL::Drv8813State_t::DISABLED);

    return 0;
}

int8_t Cylinder::Goto(int8_t index)
{
    int8_t nbIndex = 0;

    // Error cases
    if(index > this->def.indexMax)
        return -1;
    if(index < -this->def.indexMax)
        return -2;

    // Nominal cases
    if(this->motor->IsMoving())
        return 0;

    nbIndex = index - this->index;

    // Calculate shorted path
    if(this->def.shortPath)
    {
        if(nbIndex > this->def.indexMax/2)
            nbIndex -= this->def.indexMax + 1u;
        if(nbIndex < -this->def.indexMax/2)
            nbIndex += this->def.indexMax + 1u;
    }

    if(nbIndex > 0)
        this->motor->SetDirection(HAL::Drv8813State_t::FORWARD);
    else if(nbIndex < 0)
        this->motor->SetDirection(HAL::Drv8813State_t::BACKWARD);

    this->motor->SetSpeedStep(this->def.speed);
    this->motor->PulseRotation(this->def.ratio * abs(nbIndex));

    this->index = index;

    return nbIndex;
}

int8_t Cylinder::SearchRefPoint()
{
    this->motor->SetDirection(HAL::Drv8813State_t::FORWARD);
    this->motor->SetSpeedStep(this->def.speed);
    this->motor->PulseRotation(this->def.ratio * (this->def.indexMax+1u));

    while(this->motor->IsMoving())
    {
        //if(this->topz->Get() == HAL::GPIO::State::Low)
        if(this->topz->Get() == HAL::GPIO::State::High)
        {
            this->motor->SetDirection(HAL::Drv8813State_t::DISABLED);
            this->index = 0;
            return 1;   // Found
        }
    }

    this->motor->SetDirection(HAL::Drv8813State_t::DISABLED);
    return 0;  // Not Found
}

bool Cylinder::IsPositioningFinished ()
{
    return !this->motor->IsMoving();
}
