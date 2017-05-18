/**
 * @file    PositionControl.cpp
 * @author  Jeremy ROULLAND
 * @date    25 dec. 2016
 * @brief   PositionControl class
 *
 *
 */

#include "PositionControlStepper.hpp"
#include "common.h"

#include <stdio.h>

using namespace HAL;
using namespace Utils;
using namespace MotionControl;
using namespace Location;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

// Motion wheel characteristic
#define WD_MM       59.9            // Wheel diameter
#define ADW_MM      129.2           // Axial distance between wheels
#define RATIO       1.315789474     // Gear reduction ratio

#define _PI_        3.14159265358979323846
#define _2_PI_      6.28318530717958647692  // 2*PI


#define PC_MOTOR_LEFT               (Drv8813::ID::DRV8813_4)
#define PC_MOTOR_RIGHT              (Drv8813::ID::DRV8813_1)
#define PC_USTEP                    (1.0*200.0)

#define PC_ANGULAR_VELOCITY         (VelocityControlStepper::ID::ANGULAR)
#define PC_LINEAR_VELOCITY          (VelocityControlStepper::ID::LINEAR)

#define PC_TASK_STACK_SIZE          (512u)
#define PC_TASK_PRIORITY            (configMAX_PRIORITIES-4)

#define PC_TASK_PERIOD_MS           (10u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static MotionControl::PositionControl* _positionControl = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static PC_DEF _getDefStructure (enum PositionControl::ID id)
{
    PC_DEF def;

    assert(id < PositionControl::POSITION_MAX);

    switch(id)
    {
        case PositionControl::ANGULAR:
            def.Motors.ID_left        =    PC_MOTOR_LEFT;
            def.Motors.ID_right       =    PC_MOTOR_RIGHT;
            break;

        case PositionControl::LINEAR:
            def.Motors.ID_left        =    PC_MOTOR_LEFT;
            def.Motors.ID_right       =    PC_MOTOR_RIGHT;
            break;

        default:
            break;
    }

    return def;
}

/*----------------------------------------------------------------------------*/
/* Class Implementation                                                       */
/*----------------------------------------------------------------------------*/

namespace MotionControl
{

    PositionControl* PositionControl::GetInstance(bool standalone)
    {
        // If PositionControl instance already exists
        if(_positionControl != NULL)
        {
            return _positionControl;
        }
        else
        {
            _positionControl = new PositionControl(standalone);
            return _positionControl;
        }
    }

    /**
     * @brief  PositionControl constructor
     */
    PositionControl::PositionControl(bool standalone)
    {
        float32_t currentAngularPosition = 0.0;
        float32_t currentLinearPosition  = 0.0;

        this->name = "PositionControl";
        this->taskHandle = NULL;

        this->status = 0x0000;

        this->Finished = false;

        // Init Angular velocity control
        this->def = _getDefStructure(PositionControl::ANGULAR);
        this->def = _getDefStructure(PositionControl::LINEAR);
        this->leftMotor  = Drv8813::GetInstance(this->def.Motors.ID_left);
        this->rightMotor = Drv8813::GetInstance(this->def.Motors.ID_right);

        this->odometry = Odometry::GetInstance();

        // Get current positions
        currentAngularPosition = odometry->GetAngularPosition();
        currentLinearPosition  = odometry->GetLinearPosition();

        this->angularPosition = currentAngularPosition;
        this->linearPosition  = currentLinearPosition;

        this->angularPositionError = 0.0;
        this->linearPositionError  = 0.0;

        this->enable = true;

        if(standalone)
        {
            // Create task
            xTaskCreate((TaskFunction_t)(&PositionControl::taskHandler),
                        this->name.c_str(),
                        PC_TASK_STACK_SIZE,
                        NULL,
                        PC_TASK_PRIORITY,
                        NULL);
        }
    }

    /**
     * @brief  PositionControl compute
     */
    void PositionControl::Compute(float32_t period)
    {
        //float32_t currentAngularPosition = 0.0;
        //float32_t currentLinearPosition  = 0.0;

        float32_t LeftPosition  = 0.0;
        float32_t RightPosition = 0.0;

        if(this->enable == true)
        {
            this->status |= (1<<0);

            if(this->update == true)
            {
                // Get current positions
                //currentAngularPosition = odometry->GetAngularPosition();
                //currentLinearPosition  = odometry->GetLinearPosition();

                this->angularPositionError = this->angularPosition;// - currentAngularPosition;
                this->linearPositionError  = this->linearPosition;//  - currentLinearPosition;


                // Angular&Linear (radian&meter) to Left&Right (meter&meter)
                LeftPosition  = this->linearPositionError - this->angularPositionError * ADW_MM/1000.0 / 2.0;
                RightPosition = this->linearPositionError + this->angularPositionError * ADW_MM/1000.0 / 2.0;

                // Robot (meter) to Motor (rotation)
                LeftPosition  = LeftPosition  * 1000.0 / (RATIO * WD_MM * _PI_);
                RightPosition = RightPosition * 1000.0 / (RATIO * WD_MM * _PI_);

                LeftPosition  = - LeftPosition;
                RightPosition = + RightPosition;

                if(LeftPosition < 0.0)
                {
                    LeftPosition = -LeftPosition;
                    this->leftMotor->SetDirection(Drv8813State::BACKWARD);
                    this->leftMotor->PulseRotation(LeftPosition*PC_USTEP);
                }
                else if (LeftPosition > 0.0)
                {
                    LeftPosition = +LeftPosition;
                    this->leftMotor->SetDirection(Drv8813State::FORWARD);
                    this->leftMotor->PulseRotation(LeftPosition*PC_USTEP);
                }
                else
                {
                    this->leftMotor->SetDirection(Drv8813State::DISABLED);
                }

                if(RightPosition < 0.0)
                {
                    RightPosition = -RightPosition;
                    this->rightMotor->SetDirection(Drv8813State::BACKWARD);
                    this->rightMotor->PulseRotation(RightPosition*PC_USTEP);
                }
                else if (RightPosition > 0.0)
                {
                    RightPosition = +RightPosition;
                    this->rightMotor->SetDirection(Drv8813State::FORWARD);
                    this->rightMotor->PulseRotation(RightPosition*PC_USTEP);
                }
                else
                {
                    this->rightMotor->SetDirection(Drv8813State::DISABLED);
                }
                this->update = false;
            }
            if(this->leftMotor->IsMoving() || this->rightMotor->IsMoving())
                printf("%d\t%d\r\n", this->leftMotor->IsMoving(), this->rightMotor->IsMoving());
        }
        else
        {
            this->status &= ~(1<<0);
            this->leftMotor->SetDirection(Drv8813State::DISABLED);
            this->rightMotor->SetDirection(Drv8813State::DISABLED);
            this->update = false;
        }

    }

    void PositionControl::taskHandler(void* obj)
    {
        TickType_t xLastWakeTime;
        TickType_t xFrequency = pdMS_TO_TICKS(PC_TASK_PERIOD_MS);

        PositionControl* instance = _positionControl;
        TickType_t prevTick = 0u,  tick = 0u;

        float32_t period = 0.0f;

        // 1. Initialise periodical task
        xLastWakeTime = xTaskGetTickCount();

        // 2. Get tick count
        prevTick = xTaskGetTickCount();

        while(1)
        {
            // 2. Wait until period elapse
            vTaskDelayUntil(&xLastWakeTime, xFrequency);

            // 3. Get tick
            tick = xTaskGetTickCount();

            period = static_cast<float32_t>(tick) -
                     static_cast<float32_t>(prevTick);

            //4. Compute velocity (VelocityControl)
            instance->Compute(period);

            // 5. Set previous tick
            prevTick = tick;
        }
    }

    float32_t PositionControl::getTime()
    {
        float32_t time = 0.0;

        time = static_cast<float32_t>(xTaskGetTickCount());
        time /= 1000.0;

        return  time;
    }


    void PositionControl::Test()
    {
        this->Compute(0);
    }
}
