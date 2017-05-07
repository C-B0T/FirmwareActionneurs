/**
 * @file    VelocityControlStepper.cpp
 * @author  Jeremy ROULLAND
 * @date    20 dec. 2016
 * @brief   VelocityControlStepper class
 *
 *
 */

#include "VelocityControlStepper.hpp"

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
#define RATIO       1.32            // Gear reduction ratio

#define _PI_        3.14159265358979323846
#define _2_PI_      6.28318530717958647692  // 2*PI


#define VC_MOTOR_LEFT               (Drv8813::ID::DRV8813_1)
#define VC_MOTOR_RIGHT              (Drv8813::ID::DRV8813_2)

#define VC_TASK_STACK_SIZE          (256u)
#define VC_TASK_PRIORITY            (configMAX_PRIORITIES-3)

#define VC_TASK_PERIOD_MS           (5u)



/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static MotionControl::VelocityControlStepper* _velocityControlStepper = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static VC_DEF _getDefStructure (enum VelocityControlStepper::ID id)
{
    VC_DEF def;

    assert(id < VelocityControlStepper::VELOCITY_MAX);

    switch(id)
    {
        case VelocityControlStepper::ANGULAR:
            def.Motors.ID_left        =    VC_MOTOR_LEFT;
            def.Motors.ID_right       =    VC_MOTOR_RIGHT;
            break;

        case VelocityControlStepper::LINEAR:
            def.Motors.ID_left       =    VC_MOTOR_LEFT;
            def.Motors.ID_right      =    VC_MOTOR_RIGHT;
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
    VelocityControlStepper* VelocityControlStepper::GetInstance(bool standalone)
    {
        // If VelocityControlStepper instance already exists
        if(_velocityControlStepper != NULL)
        {
            return _velocityControlStepper;
        }
        else
        {
            _velocityControlStepper = new VelocityControlStepper(standalone);
            return _velocityControlStepper;
        }
    }

    /**
     * @brief  VelocityControlStepper constructor
     */
    VelocityControlStepper::VelocityControlStepper(bool standalone)
    {
        this->name = "VelocityControlStepper";
        this->taskHandle = NULL;

        this->status = 0x0000;

        // Init Angular velocity control
        this->def = _getDefStructure(VelocityControlStepper::ANGULAR);

        this->odometry = Odometry::GetInstance();

        this->leftMotor  = Drv8813::GetInstance(this->def.Motors.ID_left);
        this->rightMotor = Drv8813::GetInstance(this->def.Motors.ID_right);

        this->angularVelocity = 0.0f;
        this->linearVelocity  = 0.0f;

        this->angularSpeed = 0.0f;
        this->linearSpeed  = 0.0f;

        this->leftSpeed  = 0.0f;
        this->rightSpeed = 0.0f;

        this->enable = true;
        this->stop   = false;

        if(standalone)
        {
            // Create task
            xTaskCreate((TaskFunction_t)(&VelocityControlStepper::taskHandler),
                        this->name.c_str(),
                        VC_TASK_STACK_SIZE,
                        NULL,
                        VC_TASK_PRIORITY,
                        NULL);
        }

        this->execCnt  = 0;
        this->execMax  = 0;
        this->execTime = 0;
        this->execMoy  = 0.0;
    }

    /**
     * @brief  VelocityControlStepper compute
     */
    void VelocityControlStepper::Compute(float32_t period)
    {
        float32_t currentAngularVelocity = 0.0;
        float32_t currentLinearVelocity  = 0.0;

        float32_t speed_angular = 0.0;
        float32_t speed_linear  = 0.0;
        float32_t speed_left  = 0.0;
        float32_t speed_right = 0.0;

        if(this->enable == true)
        {
        	this->status |= (1<<0);

            // Get current velocities
            currentAngularVelocity = odometry->GetAngularVelocity();
            currentLinearVelocity  = odometry->GetLinearVelocity();

            // Set setpoint velocities
            speed_angular = this->angularVelocity;
            speed_linear  = this->linearVelocity;

            this->angularSpeed  = speed_angular;
            this->linearSpeed   = speed_linear;

            // Angular/Linear to Left/Right
            speed_left  = speed_linear - (speed_angular * ADW_MM / 1000.0) / 2.0;
            speed_right = speed_linear + (speed_angular * ADW_MM / 1000.0) / 2.0;

            // Convert robot speed (m/s) to motor speed (rad/s)
            speed_left  /= RATIO * WD_MM * _PI_;
            speed_right /= RATIO * WD_MM * _PI_;

            // Convert motor speed (rad/s) to motor speed (rot/s or Hz)
            speed_left  /= _2_PI_;
            speed_right /= _2_PI_;

            // Set speed to Motors
            this->leftSpeed  = -speed_left;
            this->rightSpeed =  speed_right;

            if(this->stop != true)
            {
                //JRO leftMotor->SetMotorSpeed(this->leftSpeed);
                //JRO rightMotor->SetMotorSpeed(this->rightSpeed);
            }
        }
        else
        {
        	this->status &= ~(1<<0);
        }
    }

    void VelocityControlStepper::taskHandler(void* obj)
    {
        TickType_t xLastWakeTime;
        TickType_t xFrequency = pdMS_TO_TICKS(VC_TASK_PERIOD_MS);

        VelocityControlStepper* instance = _velocityControlStepper;
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

            //4. Compute velocity (VelocityControlStepper)
            instance->Compute(period);

            // 5. Set previous tick
            prevTick = tick;

            // 6. Execution time measure
            period = xTaskGetTickCount() - tick;
            this->execTime += period;
            this->execCnt++;
            this->execMoy = (float32_t)this->execTime / (float32_t)this->execCnt;
            if(period > this->execMax)
                this->execMax = period;
        }
    }
}
