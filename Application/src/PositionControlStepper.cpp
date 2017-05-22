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


//#define ANGULAR_VEL_MAX               (0.314f)     /* Low (OK) */
#define ANGULAR_VEL_MAX             (3.28f)     /* Hight (OK) */
//#define ANGULAR_VEL_MAX             (12.0f)
//#define ANGULAR_ACC_MAX             (0.314f)     /* Low (OK) */
#define ANGULAR_ACC_MAX             (3.14f)     /* Hight (OK) */
//#define ANGULAR_ACC_MAX             (18.0f)
#define ANGULAR_PROFILE             (MotionProfile::PROFILE::POLY5)

//#define LINEAR_VEL_MAX              (0.04f)     /* Low (OK) */
//#define LINEAR_VEL_MAX              (0.4f)     /* Hight (OK) */
#define LINEAR_VEL_MAX              (0.468f)
//#define LINEAR_VEL_MAX              (1.0f)
//#define LINEAR_ACC_MAX              (0.05f)     /* Low (OK) */
//#define LINEAR_ACC_MAX              (0.5f)     /* Hight (OK) */
#define LINEAR_ACC_MAX              (0.2f)
#define LINEAR_PROFILE              (MotionProfile::PROFILE::POLY5)


#define ANGULAR_POSITION_PID_KP     (0.314f)
#define ANGULAR_POSITION_PID_KI     (0.03f)
#define ANGULAR_POSITION_PID_KD     (0.0f)

#define LINEAR_POSITION_PID_KP      (0.5f)
#define LINEAR_POSITION_PID_KI      (0.0f)
#define LINEAR_POSITION_PID_KD      (0.0f)


#define PC_TASK_STACK_SIZE          (512u)
#define PC_TASK_PRIORITY            (configMAX_PRIORITIES-4)

#define PC_TASK_PERIOD_MS           (100u)

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
            def.Motors.ID_left           =    PC_MOTOR_LEFT;
            def.Motors.ID_right          =    PC_MOTOR_RIGHT;
            def.PID_Angular.kp           =    ANGULAR_POSITION_PID_KP;
            def.PID_Angular.ki           =    ANGULAR_POSITION_PID_KI;
            def.PID_Angular.kd           =    ANGULAR_POSITION_PID_KD;
            break;

        case PositionControl::LINEAR:
            def.Motors.ID_left           =    PC_MOTOR_LEFT;
            def.Motors.ID_right          =    PC_MOTOR_RIGHT;
            def.PID_Linear.kp            =    LINEAR_POSITION_PID_KP;
            def.PID_Linear.ki            =    LINEAR_POSITION_PID_KI;
            def.PID_Linear.kd            =    LINEAR_POSITION_PID_KD;
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

        // Init Angular velocity control
        this->def = _getDefStructure(PositionControl::ANGULAR);
        this->pid_angular = PID(this->def.PID_Angular.kp,
                                this->def.PID_Angular.ki,
                                this->def.PID_Angular.kd,
                                PC_TASK_PERIOD_MS/1000.0);

        // Init Linear velocity control
        this->def = _getDefStructure(PositionControl::LINEAR);
        this->pid_linear  = PID(this->def.PID_Linear.kp,
                                this->def.PID_Linear.ki,
                                this->def.PID_Linear.kd,
                                PC_TASK_PERIOD_MS/1000.0);

        this->leftMotor  = Drv8813::GetInstance(this->def.Motors.ID_left);
        this->rightMotor = Drv8813::GetInstance(this->def.Motors.ID_right);

        this->odometry = Odometry::GetInstance();

        this->angularProfile = MotionProfile(ANGULAR_VEL_MAX,
                                             ANGULAR_ACC_MAX,
                                             ANGULAR_PROFILE);

        this->linearProfile = MotionProfile(LINEAR_VEL_MAX,
                                            LINEAR_ACC_MAX,
                                            LINEAR_PROFILE);


        // Get current positions
        currentAngularPosition = odometry->GetAngularPosition();
        currentLinearPosition  = odometry->GetLinearPosition();

        this->angularPosition = currentAngularPosition;
        this->linearPosition  = currentLinearPosition;

        this->angularPositionProfiled = currentAngularPosition;
        this->linearPositionProfiled  = currentLinearPosition;

        this->angularPositionLast = 0.0;
        this->linearPositionLast  = 0.0;

        this->angularPositionError = 0.0;
        this->linearPositionError  = 0.0;

        this->angularVelocity = 0.0f;
        this->linearVelocity  = 0.0f;

        this->angularProfile.SetSetPoint(currentAngularPosition,
                                         currentAngularPosition,
                                         this->getTime());
        this->linearProfile.SetSetPoint(currentLinearPosition,
                                        currentLinearPosition,
                                        this->getTime());

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
        float32_t currentAngularPosition = 0.0;
        float32_t currentLinearPosition  = 0.0;

        float32_t angularVelocity = 0.0;
        float32_t linearVelocity  = 0.0;

        float32_t time = getTime();

        // Get current positions
        currentAngularPosition = odometry->GetAngularPosition();
        currentLinearPosition  = odometry->GetLinearPosition();

        /*// Compute positions error
        this->angularPositionError = this->angularPosition - currentAngularPosition;
        this->linearPositionError  = this->linearPosition  - currentLinearPosition;

        this->pid_angular.SetSetpoint(this->angularPosition);
        this->pid_linear.SetSetpoint(this->linearPosition);

        // Compute PID
        angularVelocity = this->pid_angular.Get(currentAngularPosition);
        linearVelocity  = this->pid_linear.Get(currentLinearPosition);

        // Set Velocities requiered to velocity controller
        this->angularVelocity = angularVelocity;
        this->linearVelocity  = linearVelocity;*/

        this->angularProfile.SetPoint(this->angularPosition);
        this->linearProfile.SetPoint(this->linearPosition);

        this->angularPositionProfiled = this->angularProfile.Get(time);
        this->linearPositionProfiled  = this->linearProfile.Get(time);

        // OpenLoop
        //this->angularPositionError = this->angularPositionProfiled - this->angularPositionLast;
        //this->linearPositionError  = this->linearPositionProfiled  - this->linearPositionLast;

        // LoopBack
        // Compute positions error
        this->angularPositionError = this->angularPositionProfiled - currentAngularPosition;
        this->linearPositionError  = this->linearPositionProfiled  - currentLinearPosition;

        // FIXME PID Test Kp
        //this->angularPositionError *= 0.4;
        //this->linearPositionError  *= 0.5;

        // FIXME Force lin to callibrate ang
        //this->linearPositionError = 0.0;

        // Compute PID
        this->pid_angular.SetSetpoint(this->angularPositionProfiled);
        this->pid_linear.SetSetpoint(this->linearPositionProfiled);
        this->angularPositionError = this->pid_angular.Get(currentAngularPosition);
        this->linearPositionError  = this->pid_linear.Get(currentLinearPosition);

        if(!this->isPositioningFinished())
        {
            printf("%.7f\t%.7f\t", this->linearPositionProfiled, this->angularPositionProfiled);
            printf("%.7f\t%.7f\t", currentLinearPosition, currentAngularPosition);
        }

        this->angularPositionLast = this->angularPositionProfiled;
        this->linearPositionLast  = this->linearPositionProfiled;

        this->ToMotors();
    }

    void PositionControl::ToMotors()
    {
        float32_t LeftPosition  = 0.0;
        float32_t RightPosition = 0.0;

        float32_t LeftVelocity  = 0.0;
        float32_t RightVelocity = 0.0;

        if(this->enable == true)
        {
            this->status |= (1<<0);

            this->angularVelocity = this->angularPositionError * 1000.0 / PC_TASK_PERIOD_MS;
            this->linearVelocity  = this->linearPositionError * 1000.0 / PC_TASK_PERIOD_MS;

            if(!this->isPositioningFinished())
            {
                printf("%.7f\t%.7f\t", this->linearVelocity, this->angularVelocity);
                printf("\r\n");
            }

            // Angular&Linear (radian&meter) to Left&Right (meter&meter)
            LeftPosition  = this->linearPositionError - this->angularPositionError * ADW_MM/1000.0 / 2.0;
            RightPosition = this->linearPositionError + this->angularPositionError * ADW_MM/1000.0 / 2.0;


            LeftVelocity  = this->linearVelocity - this->angularVelocity * ADW_MM/1000.0 / 2.0;
            RightVelocity = this->linearVelocity + this->angularVelocity * ADW_MM/1000.0 / 2.0;

            // Robot (meter) to Motor (rotation)
            LeftPosition  = LeftPosition  * 1000.0 / (RATIO * WD_MM * _PI_);
            RightPosition = RightPosition * 1000.0 / (RATIO * WD_MM * _PI_);

            LeftVelocity  = LeftVelocity  * 1000.0 / (RATIO * WD_MM * _PI_);
            RightVelocity = RightVelocity * 1000.0 / (RATIO * WD_MM * _PI_);

            LeftPosition  = - LeftPosition;
            RightPosition = + RightPosition;

            LeftVelocity  = this->abs(LeftVelocity);
            RightVelocity = this->abs(RightVelocity);

            if(!this->isPositioningFinished())
            {
                //printf("%.7f\t%.7f\t", -LeftPosition, RightPosition);
                //printf("%.3f\t%.3f\t", LeftVelocity, RightVelocity);
            }

            if(LeftPosition < 0.0)
            {
                LeftPosition = -LeftPosition;
                this->leftMotor->SetDirection(Drv8813State::BACKWARD);
                this->leftMotor->PulseRotation(LeftPosition*PC_USTEP);
                this->leftMotor->SetSpeedStep((uint32_t)(LeftVelocity*PC_USTEP));
            }
            else if (LeftPosition > 0.0)
            {
                LeftPosition = +LeftPosition;
                this->leftMotor->SetDirection(Drv8813State::FORWARD);
                this->leftMotor->PulseRotation(LeftPosition*PC_USTEP);
                this->leftMotor->SetSpeedStep((uint32_t)(LeftVelocity*PC_USTEP));
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
                this->rightMotor->SetSpeedStep((uint32_t)(RightVelocity*PC_USTEP));
            }
            else if (RightPosition > 0.0)
            {
                RightPosition = +RightPosition;
                this->rightMotor->SetDirection(Drv8813State::FORWARD);
                this->rightMotor->PulseRotation(RightPosition*PC_USTEP);
                this->rightMotor->SetSpeedStep((uint32_t)(RightVelocity*PC_USTEP));
            }
            else
            {
                this->rightMotor->SetDirection(Drv8813State::DISABLED);
            }

            /*if(this->leftMotor->IsMoving() || this->rightMotor->IsMoving())
                printf("%d\t%d\r\n", this->leftMotor->IsMoving(), this->rightMotor->IsMoving());*/
        }
        else
        {
            this->status &= ~(1<<0);
            this->leftMotor->SetDirection(Drv8813State::DISABLED);
            this->rightMotor->SetDirection(Drv8813State::DISABLED);
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

    float32_t PositionControl::abs(float32_t val)
    {
        if(val < 0.0)
            val = -val;
        return val;
    }
}
