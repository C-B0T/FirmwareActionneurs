/**
 * @file    VelocityControlStepper.hpp
 * @author  Jeremy ROULLAND
 * @date    20 dec. 2016
 * @brief   VelocityControlStepper class
 */

#ifndef INC_VELOCITYCONTROLSTEPPER_HPP_
#define INC_VELOCITYCONTROLSTEPPER_HPP_

#include <stdio.h>

#include "common.h"

#include "HAL.hpp"
#include "DRV8813.hpp"
#include "Utils.hpp"
#include "Odometry.hpp"

// FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/
typedef struct
{
    // Motor
    struct
    {
        HAL::Drv8813::ID ID_left;
        HAL::Drv8813::ID ID_right;
    }Motors;

}VC_DEF;

/*----------------------------------------------------------------------------*/
/* Class declaration                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @namespace MotionControl
 */
namespace MotionControl
{
    /**
     * @class VelocityControlStepper
     * @brief Velocity control class
     *
     * HOWTO :
     * -
     *
     */
    class VelocityControlStepper
    {
    public:

        /**
         * @brief Velocity Identifier List
         */
        enum ID
        {
            ANGULAR = 0,    //!< ANGULAR
            LINEAR,         //!< LINEAR
            VELOCITY_MAX    //!< VELOCITY_MAX
        };

        /**
         * @brief Get instance method
         * @return VelocityLoop instance
         * @return BrushlessMotor instance
         */
        static VelocityControlStepper* GetInstance(bool standalone = true);

        /**
         * @brief Return instance name
         */
        std::string Name()
        {
            return this->name;
        }

        uint16_t GetStatus()
        {
        	return this->status;
        }

        /**
         * @brief Set angular velocity setpoint
         */
        void SetAngularVelocity(float32_t velocity)
        {
            this->angularVelocity = velocity;
        }

        /**
         * @brief Get angular velocity setpoint
         */
        float32_t GetAngularVelocity()
        {
            float32_t r = 0.0;

            r = this->angularVelocity;

            return r;
        }

        /**
         * @brief Set linear velocity setpoint
         */
        void SetLinearVelocity(float32_t velocity)
        {
            this->linearVelocity = velocity;
        }

        /**
         * @brief Get linear velocity setpoint
         */
        float32_t GetLinearVelocity()
        {
            return this->linearVelocity;
        }

        /**
         * @brief Get angular speed
         */
        float32_t GetAngularSpeed()
        {
            return this->angularSpeed;
        }

        /**
         * @brief Get linear speed
         */
        float32_t GetLinearSpeed()
        {
            return this->linearSpeed;
        }

        /**
         * @brief Get left speed
         */
        float32_t GetLeftSpeed()
        {
            return this->leftSpeed;
        }

        /**
         * @brief Get right speed
         */
        float32_t GetRightSpeed()
        {
            return this->rightSpeed;
        }

        void Brake()
        {
            //JRO leftMotor->Brake();
            //JRO rightMotor->Brake();
        }

        void Brake(float32_t percent)
        {
            //JRO leftMotor->Brake(percent);
            //JRO rightMotor->Brake(percent);
        }

        void Stop()
        {
            this->stop = true;
            //JRO leftMotor->Brake();
            //JRO rightMotor->Brake();
        }

        void Start()
        {
            this->stop = false;
            //JRO leftMotor->SetMotorSpeed(0.0);
            //JRO rightMotor->SetMotorSpeed(0.0);
        }

        void Enable()
        {
            if(this->enable == false)
            {
                if (this->stop == false)
                {
                    //JRO leftMotor->SetMotorSpeed(0.0);
                    //JRO rightMotor->SetMotorSpeed(0.0);
                    this->enable = true;
                }
            }
        }

        void Disable()
        {
            if(this->enable == true)
            {
                //JRO leftMotor->SetMotorSpeed(0.0);
                //JRO rightMotor->SetMotorSpeed(0.0);
                this->enable = false;
            }
        }

        void GetExecTime(float32_t* Moy, uint32_t* Max)
        {
            *Moy = this->execMoy;
            *Max = this->execMax;
        }

        /**
         * @brief Compute robot velocity
         */
        void Compute(float32_t period);

    protected:

        /**
         * @protected
         * @brief Private constructor
         */
        VelocityControlStepper(bool standalone);

        /**
         * @protected
         * @brief Instance name
         */
        std::string name;

        /**
         * @protected
         * @brief 16 Flags Status
         */
        uint16_t status;

        /**
         * @protected
         * @brief Coef definitions
         */
        VC_DEF def;

        /**
         * @protected
         * @brief Odometry instance
         */
        Location::Odometry* odometry;

        /**
         * @protected
         * @brief Left motor brushless driver instance
         */
        HAL::Drv8813* leftMotor;

        /**
         * @protected
         * @brief Right motor brushless driver instance
         */
        HAL::Drv8813* rightMotor;

        /**
         * @protected
         * @brief Angular velocity setpoint
         */
        float32_t angularVelocity;

        /**
         * @protected
         * @brief Linear velocity setpoint
         */
        float32_t linearVelocity;

        /**
         * @protected
         * @brief angular speed
         */
        float32_t angularSpeed;

        /**
         * @protected
         * @brief right speed
         */
        float32_t linearSpeed;

        /**
         * @protected
         * @brief left speed
         */
        float32_t leftSpeed;

        /**
         * @protected
         * @brief right speed
         */
        float32_t rightSpeed;


        /**
         * @protected
         * @brief enable/disable
         */
        bool enable;

        /**
         * @protected
         * @brief emergency stop
         */
        bool stop;

        /**
         * @protected
         * @brief OS Task handle
         *
         * Used by speed control loop
         */
        TaskHandle_t taskHandle;

        /**
         * @protected
         * @brief Task execution time
         *
         * Measure execution time (moy and max)
         */
        uint32_t execTime;
        uint32_t execCnt;
        float32_t execMoy;
        uint32_t execMax;

        /**
         * @protected
         * @brief Speed control loop task handler
         * @param obj : Always NULL
         */
        void taskHandler (void* obj);
    };
}

#endif /* INC_VELOCITYCONTROLSTEPPER_HPP_ */
