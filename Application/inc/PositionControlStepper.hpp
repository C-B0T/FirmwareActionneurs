/**
 * @file    PositionControl.hpp
 * @author  Jeremy ROULLAND
 * @date    24 dec. 2016
 * @brief   PositionControl class
 */

#ifndef INC_POSITIONCONTROL_HPP_
#define INC_POSITIONCONTROL_HPP_

#include "HAL.hpp"
#include "Utils.hpp"
#include "Odometry.hpp"
#include "DRV8813.hpp"
#include "MotionProfile.hpp"

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
    // Motors
    struct
    {
        HAL::Drv8813::ID ID_left;
        HAL::Drv8813::ID ID_right;
    }Motors;

    // PID
    struct pc_pid
    {
        float32_t    kp;
        float32_t    ki;
        float32_t    kd;
    }PID_Angular;

    struct pc_pid PID_Linear;

}PC_DEF;

/*----------------------------------------------------------------------------*/
/* Class declaration                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @namespace MotionControl
 */
namespace MotionControl
{
    /**
     * @class PositionControl
     * @brief Velocity control class
     *
     * HOWTO :
     * -
     *
     */
    class PositionControl
    {
    public:

        /**
         * @brief Position Identifier List
         */
        enum ID
        {
            ANGULAR = 0,    //!< ANGULAR
            LINEAR,         //!< LINEAR
            POSITION_MAX    //!< POISTION_MAX
        };

        /**
         * @brief Get instance method
         * @return Position loop instance
         */
        static PositionControl* GetInstance(bool standalone = true);

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
         * @brief Set angular position setpoint
         */
        void SetAngularPosition(float32_t position)
        {
            float32_t currentAngularPosition  = 0.0;
            float32_t time = 0.0;

            // Get current and time
            currentAngularPosition = odometry->GetAngularPosition();
            time = getTime();

            // Set angular position order
            this->angularPosition = position;

            // Start profile
             this->angularProfile.SetSetPoint(this->angularPosition, currentAngularPosition, time);
          }

        /**
         * @brief Get angular position setpoint
         */
        float32_t GetAngularPosition()
        {
            return this->angularPosition;
        }

        /**
         * @brief Set linear position setpoint
         */
        void SetLinearPosition(float32_t position)
        {
            float32_t currentLinearPosition  = 0.0;
            float32_t time = 0.0;

            // Get current and time
            currentLinearPosition = odometry->GetLinearPosition();
            time = getTime();

            // Set linear position order
            this->linearPosition = position;

            // Start profile
             this->linearProfile.SetSetPoint(this->linearPosition, currentLinearPosition, time);
        }

        /**
         * @brief Get linear position setpoint
         */
        float32_t GetLinearPosition()
        {
            return this->linearPosition;
        }

        /**
        * @brief Get angular position error
         */
        float32_t GetAngularPositionProfiled()
        {
            return this->angularPositionProfiled;
        }

        /**
        * @brief Get linear position error
         */
        float32_t GetLinearPositionProfiled()
        {
            return this->linearPositionProfiled;
        }

        /**
        * @brief Get angular position error
         */
        float32_t GetAngularPositionError()
        {
            return this->angularPositionError;
        }

        /**
        * @brief Get linear position error
         */
        float32_t GetLinearPositionError()
        {
            return this->linearPositionError;
        }

        /**
         * @brief Get angular velocity requiered
         */
        float32_t GetAngularVelocity()
        {
            return this->angularVelocity;
        }

        /**
         * @brief Get linear velocity requiered
         */
        float32_t GetLinearVelocity()
        {
            return this->linearVelocity;
        }

        /**
         * @brief Set Angular Kp
         */
        void SetAngularKp(float32_t Kp)
        {
            this->def.PID_Angular.kp = Kp;
            this->pid_angular.SetKp(Kp);
        }

        /**
         * @brief Set Angular Ki
         */
        void SetAngularKi(float32_t Ki)
        {
            this->def.PID_Angular.ki = Ki;
            this->pid_angular.SetKi(Ki);
        }

        /**
         * @brief Set Angular Kd
         */
        void SetAngularKd(float32_t Kd)
        {
            this->def.PID_Angular.kd = Kd;
            this->pid_angular.SetKd(Kd);
        }

        /**
         * @brief Set Linear Kp
         */
        void SetLinearKp(float32_t Kp)
        {
            this->def.PID_Linear.kp = Kp;
            this->pid_linear.SetKp(Kp);
        }

        /**
         * @brief Set Linear Ki
         */
        void SetLinearKi(float32_t Ki)
        {
            this->def.PID_Linear.ki = Ki;
            this->pid_linear.SetKi(Ki);
        }

        /**
         * @brief Set Linear Kd
         */
        void SetLinearKd(float32_t Kd)
        {
            this->def.PID_Linear.kd = Kd;
            this->pid_linear.SetKd(Kd);
        }

        /**
         * @brief Enable
         */
        void Enable()
        {
            if(this->enable == false)
            {
                this->pid_angular.Reset();
                this->pid_linear.Reset();
                this->enable = true;
            }
        }

        /**
         * @brief Disable
         */
        void Disable()
        {
            if(this->enable == true)
            {
                this->enable = false;
            }
        }

        /**
         * @brief is angular and linear positioning finished
         */
        bool isPositioningFinished()
        {
            bool Finished = false;

            Finished = this->angularProfile.isFinished() && this->linearProfile.isFinished();

            return Finished;
        }

        /**
         * @brief Compute robot velocity
         */
        void Compute(float32_t period);

        /**
         * @brief Compute robot velocity
         */
        void ToMotors();

    protected:

        /**
         * @protected
         * @brief Private constructor
         */
        PositionControl(bool standalone);

        /**
         * @protected
         * @brief get time in seconds
         */
        float32_t getTime();

        /**
         * @protected
         * @brief get absolute value
         */
        float32_t abs(float32_t val);

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
         * @brief angular velocity PID controller
         */
        Utils::PID    pid_angular;

        /**
         * @protected
         * @brief linear velocity PID controller
         */
        Utils::PID    pid_linear;

        /**
         * @protected
         * @brief Coef definitions
         */
        PC_DEF def;

        /**
         * @protected
         * @brief Odometry instance
         */
        Location::Odometry* odometry;

        /**
         * @protected
         * @brief MotionProfile asserts
         */
        MotionProfile linearProfile;

        /**
         * @protected
         * @brief MotionProfile asserts
         */
        MotionProfile angularProfile;

        /**
         * @protected
         * @brief
         */
        HAL::Drv8813* leftMotor;

        /**
         * @protected
         * @brief
         */
        HAL::Drv8813* rightMotor;

        /**
         * @protected
         * @brief Angular position setpoint
         */
        float32_t angularPosition;

        /**
         * @protected
         * @brief Linear position setpoint
         */
        float32_t linearPosition;

        /**
         * @protected
         * @brief Angular position setpoint
         */
        float32_t angularPositionProfiled;

        /**
         * @protected
         * @brief Linear position setpoint
         */
        float32_t linearPositionProfiled;

        /**
         * @protected
         * @brief Angular position last setpoint
         */
        float32_t angularPositionLast;

        /**
         * @protected
         * @brief Linear position last setpoint
         */
        float32_t linearPositionLast;

        /**
         * @protected
         * @brief Angular position error
         */
        float32_t angularPositionError;

        /**
         * @protected
         * @brief Linear position error
         */
        float32_t linearPositionError;

        /**
         * @protected
         * @brief angular velocity required
         */
        float32_t angularVelocity;

        /**
         * @protected
         * @brief linear velocity required
         */
        float32_t linearVelocity;

        /**
         * @protected
         * @brief enable/disable
         */
        bool enable;

        /**
         * @protected
         * @brief OS Task handle
         *
         * Used by position control loop
         */
        TaskHandle_t taskHandle;

        /**
         * @protected
         * @brief Position control loop task handler
         * @param obj : Always NULL
         */
        void taskHandler (void* obj);
    };
}

#endif /* INC_POSITION */
