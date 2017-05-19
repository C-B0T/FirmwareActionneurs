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
            // Info: angular position is absolute
            float32_t currentLinearPosition = 0.0;
            float32_t currentAngularPosition = 0.0;

            // Get current position
            currentLinearPosition = odometry->GetLinearPosition();
            currentAngularPosition = odometry->GetAngularPosition();

            // Set relative positions order
            this->linearPosition  = currentLinearPosition;
            this->angularPosition = position - currentAngularPosition;
            this->update = true;
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
            // Info: linear position is absolute
            float32_t currentLinearPosition = 0.0;
            float32_t currentAngularPosition = 0.0;

            // Get current position
            currentLinearPosition = odometry->GetLinearPosition();
            currentAngularPosition = odometry->GetAngularPosition();

            // Set relative positions order
            this->linearPosition = position - currentLinearPosition;
            this->angularPosition = currentAngularPosition;
            this->update = true;
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
         * @brief Enable
         */
        void Enable()
        {
            if(this->enable == false)
            {
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
                this->leftMotor->SetDirection(HAL::Drv8813State::DISABLED);
                this->rightMotor->SetDirection(HAL::Drv8813State::DISABLED);
                this->enable = false;
            }
        }

        /**
         * @brief is angular and linear positioning finished
         */
        bool isPositioningFinished()
        {
            bool Finished = false;

            Finished = !this->leftMotor->IsMoving() && !this->rightMotor->IsMoving();

            return Finished;
        }

        /**
         * @brief Test compute robot velocity
         */
        void Test();

        /**
         * @brief Compute robot velocity
         */
        void Compute(float32_t period);


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
        PC_DEF def;

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
         * @brief Profile finished
         */
        bool Finished;

        /**
         * @protected
         * @brief enable/disable
         */
        bool enable;

        /**
         * @protected
         * @brief enable/disable
         */
        bool update;

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
