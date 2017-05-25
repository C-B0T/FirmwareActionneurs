/**
 * @file    MotionControl.hpp
 * @author    Jeremy ROULLAND
 * @date    3 jan. 2017
 * @brief    MotionControl namespace include file
 */

#ifndef INC_MOTIONCONTROL_HPP_
#define INC_MOTIONCONTROL_HPP_

/**
 * @namespace MotionControl
 * @brief motion controller objects
 *
 * Contains class such as PositionControl, VelocityControl, MotionProfile, etc.
 */

#include "common.h"

// For std::string
#include <algorithm>


#include "Odometry.hpp"
#include "PositionControlStepper.hpp"
#include "ProfileGenerator.hpp"
#include "TrajectoryPlanning.hpp"

#include "Telemeter.hpp"

// FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

using namespace Location;

typedef enum
{
    CMD_ID_UNKNOWN                =    -1,
//    CMD_TYPE_CHECKUP            =    0x10,
//    CMD_ID_GET_POSITION            =    0x20,
    CMD_ID_GET_ANGLE            =    0x21,
    CMD_ID_GOTO                    =    0x40,
    CMD_ID_GOLIN                =    0x41,
    CMD_ID_GOANG                =    0x42,
    CMD_ID_SET_POSITION            =    0x50,
    CMD_ID_SET_ANGLE            =    0x51,
//    CMD_ID_STOP                    =    0x60,
}CMD_TYPE;

struct cmd_t
{
    CMD_TYPE id;
    union
    {
        float32_t d;
        float32_t a;
        struct {
        	float32_t x;
        	float32_t y;
        }xy;
    }data;
};

namespace MotionControl
{

    /**
     * @class FBMotionControl
     * @brief Feedback Controller class
     *
     * HOWTO :
     * -
     *
     */
    class FBMotionControl
    {
    public:
        /**
         * @brief Get instance method
         * @return VelocityLoop instance
         */
        static FBMotionControl* GetInstance();

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

        void Enable();

        void Disable();

        void DisableSafeguard()
        {
            this->safeguard = true;
        }
        void EnableSafeguard()
        {
            this->safeguard = false;
        }
        void ToggleSafeguard()
        {
            this->safeguard = !this->safeguard;
        }
        bool GetSafeguard()
        {
            return this->safeguard;
        }

        // MotionControl Commands
        void GoLin(int32_t d)
        {
            struct cmd_t cmd;

            cmd.id = CMD_ID_GOLIN;
            cmd.data.d = ((float32_t)d)/1000.0;

            xQueueSend(this->Qorders, (void*) &cmd, 0);
        }

        void GoAng(int32_t a)
        {
        	struct cmd_t cmd;

            cmd.id = CMD_ID_GOANG;
            cmd.data.a = ((float32_t)a)/10.0/180.0*_PI_;

            xQueueSend(this->Qorders, (void*) &cmd, 0);
        }

        void Goto(int32_t X, int32_t Y)
        {
        	struct cmd_t cmd;

            cmd.id = CMD_ID_GOTO;
            cmd.data.xy.x = ((float32_t)X)/1000.0;
            cmd.data.xy.y = ((float32_t)Y)/1000.0;

            xQueueSend(this->Qorders, (void*) &cmd, 0);
        }

        void Stop()
        {
            xQueueReset(this->Qorders);
            this->tp->stop();
        }

    protected:
        FBMotionControl();

        // 16 Flags Status
        uint16_t status;

        /**
         * @protected
         * @brief Instance name
         */
        std::string name;

        Odometry           *odometry;
        PositionControl    *pc;
        ProfileGenerator   *pg;
        TrajectoryPlanning *tp;

        HAL::Telemeter* telAv;
        HAL::Telemeter* telAr;


        bool enable;

        bool safeguard;

        struct {
            bool newOrder;
            uint8_t id;
            int32_t X;
            int32_t Y;
        } order;

        /**
         * @brief Compute motion control
         */
        void Compute(float32_t period);

        /**
         * @brief Compute motion control
         */
        void Test();

        /**
         * @protected
         * @brief Semaphore order
         *
         * Used to set order
         */
        SemaphoreHandle_t mutex;

        QueueHandle_t Qorders;

        /**
         * @protected
         * @brief OS Task handle
         *
         * Used by speed control loop
         */
        TaskHandle_t taskHandle;

        /**
         * @protected
         * @brief Speed control loop task handler
         * @param obj : Always NULL
         */
        void taskHandler (void* obj);
    };

}

#endif /* INC_MOTIONCONTROL_HPP_ */
