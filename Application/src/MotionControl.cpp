/**
 * @file    MotionControl.cpp
 * @author  Jeremy ROULLAND
 * @date    17 apr. 2017
 * @brief   Motion Control class
 */

#include "MotionControl.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define MC_TASK_STACK_SIZE          (256u)
#define MC_TASK_PRIORITY            (configMAX_PRIORITIES-3)

#define MC_TASK_PERIOD_MS           (5u)
#define TP_TASK_PERIOD_MS           (200u)
#define PG_TASK_PERIOD_MS           (10u)
#define PC_TASK_PERIOD_MS           (10u)
#define VC_TASK_PERIOD_MS           (5u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static MotionControl::FBMotionControl* _motionControl = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

namespace MotionControl
{

    FBMotionControl* FBMotionControl::GetInstance()
    {
        // If MotionControl instance already exists
        if(_motionControl != NULL)
        {
            return _motionControl;
        }
        else
        {
            _motionControl = new FBMotionControl();
            return _motionControl;
        }
    }

    FBMotionControl::FBMotionControl()
    {
        this->name = "MotionControl";
        this->taskHandle = NULL;

        // Set 16 Flags Status
        this->status = 0x0000;

        // MotionControl is enabled by default
        this->enable = true;

        // MotionControl Safeguard is enabled by default
        this->safeguard = true;

        // Odometry instance created in standalone mode
        this->odometry = Odometry::GetInstance(true);

        // VC, PC instances creations
        this->pc = PositionControl::GetInstance(false);
        this->pg = ProfileGenerator::GetInstance(false);
        this->tp = TrajectoryPlanning::GetInstance(false);

        // Create task
        xTaskCreate((TaskFunction_t)(&FBMotionControl::taskHandler),
                    this->name.c_str(),
                    MC_TASK_STACK_SIZE,
                    NULL,
                    MC_TASK_PRIORITY,
                    NULL);

        this->mutex = xSemaphoreCreateMutex();

        this->Qorders = xQueueCreate(10, sizeof(cmd_t));
    }

    void FBMotionControl::Enable()
    {
    	this->enable = true;
    	this->pc->Enable();
    }

    void FBMotionControl::Disable()
    {
    	this->enable = false;
    	this->pc->Disable();
    }

    void FBMotionControl::Test()
    {
        static uint32_t localTime = 0;

        // Schedule MotionControl
        localTime += MC_TASK_PERIOD_MS;

        // #1 Compute TrajectoryPlanning
        if((localTime % TP_TASK_PERIOD_MS) == 0)
            this->tp->Compute(TP_TASK_PERIOD_MS);

        // #2 Schedule PositionControl
        if((localTime % PC_TASK_PERIOD_MS) == 0)
            this->pc->Compute(PC_TASK_PERIOD_MS);

        // #3 Schedule ProfileGenerator
        if((localTime % PG_TASK_PERIOD_MS) == 0)
            this->pg->Compute(PG_TASK_PERIOD_MS);
    }

    void FBMotionControl::Compute(float32_t period)
    {
        static uint32_t localTime = 0;
        cmd_t cmd;

        // Update configuration & state status
        if(this->enable)
        	this->status |= (1<<0);
        else
        	this->status &= ~(1<<0);

        if(this->safeguard)
        	this->status |= (1<<1);
        else
        	this->status &= ~(1<<1);

        if(this->tp->isFinished())
            this->status |= (1<<8);
        else
            this->status &= ~(1<<8);

        // Schedule MotionControl
        localTime += MC_TASK_PERIOD_MS;

        // If MotionControl is disabled then don't schedule submodules
        if(this->enable == false)
            return;

        // #1 Schedule TrajectoryPlanning
        if((localTime % TP_TASK_PERIOD_MS) == 0)
        {
            // 1- Pull orders one by one
            if(this->tp->isFinished())
            {
                if(xQueueReceive(this->Qorders, &cmd, 0) == pdTRUE)
                {
                    switch (cmd.id)
                    {
                    case CMD_ID_GOLIN:
                        this->tp->goLinear(cmd.data.d);
                        break;
                    case CMD_ID_GOANG:
                        this->tp->goAngular(cmd.data.a);
                        break;
                    case CMD_ID_GOTO:
                        this->tp->gotoXY(cmd.data.xy.x, cmd.data.xy.y);
                        break;
                    default:
                        break;
                    }
                }
            }
            // 2- Compute TrajectoryPlanning
            this->tp->Compute((period * TP_TASK_PERIOD_MS) / MC_TASK_PERIOD_MS);
        }

        // #2 Schedule PositionControl
        if((localTime % PC_TASK_PERIOD_MS) == 0)
            this->pc->Compute((period * PC_TASK_PERIOD_MS) / MC_TASK_PERIOD_MS);

        // #3 Schedule ProfileGenerator
        if((localTime % PG_TASK_PERIOD_MS) == 0)
            this->pg->Compute((period * PG_TASK_PERIOD_MS) / MC_TASK_PERIOD_MS);
    }


    void FBMotionControl::taskHandler(void* obj)
    {
        TickType_t xLastWakeTime;
        TickType_t xFrequency = pdMS_TO_TICKS(MC_TASK_PERIOD_MS);

        FBMotionControl* instance = _motionControl;
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

            //4. Compute velocity (MotionControl)
//            instance->Compute(period);
            //instance->Test();

            // 5. Set previous tick
            prevTick = tick;
        }
    }
}
