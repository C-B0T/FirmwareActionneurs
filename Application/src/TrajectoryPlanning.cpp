/**
 * @file    TrajectoryPlanning.cpp
 * @author  Jeremy ROULLAND
 * @date    12 nov. 2016
 * @brief   Motion profile (Trapezoidal, S-Curve, ...)
 */

#include "TrajectoryPlanning.hpp"

#include <math.h>

using namespace MotionControl;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define TP_TASK_STACK_SIZE          (512u)
#define TP_TASK_PRIORITY            (configMAX_PRIORITIES-6)

#define TP_TASK_PERIOD_MS           (100u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static MotionControl::TrajectoryPlanning* _trajectoryPlanning = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

namespace MotionControl
{
    TrajectoryPlanning* TrajectoryPlanning::GetInstance(bool standalone)
    {
        // If TrajectoryPlanning instance already exists
        if(_trajectoryPlanning != NULL)
        {
            return _trajectoryPlanning;
        }
        else
        {
            _trajectoryPlanning = new TrajectoryPlanning(standalone);
            return _trajectoryPlanning;
        }
    }

    TrajectoryPlanning::TrajectoryPlanning(bool standalone)
    {
        this->name = "TrajectoryPlanning";
        this->taskHandle = NULL;

        this->finished = true;

        this->state = FREE;
        this->step = 0;

        // Set down 16 Flags Status
        this->status = 0x0000;

        this->stallMode = 0;

        this->X[0] = 0.0;
        this->Y[0] = 0.0;
        this->XYn  = 0;

        this->linearSetPoint = 0.0;
        this->linearNextSetPoint = 0.0;
        this->angularSetPoint = 0.0;

        this->startTime = 0.0;
        this->startLinearPosition  = 0.0;
        this->startAngularPosition = 0.0;

        this->endLinearPosition = 0.0;
        this->endAngularPosition = 0.0;


        this->odometry = Odometry::GetInstance();
        this->position = PositionControl::GetInstance();

        if(standalone)
        {
            // Create task
            xTaskCreate((TaskFunction_t)(&TrajectoryPlanning::taskHandler),
                        this->name.c_str(),
                        TP_TASK_STACK_SIZE,
                        NULL,
                        TP_TASK_PRIORITY,
                        NULL);
        }

    }
    void TrajectoryPlanning::goLinear(float32_t linear) // linear in meters
    {
        float32_t Lm = this->odometry->GetLinearPosition();

    	this->linearSetPoint = Lm + linear;

        this->state = LINEAR;
        this->step  = 1;
    }

    void TrajectoryPlanning::goAngular(float32_t angular) // angular in radian
    {
        this->angularSetPoint = angular;

        this->state = ANGULAR;
        this->step  = 1;
    }

    void TrajectoryPlanning::freewheel()
    {
        this->state = FREE;
        this->step  = 1;
    }

    void TrajectoryPlanning::stop()
    {
        //TODO: Deceleration

        this->state = STOP;
        this->step  = 1;
    }

    void TrajectoryPlanning::gotoXY(float32_t X, float32_t Y)
    {
        robot_t r;

        this->odometry->GetRobot(&r);

        float32_t Xm = static_cast<float32_t>(r.Xmm) / 1000.0;
        float32_t Ym = static_cast<float32_t>(r.Ymm) / 1000.0;
        float32_t Lm = static_cast<float32_t>(r.Lmm) / 1000.0;

        float32_t XX = pow((X - Xm), 2);
        float32_t YY = pow((Y - Ym), 2);

        float32_t dX = X - Xm;   // meters
        float32_t dY = Y - Ym;   // meters

        this->linearSetPoint  = Lm + sqrtl(XX + YY); // meters
        this->angularSetPoint = atan2f(dY,dX);  // radians

        /* Faster path */
        while( (this->angularSetPoint - r.O) > _PI_)
            this->angularSetPoint -= _2_PI_;
        while( (this->angularSetPoint - r.O) < -_PI_)
            this->angularSetPoint += _2_PI_;

        this->state = LINEARPLAN;
        this->step  = 1;
    }

    void TrajectoryPlanning::pushXY(float32_t X[], float32_t Y[], uint32_t n)
    {
        uint32_t i;

        assert(n<10);

        for(i=0 ; i<n ; i++)
        {
            this->X[i] = X[i];
            this->Y[i] = Y[i];
        }

        this->XYn = n;

        this->state = DRAWPLAN;
        this->step  = 1;
    }

    int32_t TrajectoryPlanning::stallX(int32_t stallMode)
    {
        // TODO:Check the stallMode coherence (Ex1: if ur on the left side of the table don't exe rightTable side to side Mode)
        //                                    (Ex2: if ur on the up side of the table don't exe downTable side to side Mode)

        struct robot r;
        odometry->GetRobot(&r);

        startLinearPosition = r.L;
        startAngularPosition = r.O;

        endLinearPosition = startLinearPosition - 0.0;
        endAngularPosition = 0.0;

        state = STALLX;
        step = 1;

        return 0;
    }

    int32_t TrajectoryPlanning::stallY(int32_t stallMode)
    {
        // TODO:Check the stallMode coherence (Ex1: if ur on the left side of the table don't exe rightTable side to side Mode)
        //                                    (Ex2: if ur on the up side of the table don't exe downTable side to side Mode)

        struct robot r;
        odometry->GetRobot(&r);

        startLinearPosition = r.L;
        startAngularPosition = r.O;

        endLinearPosition = startLinearPosition - 0.0;
        endAngularPosition = _PI_/2.0;

        state = STALLY;
        step = 1;

        return 0;
    }


    float32_t TrajectoryPlanning::update()
    {
        if( (this->state != FREE) && (this->state != STOP) )
        {
        	this->status |= (1<<8);
            this->finished = false;
            calculateMove();
        }
        else
        {
            this->status &= ~(1<<8);
            this->finished = true;
        }

    	switch(state)
        {
            // Simple mouvements
            case LINEAR:
                calculateGoLinear();
                break;

            case ANGULAR:
                calculateGoAngular();
                break;

            case FREE:
            	calculateFree();
            	break;

            case STOP:
                calculateStop();
                break;

            case KEEP:
                calculateKeepPosition();
                break;

            // semi-complex mouvements
            case LINEARPLAN:
                calculateLinearPlan();
                break;

            case DRAWPLAN:
                calculateDrawPlan();
                break;

            // complex mouvements
            case CURVEPLAN:
                calculateCurvePlan();
                break;

            // special mouvements
            case STALLX:
                calculateStallX(1);
                break;

            case STALLY:
                calculateStallY(1);
                break;

            default:
                break;
        }

		return 0;
    }

    void TrajectoryPlanning::calculateGoLinear()
    {
        switch (step)
        {
            case 1:    // Set order
                //this->profile->StartLinearPosition(this->linearSetPoint);
                //this->profile->StartAngularPosition(odometry->GetAngularPosition());
                this->position->SetLinearPosition(this->linearSetPoint);
                this->position->SetAngularPosition(odometry->GetAngularPosition());
                //this->profile->StartLinearVelocity(this->linearSetPoint);
                this->step = 2;
                break;

            case 2:    // Check is arrived
                //if(this->profile->isPositioningFinished())
                if(this->position->isPositioningFinished())
                {
                    this->step = 3;
                    this->state = FREE;
                }
                break;

            default:
                break;
        }
    }

    void TrajectoryPlanning::calculateGoAngular()
    {
        switch (step)
        {
            case 1:    // Set order
                //this->profile->StartLinearPosition(odometry->GetLinearPosition());
                //this->profile->StartAngularPosition(this->angularSetPoint);
                this->position->SetLinearPosition(odometry->GetLinearPosition());
                this->position->SetAngularPosition(this->angularSetPoint);
                //this->profile->StartAngularVelocity(this->angularSetPoint);
                this->step = 2;
                break;

            case 2:    // Check is arrived
                //if(this->profile->isPositioningFinished())
                if(this->position->isPositioningFinished())
                {
                    this->step = 3;
                    this->state = FREE;
                }
                break;

            default:
                break;
        }
    }

    void TrajectoryPlanning::calculateFree()
    {
        this->position->Disable();
    }

    void TrajectoryPlanning::calculateMove()
    {
        this->position->Enable();
    }

    void TrajectoryPlanning::calculateStop()
    {
        this->position->Disable();
    }

    void TrajectoryPlanning::calculateKeepPosition()
    {
        //Do nothing else => Keeping position calculation on preview order
    }

    void TrajectoryPlanning::calculateLinearPlan()
    {
        switch (step)
        {
            case 1:    // Start Angular Position
                break;

            case 2:
                if(this->position->isPositioningFinished())
                {
                    step = 3;
                }
                break;

            case 3:    // Start Linear Position
                break;

            case 4:
                if(this->position->isPositioningFinished())
                {
                    step = 5;
                    this->state = FREE;
                }
                break;

            default:
                break;
        }

        switch (step)
        {
            case 1:
                //this->profile->StartLinearPosition(odometry->GetLinearPosition());
                //this->profile->StartAngularPosition(this->angularSetPoint);
                this->position->SetAngularPosition(this->angularSetPoint);
                //this->profile->StartAngularVelocity(this->angularSetPoint);
                step = 2;
                break;

            case 2:
                break;

            case 3:
                //this->profile->StartLinearPosition(this->linearSetPoint);
                //this->profile->StartAngularPosition(odometry->GetAngularPosition());
                this->position->SetLinearPosition(this->linearSetPoint);
                //this->profile->StartLinearVelocity(this->linearSetPoint);
                step = 4;
                break;

            case 4:
                break;

            default:
                break;
        }

    }

    void TrajectoryPlanning::updateXYtoLA(uint32_t n)
    {
        float32_t Xm, Ym, Lm;
        float32_t XX, YY;
        float32_t dX, dY;
        robot_t r;
        uint32_t i;

        this->odometry->GetRobot(&r);

        Xm = static_cast<float32_t>(r.Xmm) / 1000.0;
        Ym = static_cast<float32_t>(r.Ymm) / 1000.0;
        Lm = static_cast<float32_t>(r.Lmm) / 1000.0;

        XX = pow((this->X[n] - Xm), 2);
        YY = pow((this->Y[n] - Ym), 2);

        dX = this->X[n] - Xm;   // meters
        dY = this->Y[n] - Ym;   // meters

        this->linearSetPoint  = Lm + sqrtl(XX + YY); // meters
        this->angularSetPoint = atan2f(dY,dX);  // radians

        /* Faster path */
        while( (this->angularSetPoint - r.O) > _PI_)
            this->angularSetPoint -= _2_PI_;
        while( (this->angularSetPoint - r.O) < -_PI_)
            this->angularSetPoint += _2_PI_;

        this->linearNextSetPoint  = this->linearSetPoint;

        if(this->XYn >= 2)
        {
    for(i=n ; i < (this->XYn-1) ; i++)
    {
    XX = pow((this->X[i] - this->X[i-1]), 2);
    YY = pow((this->Y[i] - this->Y[i-1]), 2);

    this->linearSetPoint  += sqrtl(XX + YY);
    }
        }
    }

    void TrajectoryPlanning::calculateDrawPlan()
    {
        static uint32_t n = 0;

        float32_t currentLinearPosition = 0.0;

        switch (step)
        {
            case 1:    // Start Angular Position
                n = 0;
                step = 2;
                this->updateXYtoLA(n);
            //JRO    this->profile->StartLinearPosition(this->linearSetPoint);
            //JRO    this->profile->StartAngularPosition(this->angularSetPoint);
                /* no break */
                //break;

            case 2:
                this->updateXYtoLA(n);
                currentLinearPosition = odometry->GetLinearPosition();
                if(abs(this->linearNextSetPoint - currentLinearPosition) <= 0.1)
                {
                    n++;
                    step = 2;
                    if(n >= this->XYn-1)
                    {
                        step = 3;
                    }
                    this->updateXYtoLA(n);
             //JRO       this->profile->StartAngularPosition(this->angularSetPoint);
                }
                break;

            case 3:    /* Last coordinate */
                this->updateXYtoLA(n);
                currentLinearPosition = odometry->GetLinearPosition();

                if(abs(this->linearSetPoint - currentLinearPosition) <= 0.1)
                {
                    step = 4;
                }
                break;

            case 4:    /* Finishing last coordinate */
                this->updateXYtoLA(n);
                currentLinearPosition = odometry->GetLinearPosition();
                //JRO if(this->profile->isPositioningFinished() || (abs(this->linearSetPoint - currentLinearPosition) <= 0.01))
                {
                    step = 5;
                    this->state = FREE;
                }
                break;

            default:
                break;
        }

        switch (step)
        {
            case 1:
                break;

            case 2:
            //JRO    this->profile->SetLinearPosition(this->linearSetPoint);
            //JRO    this->profile->SetAngularPosition(this->angularSetPoint);
                break;

            case 3:
            //JRO    this->profile->SetLinearPosition(this->linearSetPoint);
            //JRO    this->profile->SetAngularPosition(this->angularSetPoint);
                break;

            case 4:
            //JRO    this->profile->SetLinearPosition(this->linearSetPoint);
            //JRO    this->profile->SetAngularPosition(this->angularSetPoint);
                break;

            default:
                break;
        }

    }

    void TrajectoryPlanning::calculateCurvePlan()
    {
        //TODO: Curve Plan : Need both MotionProfile synchronized
    }


    void TrajectoryPlanning::calculateStallX(int32_t mode)
    {
        float32_t time = getTime();
        time -= startTime;

        float32_t profile = 0.0;

        //TODO:Add stallMode gestion
        bool jackBackLeft  = true;
        bool jackBackRight = true;

        switch (step)
        {
            case 1:
                //JRO if(this->profile->isPositioningFinished())
                    step = 2;
                break;

            case 2:
                if(jackBackLeft && jackBackRight)
                    step = 3;
                break;

            case 3:
                // X axis and Angular are calibrated
                break;

            default:
                break;
        }

        switch (step)
        {
            case 1: // Rotate to 0 rad
                this->position->SetAngularPosition(0.0);
                break;

            case 2:
                //TODO:Disable Angular asserv.
                //TODO:Back and wait both jacks
                break;

            case 3:
                //TODO:Modify X et O value in function of the mechanic
                odometry->SetXO(0.0, 0.0);
                break;

            default:
                break;
        }

    }

    void TrajectoryPlanning::calculateStallY(int32_t mode)
    {
        float32_t time = getTime();
        time -= startTime;

        float32_t profile = 0.0;

        //TODO:Add stallMode gestion
        bool jackBackLeft  = true;
        bool jackBackRight = true;

        switch (step)
        {
            case 1:
                //JRO if(this->profile->isPositioningFinished())
                    step = 2;
                break;

            case 2:
                if(jackBackLeft && jackBackRight)
                    step = 3;
                break;

            case 3:
                // Y axis and Angular are calibrated
                break;

            default:
                break;
        }

        switch (step)
        {
            case 1: // Rotate to pi/2 rad
                this->position->SetAngularPosition(_PI_/2.0);
                break;

            case 2:
                //TODO:Disable Angular asserv.
                //TODO:Back and wait both jacks
                break;

            case 3:
                //TODO:Modify Y value in function of the mechanic
                odometry->SetYO(0.0, _PI_/2.0);
                break;

            default:
                break;
        }

    }

    void TrajectoryPlanning::Compute(float32_t period)
    {
        this->status |= (1<<0);

        this->update();
    }

    void TrajectoryPlanning::taskHandler(void* obj)
    {
        TickType_t xLastWakeTime;
        TickType_t xFrequency = pdMS_TO_TICKS(TP_TASK_PERIOD_MS);

        TrajectoryPlanning* instance = _trajectoryPlanning;
        TickType_t prevTick = 0u,  tick = 0u;

        float32_t period = 0.0f;

        // 1. Initialize periodical task
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

            //4. Compute profile (ProfileGenerator)
            instance->Compute(period);

            // 5. Set previous tick
            prevTick = tick;
        }
    }

    float32_t TrajectoryPlanning::getTime()
    {
        float32_t time = 0.0;

        time = static_cast<float32_t>(xTaskGetTickCount());
        time /= 1000.0;

        return  time;
    }

    float32_t TrajectoryPlanning::abs(float32_t val)
    {
        if(val < 0.0)
            val = -val;
        return val;
    }
}
