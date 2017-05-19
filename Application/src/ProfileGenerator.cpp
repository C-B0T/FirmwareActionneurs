/**
 * @file    ProfileGenerator.cpp
 * @author  Jeremy ROULLAND
 * @date    16 may 2017
 * @brief   ProfileGenerator class
 */

#include "ProfileGenerator.hpp"
#include "common.h"

#include <stdio.h>

using namespace MotionControl;
using namespace Location;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define _PI_        3.14159265358979323846

// Motion wheel characteristic
#define WD_MM       59.9            // Wheel diameter
#define ADW_MM      129.2           // Axial distance between wheels
//#define RATIO       1.32            // Gear reduction ratio
#define RATIO       1.315789474     // Gear reduction ratio
//#define RATIO       1.343551914     // Gear reduction ratio

#define ANGULAR_VEL_MAX               (0.314f)     /* Low (OK) */
//#define ANGULAR_VEL_MAX             (3.14f)     /* Hight (OK) */
//#define ANGULAR_VEL_MAX             (12.0f)
#define ANGULAR_ACC_MAX             (0.314f)     /* Low (OK) */
//#define ANGULAR_ACC_MAX             (3.14f)     /* Hight (OK) */
//#define ANGULAR_ACC_MAX             (18.0f)
#define ANGULAR_PROFILE             (VelocityProfile::PROFILE::POLY5)

#define LINEAR_VEL_MAX              (0.04f)     /* Low (OK) */
//#define LINEAR_VEL_MAX              (0.4f)     /* Hight (OK) */
//#define LINEAR_VEL_MAX              (0.8f)
#define LINEAR_ACC_MAX              (0.05f)     /* Low (OK) */
//#define LINEAR_ACC_MAX              (0.5f)     /* Hight (OK) */
#define LINEAR_PROFILE              (VelocityProfile::PROFILE::POLY5)

#define PG_MOTOR_LEFT               (HAL::Drv8813::ID::DRV8813_4)
#define PG_MOTOR_RIGHT              (HAL::Drv8813::ID::DRV8813_1)
#define PG_USTEP                    (1.0*200.0)

#define PG_TASK_STACK_SIZE          (512u)
#define PG_TASK_PRIORITY            (configMAX_PRIORITIES-5)

#define PG_TASK_PERIOD_MS           (10u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static MotionControl::ProfileGenerator* _profileGenerator = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static PG_DEF _getDefStructure (enum ProfileGenerator::ID id)
{
    PG_DEF def;

    assert(id < ProfileGenerator::PROFILE_MAX);

    switch(id)
    {
        case ProfileGenerator::ANGULAR:
            def.Profile_Angular.maxVel   =    ANGULAR_VEL_MAX;
            def.Profile_Angular.maxAcc   =    ANGULAR_ACC_MAX;
            def.Profile_Angular.profile  =    ANGULAR_PROFILE;
            break;

        case ProfileGenerator::LINEAR:
            def.Profile_Linear.maxVel    =    LINEAR_VEL_MAX;
            def.Profile_Linear.maxAcc    =    LINEAR_ACC_MAX;
            def.Profile_Linear.profile   =    LINEAR_PROFILE;
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
    ProfileGenerator* ProfileGenerator::GetInstance(bool standalone)
    {
        // If ProfileGenerator instance already exists
        if(_profileGenerator != NULL)
        {
            return _profileGenerator;
        }
        else
        {
            _profileGenerator = new ProfileGenerator(standalone);
            return _profileGenerator;
        }
    }

    /**
     * @brief  Profile generator constructor
     */
    ProfileGenerator::ProfileGenerator(bool standalone)
    {
        float32_t currentAngularPosition = 0.0;
        float32_t currentLinearPosition  = 0.0;

        this->name = "ProfileGenerator";
        this->taskHandle = NULL;

        this->status = 0x0000;

        this->Finished = false;

        // Init Angular motion profile generator
        this->def = _getDefStructure(ProfileGenerator::ANGULAR);
        this->angularProfile = VelocityProfile(this->def.Profile_Angular.maxVel,
                                             this->def.Profile_Angular.maxAcc,
                                             this->def.Profile_Angular.profile);

        // Init Linear motion profile generator
        this->def = _getDefStructure(ProfileGenerator::LINEAR);
        this->linearProfile = VelocityProfile(this->def.Profile_Linear.maxVel,
                                            this->def.Profile_Linear.maxAcc,
                                            this->def.Profile_Linear.profile);

        // Get instances
        this->odometry = Odometry::GetInstance();
        this->leftMotor = HAL::Drv8813::GetInstance(PG_MOTOR_LEFT);
        this->rightMotor = HAL::Drv8813::GetInstance(PG_MOTOR_RIGHT);

        // Get current velocity
        currentAngularPosition = this->odometry->GetAngularPosition();
        currentLinearPosition  = this->odometry->GetLinearPosition();

        // Initial values
        //this->angularPosition = currentAngularPosition;
        //this->linearPosition  = currentLinearPosition;
        this->angularVelocityProfiled = 0.0;
        this->linearVelocityProfiled  = 0.0;

        this->angularVelocity = 0.0;
        this->linearVelocity  = 0.0;

        this->angularProfile.SetSetPoint(currentAngularPosition,
                                         this->getTime());
        this->linearProfile.SetSetPoint(currentLinearPosition,
                                        this->getTime());

        this->angularPhaseProfile = Zero;
        this->linearPhaseProfile  = Zero;

        if(standalone)
        {
            // Create task
            xTaskCreate((TaskFunction_t)(&ProfileGenerator::taskHandler),
                        this->name.c_str(),
                        PG_TASK_STACK_SIZE,
                        NULL,
                        PG_TASK_PRIORITY,
                        NULL);
        }
    }

    /**
     * @brief Start angular velocity setpoint
     */
    void ProfileGenerator::StartAngularVelocity(float32_t angle)
    {
        // angle is absolute

        float32_t currentAngularPosition = 0.0;
        float32_t time = 0.0;

        // Set angular velocity order
        //this->angularPosition = angle;

        // Get current velocity and time
        /*currentAngularPosition = odometry->GetAngularPosition();
        time = getTime();

        // Set profile type
        this->angularPhaseProfile = AccDec;
        this->angularProfile.SetProfile(ANGULAR_PROFILE);

        // Start profile
        this->angularProfile.SetSetPoint(angle - currentAngularPosition, time);

        // Set current velocity
        this->angularVelocityProfiled = 0.0;*/

        // FIXME Force const Velocity
        this->angularVelocity = 2.0;
        this->linearVelocity  = 0.0;
    }

    /**
     * @brief start linear velocity setpoint
     */
    void ProfileGenerator::StartLinearVelocity(float32_t distance)
    {
        // distance is relative

        float32_t currentLinearPosition  = 0.0;
        float32_t time = 0.0;
        float32_t d = 0.0;

        // Set linear velocity order
        //this->linearPosition = distance;

        // Get current velocity and time
        //currentLinearPosition = odometry->GetLinearPosition();
        time = getTime();

        // Calculate min distance (sign preserved)
        /*if(this->linearVelocity >= currentLinearVelocity)
            d = this->linearProfile.GetMinDist(VelocityProfile::POLY5_P1);
        else
            d = - this->linearProfile.GetMinDist(VelocityProfile::POLY5_P1);*/

        // Choose profile type
        //if(abs(this->linearVelocity - currentLinearVelocity) <= 2.0*abs(d))
        /*{
            // Set profile type
            this->linearPhaseProfile = AccDec;
            this->linearProfile.SetProfile(LINEAR_PROFILE);

            // Start profile
            this->linearProfile.SetSetPoint(distance, time);
        }*/
        //else
        /*{
            // Set profile type
            this->linearPhaseProfile = Acc;
            this->linearProfile.SetProfile(VelocityProfile::POLY5_P1);

            // Start profile
            // 30/04/2017 : Remplacement de this->linearVelocityProfiled par currentLinearVelocity
            this->linearProfile.SetSetPoint(currentLinearVelocity+d, currentLinearVelocity, time);
        }*/

        // Set current velocity
        /*this->linearVelocityProfiled  = 0.0;*/

        // FIXME Force const Velocity
        this->angularVelocity = 0.0;
        // FIXME Temporaire pour les essais
        //this->linearVelocity  = LINEAR_VEL_MAX;
        this->linearVelocity  = 0.1;
    }


    /**
     * @brief  VelocityControl compute
     */
    void ProfileGenerator::Compute(float32_t period)
    {
        float32_t LeftVelocity  = 0.0;
        float32_t RightVelocity = 0.0;

        float32_t time = getTime();

        this->status |= (1<<0);

        // Generate profile
        this->Generate(period);

        // FIXME Ugly Ramps
        if(this->linearVelocity != 0.0)
        {
            // Lin Ramp
            this->linearVelocity += LINEAR_ACC_MAX*(PG_TASK_PERIOD_MS/1000.0);
            if(this->linearVelocity  > LINEAR_VEL_MAX)
                this->linearVelocity  = LINEAR_VEL_MAX;
        }

        if(this->angularVelocity != 0.0)
        {
            // Ang Ramp
            this->angularVelocity += ANGULAR_ACC_MAX*(PG_TASK_PERIOD_MS/1000.0);
            if(this->angularVelocity  > ANGULAR_VEL_MAX)
                this->angularVelocity  = ANGULAR_VEL_MAX;
        }

        // FIXME Force speed
        this->linearVelocityProfiled  = this->linearVelocity;
        this->angularVelocityProfiled = this->angularVelocity;

        // Angular&Linear [rad/s]&[m/s] to Left&Right [m/s]&[m/s]
        LeftVelocity  = this->linearVelocityProfiled - this->angularVelocityProfiled * ADW_MM/1000.0 / 2.0;
        RightVelocity = this->linearVelocityProfiled + this->angularVelocityProfiled * ADW_MM/1000.0 / 2.0;

        // Left&Right Motor [m/s]&[m/s]
        this->leftVelocityProfiled  = LeftVelocity;
        this->rightVelocityProfiled = RightVelocity;

        // Left&Right Wheel [m/s]&[m/s] to Left&Right Motor [rot/s]&[rot/s]
        LeftVelocity  = LeftVelocity  * 1000.0 / (RATIO * WD_MM * _PI_);
        RightVelocity = RightVelocity * 1000.0 / (RATIO * WD_MM * _PI_);

        // Always positive (PositionControl handle direction)
        LeftVelocity  = this->abs(LeftVelocity);
        RightVelocity = this->abs(RightVelocity);

        this->leftMotor->SetSpeedStep((uint32_t)(LeftVelocity * PG_USTEP));
        this->rightMotor->SetSpeedStep((uint32_t)(RightVelocity * PG_USTEP));


        /*if(this->leftVelocityProfiled != 0.0 || this->rightVelocityProfiled != 0.0)
            printf("%.3f\t%.3f\r\n", this->leftVelocityProfiled, this->rightVelocityProfiled);*/
    }

    /**
     * @brief  VelocityControl generate
     */
    void ProfileGenerator::Generate(float32_t period)
    {
        float32_t angularVelocityProfiled = 0.0;
        float32_t linearVelocityProfiled  = 0.0;
        float32_t velocityProfiled  = 0.0;

        float32_t time = getTime();

        float32_t d = 0.0;

        // #1 : Compute Profile
        angularVelocityProfiled = this->angularProfile.Get(time);
        linearVelocityProfiled  = this->linearProfile.Get(time);

        // Set Velocity profiled
        this->angularVelocityProfiled = angularVelocityProfiled;
        this->linearVelocityProfiled  = linearVelocityProfiled;

        // #2 : Profile to Profile
        // Angular profile
        switch (this->angularPhaseProfile)
        {
            case Zero:
                break;

            case Acc:
                break;

            case ConstVel:
                break;

            case Dec:
                break;

            case AccDec:
                //this->angularProfile.SetPoint(this->angularPosition);
                if(this->angularProfile.isFinished())
                {
                    this->angularPhaseProfile = Zero;
                }
                break;

            default:
                break;
        }

        // Linear profile
        switch (this->linearPhaseProfile)
        {
            case Zero:
                break;

            case Acc:
                break;

            case ConstVel:
                break;

            case Dec:
                break;

            case AccDec:
                //this->linearProfile.SetPoint(this->linearPosition);
                if(this->linearProfile.isFinished())
                {
                    this->linearPhaseProfile = Zero;
                }
                break;

            default:
                break;
        }

        this->Finished = this->angularProfile.isFinished() && this->linearProfile.isFinished();
    }

    void ProfileGenerator::taskHandler(void* obj)
    {
        TickType_t xLastWakeTime;
        TickType_t xFrequency = pdMS_TO_TICKS(PG_TASK_PERIOD_MS);

        ProfileGenerator* instance = _profileGenerator;
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

    float32_t ProfileGenerator::getTime()
    {
        float32_t time = 0.0;

        time = static_cast<float32_t>(xTaskGetTickCount());
        time /= 1000.0;

        return  time;
    }


    float32_t ProfileGenerator::abs(float32_t val)
    {
        if(val < 0.0)
            val = -val;
        return val;
    }
}
