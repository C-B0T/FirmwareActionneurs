/**
 * @file    Diag.cpp
 * @author  Jeremy ROULLAND
 * @date    29 apr. 2017
 * @brief   Diag class
 */

#include "Diag.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Encoder.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define DIAG_TASK_STACK_SIZE          (256u)
#define DIAG_TASK_PRIORITY            (2u)

#define DIAG_TASK_PERIOD_MS           (10u)

#define DIAG_TRACES_PERIOD_MS         (10u)
#define DIAG_LED_PERIOD_MS            (10u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static Diag* _diag = NULL;

/*----------------------------------------------------------------------------*/
/* Class Implementation                                                       */
/*----------------------------------------------------------------------------*/

Diag* Diag::GetInstance()
{
    // If ????? instance already exists
    if(_diag != NULL)
    {
        return _diag;
    }
    else
    {
        _diag = new Diag();
        return _diag;
    }
}

Diag::Diag()
{
    this->name = "Diag";
    this->taskHandle = NULL;

    this->enable[0] = false;
    this->enable[1] = false;
    this->enable[2] = false;
    this->enable[3] = false;
    this->enable[4] = false;

    // Create task
    xTaskCreate((TaskFunction_t)(&Diag::taskHandler),
                this->name.c_str(),
                DIAG_TASK_STACK_SIZE,
                NULL,
                DIAG_TASK_PRIORITY,
                NULL);

    this->odometry = Odometry::GetInstance(false);
    this->pc = PositionControl::GetInstance(false);
    this->pg = ProfileGenerator::GetInstance(false);
    this->tp = TrajectoryPlanning::GetInstance(false);
    this->mc = FBMotionControl::GetInstance();

    this->led1 = GPIO::GetInstance(GPIO::GPIO0);
    this->led2 = GPIO::GetInstance(GPIO::GPIO1);
    this->led3 = GPIO::GetInstance(GPIO::GPIO2);
    this->led4 = GPIO::GetInstance(GPIO::GPIO3);

}

void Diag::TracesMC()
{
    robot_t r;

    this->odometry->GetRobot(&r);

    //printf("%ld\t%ld\t%.3f\t%.3f\t%.3f\t%.3f\r\n", tp->GetStep(), pg->GetLinearPhase(), pg->GetLinearPositionProfiled(), pc->GetLinearPosition(), pg->GetLinearVelocity(), pg->GetLinearSpeed());
    //printf("%.3f\t%.3f\t%.3f\t%.3f\r\n", pc->GetLinearPosition(), pg->GetLinearVelocityProfiled(), odometry->GetLinearVelocity(), odometry->GetLinearPosition());
    //printf("%.3f\t%.3f\t%.3f\t%.3f\r\n", pc->GetAngularPosition(), pg->GetAngularVelocity(), odometry->GetAngularVelocity(), odometry->GetAngularPosition());
    //printf("%ld\t%ld\t%.3f\t%.3f\t%.3f\t%.3f\r\n", tp->GetStep(), pg->GetLinearPhase(), pc->GetAngularPosition(), pg->GetAngularVelocity(), odometry->GetAngularVelocity(), odometry->GetAngularPosition());
    //printf("%.3f\t%.3f\r\n", odometry->GetAngularPosition(), odometry->GetAngularVelocity());
    //printf("%.3f\t%.3f\t%.3f\t%.3f\r\n", odometry->GetLinearPosition(), odometry->GetLinearVelocity(), odometry->GetAngularPosition(), odometry->GetAngularVelocity());
    //printf("%ld\t%.3f\t%.3f\t%.3f\t%.3f\r\n", tp->GetStep(), odometry->GetLinearPosition(), odometry->GetLinearVelocity(), odometry->GetAngularPosition(), odometry->GetAngularVelocity());
    //printf("%ld\t%.3f\t%.3f\t%.3f\t%.3f\t%ld\t%ld\r\n", tp->GetStep(), odometry->GetLinearPosition(), odometry->GetLinearVelocity(), odometry->GetAngularPosition(), odometry->GetAngularVelocity(), odometry->getLeftSum(), odometry->getRightSum());
    printf("%ld\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\r\n", tp->GetStep(), pg->GetLinearVelocityProfiled(), pg->GetAngularVelocityProfiled(), odometry->GetLinearPosition(), odometry->GetLinearVelocity(), odometry->GetAngularPosition(), odometry->GetAngularVelocity());
}

void Diag::TracesOD()
{
    robot_t r;

    this->odometry->GetRobot(&r);

    //printf("%.3f\t%.3f\t%.3f\r\n", r.X, r.Y, r.O);
    printf("%ld\t%ld\t%.1f\r\n", r.Xmm, r.Ymm, r.Odeg);
}

void Diag::Led()
{
	static uint32_t localTime = 0;
	uint16_t status = 0;

	localTime += (uint32_t)DIAG_LED_PERIOD_MS;

	// # Led Direction
	/*static uint32_t LedId = 4;
	switch(LedId)
	{
	case 1:
		this->led1->Toggle();
		break;
	case 2:
		this->led2->Toggle();
		break;
	case 3:
		this->led3->Toggle();
		break;
	case 4:
		this->led4->Toggle();
		break;
	default:
		break;
	}
	LedId--;
	if(LedId < 1)
		LedId = 4;
	*/

    // # Led Status

	// Led1
	if((localTime % 500) == 0)
	    this->led1->Toggle();   // Blinking alive

	status = this->mc->GetStatus();

    // Led2
	if(status & (1<<8)) // Ready
        this->led2->Set(HAL::GPIO::Low);
    else
        this->led2->Set(HAL::GPIO::High);

	// Led3
    if(status & (1<<9)) // Safeguard Flag
    {
        if((localTime % 100) == 0)
            this->led3->Toggle();
    }
    else
        this->led3->Set(HAL::GPIO::High);

    // Led4
    if( ((status & (1<<1)) == 0) && ((status & (1<<0)) == 0) ) // Config
    {
        if((localTime % 100) == 0)
            this->led4->Toggle();
    }
    else if( (status & (1<<1)) == 0 )   // Safeguard config
    {
        if((localTime % 200) == 0)
            this->led4->Toggle();
    }
    else if( (status & (1<<0)) == 0 )   // MC enable config
    {
        if((localTime % 300) == 0)
            this->led4->Toggle();
    }
    else
        this->led4->Set(HAL::GPIO::High);
}

void Diag::Compute(float32_t period)
{
	static uint32_t localTime = 0;

	localTime += (uint32_t)DIAG_TASK_PERIOD_MS;


	if((localTime % DIAG_LED_PERIOD_MS) == 0)
	{
		this->Led();
	}

	if((localTime % DIAG_TRACES_PERIOD_MS) == 0)
	{
		if(this->enable[0])
			this->TracesMC();
		if(this->enable[1])
			this->TracesOD();
	}
}

void Diag::taskHandler (void* obj)
{
    TickType_t xLastWakeTime;
    TickType_t xFrequency = pdMS_TO_TICKS(DIAG_TASK_PERIOD_MS);

    Diag* instance = _diag;
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

        //4. Compute Diag informations
        instance->Compute(period);

        // 5. Set previous tick
        prevTick = tick;
    }
}
