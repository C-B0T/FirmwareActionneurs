/**
 * @file    main.cpp
 * @author  Kevin WYSOCKI
 * @date    8 nov. 2016
 * @brief   Main
 */

#include <stdio.h>
#include <stdlib.h>

#include "../../STM32_Driver/inc/stm32f4xx.h"
#include "common.h"
#include "FreeRTOS.h"
#include "task.h"

#include "HAL.hpp"

#include "Diag.hpp"
#include "Cli.hpp"

#include "../../STM32_Driver/inc/stm32f4xx_it.h"

#include "Odometry.hpp"
#include "PositionControlStepper.hpp"

using namespace HAL;
using namespace Utils;


extern "C" void hard_fault_handler_c(unsigned int * hardfault_args)
{
      unsigned int stacked_r0;
      unsigned int stacked_r1;
      unsigned int stacked_r2;
      unsigned int stacked_r3;
      unsigned int stacked_r12;
      unsigned int stacked_lr;
      unsigned int stacked_pc;
      unsigned int stacked_psr;

      stacked_r0 = ((unsigned long) hardfault_args[0]);
      stacked_r1 = ((unsigned long) hardfault_args[1]);
      stacked_r2 = ((unsigned long) hardfault_args[2]);
      stacked_r3 = ((unsigned long) hardfault_args[3]);

      stacked_r12 = ((unsigned long) hardfault_args[4]);
      stacked_lr = ((unsigned long) hardfault_args[5]);
      stacked_pc = ((unsigned long) hardfault_args[6]);
      stacked_psr = ((unsigned long) hardfault_args[7]);

      /*printf ("\r\n\r\n[Hard fault handler - all numbers in hex]\r\n");
      printf ("R0 = %x\r\n", stacked_r0);
      printf ("R1 = %x\r\n", stacked_r1);
      printf ("R2 = %x\r\n", stacked_r2);
      printf ("R3 = %x\r\n", stacked_r3);
      printf ("R12 = %x\r\n", stacked_r12);
      printf ("LR [R14] = %x  subroutine call return address\r\n", stacked_lr);
      printf ("PC [R15] = %x  program counter\r\n", stacked_pc);
      printf ("PSR = %x\r\n", stacked_psr);
      printf ("BFAR = %x\r\n", (*((volatile unsigned long *)(0xE000ED38))));
      printf ("CFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED28))));
      printf ("HFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED2C))));
      printf ("DFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED30))));
      printf ("AFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED3C))));
      printf ("SCB_SHCSR = %x\r\n", SCB->SHCSR);*/

      while (1);
}
void HardFault_Handler(void)
{
__ASM(".extern hard_fault_handler_c");
__ASM("TST LR, #4");
__ASM("ITE EQ");
__ASM("MRSEQ R0, MSP");
__ASM("MRSNE R0, PSP");
__ASM("B hard_fault_handler_c");
}

void BusFault_Handler(void)
{
    while(1);
}
void MemManage_Handler(void)
{
    while(1);
}
void WWDG_IRQHandler(void)
{
    while(1);
}
void UsageFault_Handler(void)
{
    while(1);
}


TaskHandle_t xHandleTraces;

/**
 * @brief Initialize hardware
 */
static void HardwareInit (void)
{
    // Ensure all priority bits are assigned as preemption priority bits.
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

    // Enable all GPIO clock
    RCC_AHB1PeriphClockCmd((RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC |
                            RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF |
                            RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_GPIOH | RCC_AHB1Periph_GPIOI),
                            ENABLE);

    // Enable Timer clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5 | RCC_APB1Periph_TIM7,
                           ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8 | RCC_APB2Periph_SPI1,
                           ENABLE);

    // Enable USART Clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

}

float32_t getTime()
{
    float32_t time = 0.0;

    time = static_cast<float32_t>(xTaskGetTickCount());
    time /= 1000.0;

    return  time;
}

float32_t abso(float32_t val)
{
    if(val < 0.0)
        val = -val;
    return val;
}


void TestEvent (void * obj)
{
	GPIO *led2 = GPIO::GetInstance(GPIO::GPIO1);
	led2->Toggle();
	return;
}

/**
 * @brief Main task handler
 */
void TASKHANDLER_Test (void * obj)
{
	uint32_t i=0;
	static uint32_t cpt1=0,cpt2=0;
    static float32_t cpt1f=0.0,cpt2f=0.0;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100u);

    // Get instances
//    HAL::GPIO *led1 = HAL::GPIO::GetInstance(HAL::GPIO::GPIO0);
//    HAL::GPIO *led2 = HAL::GPIO::GetInstance(HAL::GPIO::GPIO1);
//    HAL::GPIO *led3 = HAL::GPIO::GetInstance(HAL::GPIO::GPIO2);
//    HAL::GPIO *led4 = HAL::GPIO::GetInstance(HAL::GPIO::GPIO3);

//	MotionControl::VelocityControlStepper* vc = MotionControl::VelocityControlStepper::GetInstance();
//    MotionControl::PositionControl * pc = MotionControl::PositionControl::GetInstance();
//    MotionControl::ProfileGenerator * pg = MotionControl::ProfileGenerator::GetInstance();
//    MotionControl::TrajectoryPlanning * tp = MotionControl::TrajectoryPlanning::GetInstance();

	Drv8813* drv1 = Drv8813::GetInstance(Drv8813::DRV8813_4);
	Drv8813* drv2 = Drv8813::GetInstance(Drv8813::DRV8813_1);
/*
	Encoder* e1 = Encoder::GetInstance(Encoder::ENCODER0);
	Encoder* e2 = Encoder::GetInstance(Encoder::ENCODER1);
*/
	Location::Odometry* odo = Location::Odometry::GetInstance();



	// TODO PHASE INIT
    drv1->SetSpeedStep(0);
    drv2->SetSpeedStep(0);
    drv1->SetDirection(Drv8813State::DISABLED);
    drv2->SetDirection(Drv8813State::DISABLED);

    vTaskDelay(1000);

    odo->Reset();

    /*drv1->SetDirection(Drv8813State::FORWARD);
    drv2->SetDirection(Drv8813State::BACKWARD);
    drv1->SetSpeedStep(50);
    drv2->SetSpeedStep(50);
    drv1->PulseRotation(200);
    drv2->PulseRotation(200);*/

//	vc->SetLinearVelocity(0.2);
//	drv2->SetSpeedStep(100);
//    drv2->PulseRotation(152);

//    pc->SetAngularPosition(3.14);
//    pc->Test();
	//pc->SetLinearPosition(0.5);

    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        //led1->Toggle();
        //led2->Toggle();
        //led3->Toggle();
        //led4->Toggle();

        //vc->Compute(xFrequency);
        //drv1->SetSpeedRPS(2.0);
    	//drv2->SetSpeedRPS(2.0);

        //pc->SetAngularPosition(3.14 - odo->GetAngularPosition());
        //pc->Test();

        /*tp->goLinear(0.5);

        td = getTime();

        while(t < 1.0)
        {
            vTaskDelayUntil(&xLastWakeTime, 100);
            t = getTime() - td;
            t /= tf;

            // [m/s]
            //vel1 = 10*pow(t,3) - 15*pow(t,4) + 6*pow(t,5);
            vel1 = 30*pow(t,2) - 60*pow(t,3) + 30*pow(t,4);
            vel2 = 30*pow(t,2) - 60*pow(t,3) + 30*pow(t,4);

            //vel1 *= 0.5;
            vel1 *= 0.2 / 1.875;
            vel2 *= 0.2 / 1.875;

            // [m/s] to [rot/s]
            vel1 = vel1 * 1000.0 / 247.607105257;
            vel2 = vel2 * 1000.0 / 247.607105257;

            drv1->SetSpeedStep((uint32_t)(vel1*200.0));
            drv2->SetSpeedStep((uint32_t)(vel2*200.0));

            //printf("%.3f\t%.3f\r\n", vel1, vel2);
        }

        while(1) vTaskDelayUntil(&xLastWakeTime, xFrequency);*/

        //pc->SetLinearPosition(0.5);
        //pg->StartLinearVelocity(0.5);
        /*pc->SetAngularPosition(3.14);
        pg->StartAngularVelocity(3.14);
        while(1) vTaskDelayUntil(&xLastWakeTime, xFrequency);*/

        /*drv1->SetDirection(Drv8813State::BACKWARD);
        drv2->SetDirection(Drv8813State::FORWARD);
        drv1->SetSpeedStep(200);
        drv2->SetSpeedStep(200);
        drv1->PulseRotation(cpt1);
        drv2->PulseRotation(cpt2);*/

        /*cpt1f = 1.0;
        cpt2f = 1.0;

        drv1->SetSpeedRPS(cpt1f);
        drv2->SetSpeedRPS(cpt2f);*/
    }
}


/**
 * @brief Main
 */
int main(void)
{
    HardwareInit();

    // Start (Led init and set up led1)
    HAL::GPIO *led1 = HAL::GPIO::GetInstance(HAL::GPIO::GPIO0);
    HAL::GPIO *led2 = HAL::GPIO::GetInstance(HAL::GPIO::GPIO1);
    HAL::GPIO *led3 = HAL::GPIO::GetInstance(HAL::GPIO::GPIO2);
    HAL::GPIO *led4 = HAL::GPIO::GetInstance(HAL::GPIO::GPIO3);
    led1->Set(GPIO::State::Low);

    //Set External DAC
    ExtDAC* dac = ExtDAC::GetInstance(ExtDAC::EXTDAC0);
    dac->SetOutputValue(ExtDAC::ExtDAC_Channel0,20u);

    Drv8813* drv1 = Drv8813::GetInstance(Drv8813::DRV8813_1);
	Drv8813* drv2 = Drv8813::GetInstance(Drv8813::DRV8813_4);

	Encoder* e1 = Encoder::GetInstance(Encoder::ENCODER0);
	Encoder* e2 = Encoder::GetInstance(Encoder::ENCODER1);

	MotionControl::FBMotionControl * mc = MotionControl::FBMotionControl::GetInstance();
    Location::Odometry* odo = Location::Odometry::GetInstance();

	//mc->Disable();
	//MotionControl::PositionControl * pc = MotionControl::PositionControl::GetInstance();
	//pc->Enable();

	// Serial init
    Serial *serial0 = Serial::GetInstance(Serial::SERIAL0);

    Diag *diag = Diag::GetInstance();
    CLI  *cli  = CLI::GetInstance();

    // Welcome
    printf("\r\n\r\nSirius[B] Firmware Actionneurs V1.0 (" __DATE__ " - " __TIME__ ")\r\n");

    // Create Test task
    xTaskCreate(&TASKHANDLER_Test,
                "Test Task",
                256,
                NULL,
                3,
                NULL);


    vTaskStartScheduler();

    assert(0 == 1);

    return 0;
}

/**
 * @brief Assertion failed callback
 * @param file : File name where assertion occured
 * @param line : Line where assertion occured
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    //printf(ASSERT_FAILED_MESSSAGE, file, line);

    while(1)
    {

    }
}

/**
 * @brief FreeRTOS Tick Hook
 */
void vApplicationTickHook(void)
{
    // Do something
}

/**
 * @brief FreeRTOS Idle Hook
 */
void vApplicationIdleHook(void)
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}

/**
 * @brief FreeRTOS Malloc Failed Hook
 */
void vApplicationMallocFailedHook(void)
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */

    taskDISABLE_INTERRUPTS();
    printf("ERROR | Safe malloc failed !\n");
    for( ;; );
}

/**
 * @brief FreeRTOS Stack Overflow Hook
 */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
    printf("ERROR | %s Task Stack Overflowed !\n", pcTaskName);
    for( ;; );
}
