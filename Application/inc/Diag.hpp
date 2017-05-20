/**
 * @file    Diag.hpp
 * @author  Jeremy ROULLAND
 * @date    29 apr. 2017
 * @brief   Diag class
 */

#ifndef INC_DIAG_HPP_
#define INC_DIAG_HPP_


#include "common.h"

// For std::string
#include <algorithm>

// MotionControl
#include "Odometry.hpp"
#include "MotionControl.hpp"
#include "TrajectoryPlanning.hpp"
#include "PositionControlStepper.hpp"

// LED
#include "GPIO.hpp"

// FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

using namespace Location;
using namespace MotionControl;
using namespace HAL;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

typedef void (*FunctionFunc)();


/*----------------------------------------------------------------------------*/
/* Class declaration                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @namespace Utils
 */

    /**
    * @class DIAG
    * @brief
    *
    * HOWTO :
    *
    */

    class Diag
    {
    public:
        /**
         * @brief Get instance method
         * @return Diag instance
         */
        static Diag* GetInstance();

        /**
         * @brief Diag instance name
         */
        std::string Name()
        {
            return this->name;
        }

        void Toggle(uint16_t i = 0)
        {
        	this->enable[i] = ! this->enable[i];
        }

    protected:
        /**
         * @brief DIAG default constructor
         */
        Diag();

        /**
         * @protected
         * @brief Instance name
         */
        std::string name;

        bool enable[5];

        Odometry           *odometry;
        PositionControl    *pc;
        TrajectoryPlanning *tp;
        FBMotionControl    *mc;

        GPIO *led1;
        GPIO *led2;
        GPIO *led3;
        GPIO *led4;


        void TracesMC();
        void TracesOD();
        void Led();

        /**
         * @brief Compute traces spreading
         */
        void Compute(float32_t period);


        /**
         * @protected
         * @brief OS Task handle
         */
        TaskHandle_t taskHandle;

        /**
         * @protected
         * @brief loop task handler
         * @param obj : Always NULL
         */
        void taskHandler (void* obj);

    };

#endif /* INC_DIAG_HPP_ */
