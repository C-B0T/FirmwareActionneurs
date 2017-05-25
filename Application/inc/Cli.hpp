/**
 * @file    CLI.hpp
 * @author  Jeremy ROULLAND
 * @date    14 apr. 2017
 * @brief   Command Line Interface class
 */

#ifndef INC_CLI_HPP_
#define INC_CLI_HPP_


#include "common.h"

// For std::string
#include <algorithm>

// MotionControl
#include "Odometry.hpp"
#include "MotionControl.hpp"
#include "TrajectoryPlanning.hpp"
#include "PositionControlStepper.hpp"

#include "Mandible.hpp"

#include "Diag.hpp"

// FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

using namespace Location;
using namespace MotionControl;

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
    * @class CLI
    * @brief
    *
    * HOWTO :
    *
    */

    class CLI
    {
    public:
        /**
         * @brief Get instance method
         * @return Diag instance
         */
        static CLI* GetInstance();

        /**
         * @brief Diag instance name
         */
        std::string Name()
        {
            return this->name;
        }

    protected:
        /**
         * @brief CLI default constructor
         */
        CLI();

        /**
         * @protected
         * @brief Instance name
         */
        std::string name;

        Odometry           *odometry;
        PositionControl    *pc;
        TrajectoryPlanning *tp;
        FBMotionControl    *mc;

        Diag *diag;

        Mandible* man;

        void Start();

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

#endif /* INC_CLI_HPP_ */
