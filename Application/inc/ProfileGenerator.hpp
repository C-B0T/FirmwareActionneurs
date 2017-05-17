/**
 * @file    ProfileGenerator.hpp
 * @author  Jeremy ROULLAND
 * @date    16 may 2017
 * @brief   ProfileGenerator class
 */

 #ifndef INC_PROFILEGENERATOR_HPP_
 #define INC_PROFILEGENERATOR_HPP_

 #include "HAL.hpp"
 #include "Utils.hpp"
 #include "Odometry.hpp"
 #include "PositionControlStepper.hpp"
 #include "VelocityProfile.hpp"
#include "DRV8813.hpp"

 // FreeRTOS
 #include "FreeRTOS.h"
 #include "task.h"
 #include "timers.h"

 using namespace MotionControl;

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

     // Profile generator
     struct pg_profile
     {
         float32_t    maxVel;
         float32_t    maxAcc;
         enum VelocityProfile::PROFILE profile;
     }Profile_Angular;

     struct pg_profile Profile_Linear;
 }PG_DEF;


 /*----------------------------------------------------------------------------*/
 /* Class declaration                                                          */
 /*----------------------------------------------------------------------------*/

 /**
  * @namespace MotionControl
  */
 namespace MotionControl
 {
     /**
      * @class ProfileGenerator
      * @brief Profile generator class
      *
      * HOWTO :
      * -
      *
      */
     class ProfileGenerator
     {
     public:

         /**
          * @brief Profile Identifier List
          */
         enum ID
         {
             ANGULAR = 0,    //!< ANGULAR
             LINEAR,         //!< LINEAR
             PROFILE_MAX     //!< PROFILE_MAX
         };

         /**
          * @brief Profile Phase List
          */
         enum Phase
         {
             Zero = 0,    //!< Initial state
             Acc,         //!< Acc
             ConstVel,    //!< ConstVel
             Dec,         //!< Dec
             AccDec,      //!< AccDec
             Phase_MAX    //!< Phase_max
         };

         /**
          * @brief Get instance method
          * @return Profile loop instance
          */
         static ProfileGenerator* GetInstance(bool standalone = true);

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
          * @brief start linear velocity setpoint
          */
         void StartLinearVelocity(float32_t distance);

         /**
          * @brief Start angular velocity setpoint
          */
         void StartAngularVelocity(float32_t angle);

         /**
          * @brief Get linear velocity setpoint
          */
         float32_t GetLinearVelocityProfiled()
         {
             return this->linearVelocityProfiled;
         }

         /**
          * @brief Get angular velocity setpoint
          */
         float32_t GetAngularVelocityProfiled()
         {
             return this->angularVelocityProfiled;
         }

         void SetLinearVelMax(float32_t velMax)
         {
        	 this->linearProfile.SetVelMax(velMax);
         }

         void SetLinearAccMax(float32_t accMax)
         {
        	 this->linearProfile.SetAccMax(accMax);
         }

         void SetAngularVelMax(float32_t velMax)
         {
        	 this->angularProfile.SetVelMax(velMax);
         }

         void SetAngularAccMax(float32_t accMax)
         {
        	 this->angularProfile.SetAccMax(accMax);
         }

         /**
          * @brief Compute profile control
          */
         void Compute(float32_t period);

         /**
          * @brief is angular and linear velocitying finished
          */
         bool isPositioningFinished()
         {
             return this->Finished;
         }


         /**
          * @brief get phase profile
          */
         uint32_t GetAngularPhase()
         {
             return (uint32_t)this->angularPhaseProfile;
         }

         /**
          * @brief get phase profile
          */
         uint32_t GetLinearPhase()
         {
             return (uint32_t)this->linearPhaseProfile;
         }

     protected:

         /**
          * @protected
          * @brief Private constructor
          */
         ProfileGenerator(bool standalone);

         /**
          * @brief Generate profile control
          */
         void Generate(float32_t period);

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
          * @brief angular profile generator
          */
         VelocityProfile angularProfile;

         /**
          * @protected
          * @brief linear profile generator
          */
         VelocityProfile linearProfile;

         /**
          * @protected
          * @brief left motor driver
          */
         HAL::Drv8813* leftMotor;

         /**
          * @protected
          * @brief right motor driver
          */
         HAL::Drv8813* rightMotor;


         /**
          * @protected
          * @brief Coef definitions
          */
         PG_DEF def;

         /**
          * @protected
          * @brief Odometry instance
          */
         Location::Odometry* odometry;

         /**
          * @protected
          * @brief Profile finished
          */
         bool Finished;

         /**
          * @protected
          * @brief Angular velocity required
          */
         float32_t angularPosition;

         /**
          * @protected
          * @brief Linear velocity required
          */
         float32_t linearPosition;

         /**
          * @protected
          * @brief Angular velocity
          */
         float32_t angularVelocity;

         /**
          * @protected
          * @brief Linear velocity
          */
         float32_t linearVelocity;

         /**
          * @protected
          * @brief Angular velocity profiled
          */
         float32_t angularVelocityProfiled;

         /**
          * @protected
          * @brief Linear velocity profiled
          */
         float32_t linearVelocityProfiled;

         /**
          * @protected
          * @brief Left velocity profiled
          */
         float32_t leftVelocityProfiled;

         /**
          * @protected
          * @brief Right velocity profiled
          */
         float32_t rightVelocityProfiled;

         /**
          * @protected
          * @brief Angular phase of the profile
          */
         enum Phase angularPhaseProfile;

         /**
          * @protected
          * @brief Linear phase of the profile
          */
         enum Phase linearPhaseProfile;

         /**
          * @protected
          * @brief OS Task handle
          *
          * Used by velocity control loop
          */
         TaskHandle_t taskHandle;

         /**
          * @protected
          * @brief Velocity control loop task handler
          * @param obj : Always NULL
          */
         void taskHandler (void* obj);

         /**
          * @protected
          * @brief get absolute value
          */
         float32_t abs(float32_t val);

     };
}
#endif /* INC_PROFILEGENERATOR_HPP_ */
