/**
 * @file    Cylinder.hpp
 * @author  Jeremy ROULLAND
 * @date    20 may 2017
 * @brief   Cylinder (Barillet) class
 */


#ifndef INC_CYLINDER_HPP_
#define INC_CYLINDER_HPP_

#include "common.h"

#include "DRV8813.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

typedef struct
{
    HAL::Drv8813::ID    ID_motor;
    HAL::Drv8813::ID    ID_motorRise;
    HAL::PWM::ID       ID_motorOpen[2];
    HAL::GPIO::ID       ID_topz;
    uint16_t            indexMax;
    bool                shortPath;
    bool                canRise;
    float32_t           ratio;
    uint16_t            speed;
}CYL_DEF;

/*----------------------------------------------------------------------------*/
/* Class declaration                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @class
 * @brief
 *
 * HOWTO :
 * -
 *
 */
class Cylinder
{
public:

    /**
     * @brief Cylinder Identifier List (Barillet Haut Bas)
     */
    enum ID
    {
        CYLINDER0,  //!< CYLINDER_BOTTOM
        CYLINDER1,  //!< CYLINDER_TOP
        CYLINDER_MAX
    };

    /**
     * @brief Get instance method
     */
    static Cylinder* GetInstance(Cylinder::ID id);

    int8_t Open();

    int8_t Close();

    int8_t Raise();

    int8_t Lower();

    int8_t Goto(int8_t index);

    int8_t SearchRefPoint();

    bool IsPositioningFinished();

protected:
    /**
     * @protected
     * @brief Cylinder constructor
     */
     Cylinder(Cylinder::ID id);

     /**
      * @protected
      * @brief private definitions
      */
      Cylinder::ID id;

     /**
      * @protected
      * @brief private definitions
      */
     int8_t index;

     /**
      * @protected
      * @brief private definitions
      */
     CYL_DEF def;

     /**
      * @protected
      * @brief private definitions
      */
     HAL::Drv8813* motor;

     /**
      * @protected
      * @brief private definitions
      */
     HAL::Drv8813* motorRise;

     /**
      * @protected
      * @brief private definitions
      */
     HAL::PWM* motorOpen[2];

     /**
      * @protected
      * @brief private definitions
      */
     HAL::GPIO* topz;
};



#endif /* INC_CYLINDER_HPP_ */
