/**
 * @file    VelocityProfile.cpp
 * @author  Jeremy ROULLAND
 * @date    12 nov. 2016
 * @brief   Motion profile (Trapezoidal, S-Curve, ...)
 */

#include "VelocityProfile.hpp"
#include "common.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

using namespace MotionControl;

namespace MotionControl
{

    VelocityProfile::VelocityProfile(float32_t maxVel, float32_t maxAcc, enum PROFILE profile)
    {
        assert(profile < PROFILE::MPROFILE_MAX);

        this->profile = profile;

        this->mode = MODE_AUTO;

        this->startTime = 0.0;

        this->setPoint   = 0.0;
        this->startPoint = 0.0;

        this->maxVel = maxVel;
        this->maxAcc = maxAcc;

        this->tf = 1.0;

        this->finished = false;
    }

    VelocityProfile::~VelocityProfile()
    {
    }

    bool VelocityProfile::isFinished()
    {
        return this->finished;
    }

    void VelocityProfile::SetPoint(float32_t distance)
    {
    	this->setPoint = distance - this->startPoint;
    }

    void VelocityProfile::SetSetPoint(float32_t distance, float32_t currentTime)
    {
        this->startTime = currentTime;
        this->startPoint = 0;

        this->setPoint = distance - this->startPoint;

        this->finished = false;
    }

    float32_t VelocityProfile::Get(float32_t time)
    {
        float32_t v = 0.0;
        float32_t tf = 0.0;

		tf = calculateMinTime(this->profile);

        v = this->Get(time, tf);

        return v;
    }

    float32_t VelocityProfile::Get(float32_t time, float32_t tf)
    {
        float32_t v = 0.0;
        float32_t t = 0.0;

        t = time - this->startTime;
        assert(t >= 0.0);

        this->tf = tf;
        t /= tf;

        v = this->calculateProfile(t);

        return v;
    }

    float32_t VelocityProfile::calculateProfile(float32_t t)
    {
        float32_t v = 0.0;

        switch (this->profile)
        {
            case NONE:
                v = this->calculateNoneProfile(t);
                break;

            case LINEAR:
                v = this->calculateLinearProfile(t);
                break;

            case POLY5:
                v = this->calculatePolynomial5Profile(t);
                break;

            default:
                v = 1.0;
                break;
        }

        return v;
    }

    float32_t VelocityProfile::calculateMinTime(enum VelocityProfile::PROFILE profile)
    {
        float32_t tf = 0.0, tfVel = 0.0, tfAcc = 0.0;

        switch (profile)
        {
            case LINEAR:
                tfVel = abs(this->setPoint) / this->maxVel;
                tfAcc = 0.0;
                break;

            case POLY5:
                tfVel = (15.0 * abs(this->setPoint)) / (8.0 * this->maxVel);
                tfAcc = sqrt( (10.0 * abs(this->setPoint)) / (sqrt(3) * this->maxAcc) );
                break;

            default:
                tfVel = 0.0;
                tfAcc = 0.0;
                break;
        }

        tf = (tfVel >= tfAcc) ? tfVel : tfAcc;

        return tf;
    }

    float32_t VelocityProfile::calculateMinDist(enum VelocityProfile::PROFILE profile)
    {
    	float32_t d = 0.0;

        switch (profile)
        {
            /*case POLY5_P1:
            case POLY5_P2:
                d = (64.0*sqrt(3.0)/135.0)*(pow(this->maxVel,2)/this->maxAcc);
                break;*/
            default:
                break;
        }
    	return d;
    }

    float32_t VelocityProfile::calculateNoneProfile(float32_t t)
    {
        float32_t v = 0.0;

        v = 0.0;

        this->finished = true;

        v *= this->maxVel;

        return v;
    }

    float32_t VelocityProfile::calculateLinearProfile(float32_t t)
    {
        float32_t v = 0.0;

        if(t <= 1.0)
            v = 1.0;
        else
            //v = 0.0;
            v = 1.0;

        if(t >= 1.0)
            this->finished = true;
        else
            this->finished = false;

        v *= this->maxVel;

        return v;
    }

    float32_t VelocityProfile::calculatePolynomial5Profile(float32_t t)
    {
        float32_t v = 0.0;

        if(t <= 1.0)
            v = 30.0*pow(t,2)-60.0*pow(t,3)+30.0*pow(t,4);
        else
            //v = 0.0;
            v = 0.0;

        if(t >= 1.0)
            this->finished = true;
        else
            this->finished = false;

        v *= this->maxVel / 1.875;

        return v;
    }

    float32_t VelocityProfile::abs(float32_t val)
    {
        if(val < 0.0)
            val = -val;
        return val;
    }

}
