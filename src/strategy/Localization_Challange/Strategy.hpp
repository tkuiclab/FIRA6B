/**
 * @file Strategy.hpp
 *
 * @brief Localization challange pathplan strategy
 *
 * @date July 2017
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef _STRATEGY_HPP_
#define _STRATEGY_HPP_
/*****************************************************************************
** Includes
*****************************************************************************/
#include "../common/BaseNode.h"
#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include <math.h>
#include "Env.hpp"
#include <iomanip>
/*****************************************************************************
** Define
*****************************************************************************/

class Strategy
{
  public:
    ///         public member           ///
    ///         constructor             ///
    Strategy();
    virtual ~Strategy() {}
    void setEnv(Environment *Env) { _Env = Env; }
    void setParam(Parameter *);
    void GameState(int);
    void setLocationPoint(LocationStruct *LocationPoint) { _Location = LocationPoint; }
    Environment getEnv() { return *_Env; }

    private:
    ///         private member          ///
    void StrategyHalt();
    void StrategyLocalization();
    void Forward(RobotData, int &, int &, int);
    void Back(int);
    void Turn();
    void Chase();
    void OptimatePath();
    int _LocationState;
    int _CurrentTarget;
    int _Last_state;
    LocationStruct *_Location;
    Environment *_Env;
    Parameter *_Param;
    void showInfo(double,double,double);
    enum state_location
    {
        forward,
        back,
        finish,
        chase,
        turn,
        error
    };
};
#endif