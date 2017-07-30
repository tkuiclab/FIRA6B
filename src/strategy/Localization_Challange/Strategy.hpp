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
    void StrategyLocalization2();
    void Forward();
    void Turn();
    void Chase();
    void OptimatePath();
    void EraseElement(std::vector<int>&,int);
    void MinAngle(std::vector<int>&,std::vector<int>&,int);
    int ThroughPath(int,int);
    void Normalization(double &);
    int _LocationState;
    int _CurrentTarget;
    int _Last_state;
    int through_path_ary[5][5];
    TargetStruct _Target;
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
    enum hotizon_location
    {
        up,
        down,
        right,
        left
    };
};
#endif