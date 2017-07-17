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
#include "Env.hpp"
/*****************************************************************************
** Define
*****************************************************************************/
#define STATE_HALT                       0
#define STATE_LOCALIZATION              20
#define STRATEGY_HALT                    0
#define STRATEGY_LOCALIZATION           1

class Strategy{
public:
    ///         public member           ///
    ///         constructor             ///
    Strategy();
    virtual ~Strategy(){}
    void setEnv(Environment*);
    void GameState(int);
    void setLocationPoint(LocationStruct*); 
    Environment getEnv(){return *_Env;}
private:
    ///         private member          /// 

    void StrategyHalt();
    void StrategyLocalization();
    int _LocationState;
    LocationStruct *_Location;
    Environment *_Env;
};
#endif