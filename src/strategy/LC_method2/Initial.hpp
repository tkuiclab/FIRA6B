/**
 * @file Strategy.hpp
 *
 * @brief Localization challange localization class
 *
 * @date July 2017
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef _INITIAL_HPP_
#define _INITIAL_HPP_
/*****************************************************************************
** Includes
*****************************************************************************/
#include "ros/ros.h"
#include "NodeHandle.hpp"
#include "Env.hpp"
Environment InitData(Environment Env)
{
    Env.Robot.v_x = 0;
    Env.Robot.v_y = 0;
    Env.Robot.v_yaw = 0;
    Env.Robot.pos.x = 0;
    Env.Robot.pos.y = 0;
    Env.Robot.pos.angle = 0;
    Env.Robot.pos.distance = 0;
    Env.Robot.ball.x = 0;
    Env.Robot.ball.y = 0;
    Env.Robot.ball.angle = 0;
    Env.Robot.ball.distance = 0;
    Env.Robot.goal.x = 0;
    Env.Robot.goal.y = 0;
    Env.Robot.goal.angle = 0;
    Env.Robot.goal.distance = 0;
    Env.Robot.op_goal.x = 0;
    Env.Robot.op_goal.y = 0;
    Env.Robot.op_goal.angle = 0;
    Env.Robot.op_goal.distance = 0;
    Env.GameState = 0;
    Env.SaveParam = 1;
    return Env;
}
#endif