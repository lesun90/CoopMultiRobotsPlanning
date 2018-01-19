#ifndef MP_GPCONSTANTS_HPP_
#define MP_GPCONSTANTS_HPP_

#include "Utils/Constants.hpp"
#include <cmath>

namespace MP
{
  namespace Constants
  {
    const bool   PARAM_TRI_DECOMP_USECONFORM= false;
    const double PARAM_TRI_DECOMP_ANGLE     = 20;
    const double PARAM_TRI_DECOMP_AREA      = 100;
    const double PARAM_TRI_MINAREA          = 0.5;

    const int    ABSTRACTION_NRNEIGHS              = 5;
    const int    ABSTRACTION_NRSAMPLE              = 1000;
    const double ABSTRACTION_MINDIST              = 10;
    const double ABSTRACTION_MAXDIST              = 12;

    const double OBSTACLES_CLEARANCE = 1;

    const double PLANNER_DIST_TOL = 5;
    const double DEF_HEIGHT = 0.4;


    const double SCENE_OBSTACLES_HEIGHT     = 2.0;
    const double SCENE_DRAWZ_ROOM_BOX       = 0.01;
    const double SCENE_DRAWZ_GOAL_ID        = 0.04;

    const double DECOMPOSITION_MIN_AREA_TO_ADD = 0.05;
    const int    DECOMPOSITION_COV_GRID_DIMX = 128;
    const int    DECOMPOSITION_COV_GRID_DIMY = 128;

    const double SIMULATOR_TIME_STEP             = 0.05;
    const double SIMULATOR_MIN_DISTANCE_ONE_STEP = 0.5;
    const double SIMULATOR_MAX_DISTANCE_ONE_STEP = 0.8;

    const double ROBOT_MIN_STEER_ANGLE     = -60 * Constants::DEG2RAD;
    const double ROBOT_MAX_STEER_ANGLE     =  60 * Constants::DEG2RAD;
    const double ROBOT_MIN_VELOCITY        = -8.0;
    const double ROBOT_MAX_VELOCITY        =  8.0;
    const double ROBOT_MIN_ACCELERATION    = -2.0;
    const double ROBOT_MAX_ACCELERATION    =  2.0;
    const double ROBOT_MIN_STEER_VELOCITY  = -4.0;
    const double ROBOT_MAX_STEER_VELOCITY  =  4.0;
    const double ROBOT_LENGTH         =  4.0;
    const double ROBOT_WIDTH          =  1.5;
    const double ROBOT_HEIGHT         =  1.5;
    const double ROBOT_WHEEL_RADIUS        =  0.5;
    const double ROBOT_WHEEL_HEIGHT        =  0.4;

    const double FOLLOW_WEIGHT_BASE = 100000000.0;
    const double FOLLOW_REACH_TOLERANCE = 5.0;
    const double NORMAL_REACH_TOLERANCE = 3.0;

    const double PLANNER_SELECTION_PENALTY    = 0.8;
    const double PLANNER_PROBABILITY_SELECT_NEAREST_VERTEX = 0.9;
    const int    PLANNER_EXTEND_MIN_NR_STEPS  = 40;
    const int    PLANNER_EXTEND_MAX_NR_STEPS  = 80;

    const int PARAM_PLANNER_MIN_NR_ITERS_EXPAND_ACTION = 50;
    const int PARAM_PLANNER_MAX_NR_ITERS_EXPAND_ACTION = 150;

    const double PARAM_NEAR_TOL = 5;


  }
}

#endif
