#ifndef MP_DISCRETE_PLANNER_HPP_
#define MP_DISCRETE_PLANNER_HPP_

#include "MP/MPScene.hpp"
#include "MP/RobotPlanner.hpp"
#include "MP/MPMultiSearch.hpp"
#include "MP/MPCoopAStar.hpp"
#include "MP/MPPushAndSwap.hpp"
#include "MP/MPSIPP.hpp"
#include "Utils/GraphSearch.hpp"
#include <vector>
#include <algorithm>

namespace MP
{
  class MPDiscretePlanner
  {
  public:

    MPDiscretePlanner(void)
    {
      m_pos = 0;
      m_robotType = 0;
    }

    virtual ~MPDiscretePlanner(void)
    {
    }

    virtual void CompleteSetup();
    virtual void Run(const int nrIters);
    virtual void Draw(void);
    virtual void MoveOneStep();
    MPScene                     m_scene;
    std::vector<RobotPlanner*>  m_robots;
    MPAbstraction       *m_abstract;
    MPMultiSearch *m_multiSearch;
    int m_pos;
    int m_robotType;
    int m_searchType;

  };
}

#endif
