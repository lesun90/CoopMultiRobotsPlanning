#ifndef MP_PRIORITY_PLANNER_HPP_
#define MP_PRIORITY_PLANNER_HPP_

#include "MP/MPProximityPlanner.hpp"
#include "MP/MPEPlanner.hpp"
#include "MP/MPScene.hpp"
#include "MP/RobotPlanner.hpp"

namespace MP
{
  class MPPriorityPlannerRRT : public MPTreePlanner
  {
  public:

    MPPriorityPlannerRRT(void) : MPTreePlanner()
    {
      m_vidSolved = -1;
      m_probSteer = 1;
      m_dtol      = 5;
      m_robotType = 0;
    }

    virtual ~MPPriorityPlannerRRT(void)
    {

    }

    std::vector<std::vector<double*>> m_reserveTable;

    virtual void CompleteSetup();
    virtual void Run(const int nrIters);
    virtual void ReadyForNewRobot(void);
    virtual void GetRobotID();
    virtual void Initialize(void);
    virtual int  AddVertex(Vertex * const v);
    virtual void GetPath(int robotid);
    virtual bool GetCfgPath(int robotid);
    virtual bool IsSolved(void);
    virtual void Draw();
    virtual int  ExtendFrom(const int vid);
    virtual void ReserveTable(int robotid);
    virtual bool CheckWithReserveTable(int timeid);
    virtual int SelectVertex();

    virtual double getAvgPathCosts(void)
    {
      if (IsSolved() == false)
      {
        return -1;
      }
      double sum = 0;
      for (int r = 0 ; r < m_robots.size(); r++)
      {
        sum+= m_robots[r]->GetCfgPathCost();
      }
      return sum/m_robots.size();
    }

    MPScene                     m_scene;
    std::vector<RobotPlanner*>  m_robots;
    std::vector<int> m_path;
    std::vector<int> m_robotids;
    int m_currentrobotid;
    int             m_robotType;

  };
}

#endif
