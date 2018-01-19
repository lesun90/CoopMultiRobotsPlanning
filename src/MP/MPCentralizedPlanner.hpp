#ifndef MP_CENTRALIZED_PLANNER_HPP_
#define MP_CENTRALIZED_PLANNER_HPP_

#include "MP/MPProximityPlanner.hpp"
#include "MP/MPScene.hpp"
#include "MP/RobotPlanner.hpp"

namespace MP
{
  class MPCentralizedPlanner: public MPProximityPlanner
  {
  public:

    MPCentralizedPlanner(void): MPProximityPlanner()
    {
      m_vidSolved = -1;
      m_probSteer = 1;
      m_dtol      = 5;
      m_robotType = 0;
    }

    virtual ~MPCentralizedPlanner(void)
    {
    }

    class GroupVertex
    {
    public:
      GroupVertex(void)
      {
        m_parent = Constants::ID_UNDEFINED;
        m_vid    = Constants::ID_UNDEFINED;
      }

      virtual ~GroupVertex(void)
      {
      }

      int                       m_parent;
      int                       m_vid;
      std::vector<MPState*>     m_groupState;
      std::vector<double*>      m_groupCfg;
    };

    virtual void CompleteSetup();
    virtual void Draw();
    virtual void Initialize(void);
    virtual int AddGroupVertex(GroupVertex * const v);
    virtual int ExtendFrom(const int vid);

    virtual bool ReachGoals(void);

    virtual bool AreValidStates(void);
    virtual bool GroupReachTarget();
    virtual int SelectGroupVertex();
    virtual void GenerateTargets();
    virtual void DrawGroupVertex(void);
    virtual void Run(const int nrIters);
    virtual void GetPaths(void);
    virtual void DrawCfgPaths(void);
    virtual bool IsSolved(void)
    {
      return GetSolved() > 0;
    }
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
    MPScene                       m_scene;
    std::vector<RobotPlanner*>  m_robots;
    std::vector<GroupVertex*>   m_vertices;
    std::vector<int> m_path;
    int             m_robotType;

  };
}

#endif
