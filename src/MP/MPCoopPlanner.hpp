#ifndef MP_COOP_PLANNER_HPP_
#define MP_COOP_PLANNER_HPP_

#include "MP/MPProximityPlanner.hpp"
#include "MP/MPScene.hpp"
#include "MP/RobotPlanner.hpp"
#include "MP/MPMultiSearch.hpp"
#include "MP/MPCoopAStar.hpp"
#include "MP/MPSIPP.hpp"
#include "MP/MPPushAndSwap.hpp"
#include "Utils/Geometry.hpp"

namespace MP
{
  class MPCoopPlanner: public MPProximityPlanner
  {
  public:
    enum Type
    {
      COOPASTART  = 0,
      PAS         = 1,
      SIPP        = 2
    };

    MPCoopPlanner();
    virtual ~MPCoopPlanner(void)
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
      std::vector<double*>      m_groupCfg;
      std::vector<MPState*>     m_groupState;
      std::vector<int>          m_groupid;
      double                    m_weight;
    };

    struct Group
    {
      std::vector<int>              m_id;
      std::vector<int>              m_vids;
      double                        m_weight;
      std::vector<std::vector<int>> m_pathsToGoal;

    };

    virtual void  CompleteSetup();
    virtual void  Run(const int nrIters);
    virtual MPCoopPlanner::Group* SelectGroup(void);
    virtual int   SelectTargetAndVertex(Group * g);
    virtual bool  ExtendFrom(Group * g, const int vid);

    virtual void  Initialize(void);
    virtual void  AddGroupData(const int vid);
    virtual void  GetPaths(void);
    virtual int   AddGroupVertex(GroupVertex * const v);
    virtual bool  AllRobotsAtGoals(void)
    {
      for (int i = 0; i< m_robots.size(); i++)
      {
        if(m_robots[i]->HasReachGoal() == false)
        {
          return false;
        }
      }
      return true;
    }
    virtual void PermuteRobots(std::vector<int> &robotorder)
    {
      //PermuteItem
      robotorder.clear();
      for (int r = 0; r < m_robots.size(); r++)
      {
        robotorder.push_back(r);
      }

      PermuteItems<int>(&robotorder,m_robots.size());
    }

    virtual void Draw();
    virtual bool AreValidStates();
    virtual bool IsSolved(void)
    {
      return GetSolved() > 0;
    }

    virtual void  DrawGroupVertex(void);
    virtual void  DrawCfgPaths(void);

    MPScene                       m_scene;
    std::vector<RobotPlanner*>    m_robots;
    MPAbstraction                *m_abstract;
    MPMultiSearch                *m_discreteSearch;
    // MPCoopAStar                   m_discreteSearch;
    // MPPushAndSwap                   m_discreteSearch;


    double                            m_probSelectNearestVertex;
    double                            m_dsel;

    std::vector<GroupVertex*>         m_vertices;
    std::vector<int>                  m_path;
    UseMap(std::vector<int>, Group*)  m_groups;

    int             m_robotType;
    int             m_searchType;
    int             m_depthSearch;

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

  };
}

#endif
