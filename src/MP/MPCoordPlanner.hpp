#ifndef MP_COORD_PLANNER_HPP_
#define MP_COORD_PLANNER_HPP_

#include "MP/MPScene.hpp"
#include "MP/RobotPlanner.hpp"
#include "MP/MPAbstraction.hpp"
#include "MP/MPTriAbstraction.hpp"
#include "MP/MPPrmAbstraction.hpp"
#include "MP/MPMultiSearch.hpp"
#include "MP/MPCoopAStar.hpp"
#include "MP/MPPushAndSwap.hpp"
namespace MP
{
  class MPCoordPlanner
  {
  public:
    MPCoordPlanner();
    virtual ~MPCoordPlanner(void)
    {
    }

    struct GroupData
    {
      std::vector<MPState*>               m_groupState;
      std::vector<double*>                m_groupCfg;
      std::vector<std::vector<MPState*>>  m_statePaths;
      std::vector<std::vector<double*>>   m_cfgPaths;
    };

    struct Group
    {
      std::vector<int>                    m_id;
      std::vector<GroupData>              m_groupData;
      double                              m_weight;
      std::vector<std::vector<int>>       m_pathsToGoal;
      std::vector<int>                    m_robotOrder;
    };

    virtual void  CompleteSetup();
    virtual void  Draw();
    virtual void  Run(const int nrIters);
    virtual void  Initialize(void);
    virtual void  AddGroupData(std::vector<std::vector<MPState*>> statePath,std::vector<std::vector<double*>> cfgPaths);
    virtual bool  IsSolved();

    virtual void UpdateReserveTable(int robotid, GroupData data);
    virtual void UpdatePaths(int maxtime);

    virtual void Extend(Group* g);
    virtual MPCoordPlanner::Group* SelectGroup(void);


    UseMap(std::vector<int>, Group*)  m_groups;


    MPScene                       m_scene;
    std::vector<RobotPlanner*>    m_robots;
    MPAbstraction                *m_abstract;
    MPMultiSearch                *m_discreteSearch;
    int                           m_robotType;
    double                        m_dsel;
    int             m_depthSearch;
    std::vector<std::vector<double*>> m_reserveTable;

  };
}

#endif
