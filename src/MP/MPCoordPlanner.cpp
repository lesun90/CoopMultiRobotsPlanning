#include "MP/MPCoordPlanner.hpp"

namespace MP
{
  MPCoordPlanner::MPCoordPlanner(void)
  {
    m_robotType = 0;
    m_dsel = 0.5;
    m_depthSearch = 10;
  }

  void MPCoordPlanner::CompleteSetup()
  {
    m_scene.CompleteSetup();
    m_abstract = new MPPrmAbstraction();
    // m_abstract = new MPTriAbstraction();

    m_abstract->SetScene(&m_scene);
    m_abstract->CompleteSetup();
    m_robots.resize(m_scene.m_robotInit.size());
    for (int i = 0; i< m_robots.size(); i++)
    {
      m_robots[i] = new RobotPlanner(m_abstract,i,m_robotType);
    }

    m_discreteSearch = new MPCoopAStar();
    m_discreteSearch->SetAbstraction(m_abstract);
    m_discreteSearch->CompleteSetup();
    m_discreteSearch->m_depth = m_depthSearch;
    for (int i = 0; i< m_robots.size(); i++)
    {
      m_discreteSearch->m_goals[i] = m_abstract->LocateRegion(m_scene.m_goals[i]->m_cfg);
    }
    Initialize();
  }

  void MPCoordPlanner::Initialize(void)
  {
    for (int i = 0; i< m_robots.size(); i++)
    {
      if(!(m_robots[i]->m_sim->IsStateValid()))
      {
        printf("initial state is in collision %d\n",i);
        exit(0);
      }
    }

    std::vector<std::vector<MPState*>> statePaths;
    std::vector<std::vector<double*>> cfgPaths;

    statePaths.resize(m_robots.size());
    cfgPaths.resize(m_robots.size());
    for (int r = 0; r< m_robots.size(); r++)
    {
      MPState* state;
      state = m_robots[r]->m_sim->NewState();
      m_robots[r]->m_sim->GetState(state);
      statePaths[r].push_back(state);

      double* cfg;
      cfg = m_robots[r]->m_sim->NewCfg();
      m_robots[r]->m_sim->GetCfg(cfg);
      cfgPaths[r].push_back(cfg);
    }
    AddGroupData(statePaths,cfgPaths);
  }

  void MPCoordPlanner::AddGroupData(std::vector<std::vector<MPState*>> statePaths,std::vector<std::vector<double*>> cfgPaths)
  {
    std::vector<int> groupid;
    groupid.resize(m_robots.size());

    std::vector<MPState*> groupState;
    groupState.resize(m_robots.size());

    std::vector<double*> groupCfg;
    groupCfg.resize(m_robots.size());

    for (int r = 0; r< m_robots.size(); r++)
    {
      m_robots[r]->SetState(statePaths[r].back());
      groupid[r]    = m_robots[r]->GetCurrentRegion();
      groupState[r] = statePaths[r].back();
      groupCfg[r]   = cfgPaths[r].back();
    }
    Group *g;
    GroupData data;
    data.m_groupState = groupState;
    data.m_groupCfg   = groupCfg;
    data.m_statePaths = statePaths;
    data.m_cfgPaths   = cfgPaths;

    auto it = m_groups.find(groupid);
    if(it == m_groups.end())
    {
      g = new Group();
      g->m_id = groupid;
      g->m_groupData.push_back(data);
      m_discreteSearch->Clear();
      if (m_discreteSearch->RunSearch(g->m_id) == false)
      {
        return;
      }

      // printf("add Id: ");
      // for (int i = 0; i < g->m_id.size(); i++)
      // {
      //   printf("%d ",g->m_id[i] );
      // }
      // printf("\n" );
      // g->m_pathsToGoal.resize(m_robots.size());
      // for(int r = 0 ; r < m_robots.size(); r++)
      // {
      //   g->m_pathsToGoal[r] = m_abstract->m_regions[g->m_id[r]]->m_pathsToGoal[r];
      // }
      // g->m_weight = 0;
      // for(int r = 0 ; r < m_robots.size(); r++)
      // {
      //   for (int i = 0 ; i < g->m_pathsToGoal[r].size()-1; i++)
      //   {
      //       g->m_weight += Algebra2D::PointDist(m_abstract->m_regions[g->m_pathsToGoal[r][i]]->m_cfg
      //         ,m_abstract->m_regions[g->m_pathsToGoal[r][i+1]]->m_cfg);
      //   }
      // }
      // g->m_weight = 1000 / g->m_weight;


      g->m_pathsToGoal = m_discreteSearch->m_pathsToGoal;
      g->m_weight = 1000 / m_discreteSearch->GetPathsCost();
      g->m_robotOrder = m_discreteSearch->m_robotOrder;
      m_groups.insert(std::make_pair(g->m_id, g));
    }
    else
    {
      g = it->second;
      g->m_groupData.push_back(data);
    }
  }

  void MPCoordPlanner::Run(const int nrIters)
  {
    Group *g;
    for(int i = 0; i < 1 && IsSolved() == false; ++i)
    {
      g = SelectGroup();
      printf("select: %d %d\n",g->m_id[0],g->m_id[1] );
      Extend(g);
      g->m_weight *= m_dsel;
      if(g->m_weight < Constants::EPSILON)
      {
        for(auto & it : m_groups)
        it.second->m_weight /= m_dsel;

        m_discreteSearch->Clear();
        if (m_discreteSearch->RunSearch(g->m_id) == true)
        {
          g->m_pathsToGoal.clear();
          g->m_pathsToGoal = m_discreteSearch->m_pathsToGoal;
          g->m_weight = 1000 / m_discreteSearch->GetPathsCost();
          g->m_robotOrder = m_discreteSearch->m_robotOrder;
        }
      }
    }
  }

  void MPCoordPlanner::Extend(Group* g)
  {
    int maxTime = -1;
    int minTime = 9999;

    GroupData data;
    data = g->m_groupData[RandomUniformInteger(0, g->m_groupData.size() - 1)];
    std::vector<int> m_robotOrder;
    for (int r = 0; r< m_robots.size(); r++)
    {
      m_robotOrder.push_back(r);
      m_robots[r]->SetCurrPathToGoal(g->m_pathsToGoal[r]);
      m_robots[r]->ClearPaths();
    }
    // std::random_shuffle ( m_robotOrder.begin(), m_robotOrder.end() );
    m_robotOrder = g->m_robotOrder;
    for (int r = 0; r < m_robotOrder.size(); r++)
    {
      int robotid = m_robotOrder[r];
      UpdateReserveTable(robotid,data);
      m_robots[robotid]->m_reserveTable = m_reserveTable;
      m_robots[robotid]->m_maxTime = maxTime;
      m_robots[robotid]->FindFollowRegionPath(data.m_groupState[robotid]);
      if (maxTime < (int)m_robots[robotid]->m_statePath.size() )
      {
        maxTime = (int)m_robots[robotid]->m_statePath.size();
      }
    }

    // std::vector<std::vector<MPState*>> statePaths;
    // std::vector<std::vector<double*>> cfgPaths;
    //
    // statePaths.resize(m_robots.size());
    // cfgPaths.resize(m_robots.size());
    //
    // for (int i = 0; i < minTime ; i++)
    // {
    //   for (int r = 0; r < m_robots.size(); r++)
    //   {
    //     statePaths[r].clear();
    //     statePaths[r] = data.m_statePaths[r];
    //     cfgPaths[r].clear();
    //     cfgPaths[r] = data.m_cfgPaths[r];
    //     for (int k = 1; k<= i; k++)
    //     {
    //       statePaths[r].push_back(m_robots[r]->m_statePath.back());
    //       cfgPaths[r].push_back(m_robots[r]->m_cfgPath.back());
    //     }
    //   }
    //   AddGroupData(statePaths,cfgPaths);
    // }

    // for (int r = 0; r < m_robots.size(); r++)
    // {
    //   statePaths[r].insert(statePaths[r].begin()
    //   ,data.m_statePaths[r].begin()
    //   ,data.m_statePaths[r].end());
    //   m_robots[r]->m_statePath = statePaths[r];
    //
    //   cfgPaths[r].insert(cfgPaths[r].begin()
    //   ,data.m_cfgPaths[r].begin()
    //   ,data.m_cfgPaths[r].end());
    //   m_robots[r]->m_cfgPath = cfgPaths[r];
    // }

  }

  void MPCoordPlanner::UpdateReserveTable(int robotid, GroupData data)
  {
    m_reserveTable.clear();
    for (int r = 0; r< m_robots.size(); r++)
    {
      if (r == robotid)
      {
        continue;
      }
      if (m_robots[r]->m_cfgPath.size() > 0)
      {
        m_reserveTable.push_back(m_robots[r]->m_cfgPath);
      }
    }
  }

  void MPCoordPlanner::UpdatePaths(int maxtime)
  {
    for (int r = 0; r< m_robots.size(); r++)
    {
      if (m_robots[r]->m_statePath.size()>maxtime)
      {
        m_robots[r]->m_statePath.resize(maxtime);
        m_robots[r]->m_cfgPath.resize(maxtime);
      }
    }
  }

  MPCoordPlanner::Group* MPCoordPlanner::SelectGroup(void)
  {
    double wmax = -HUGE_VAL;
    Group *gmax = NULL;

    for(auto & it : m_groups)
    {
      if(it.second->m_weight > wmax)
      {
        wmax = it.second->m_weight;
        gmax = it.second;
      }
    }
    return gmax;
  }

  bool MPCoordPlanner::IsSolved()
  {
    for (int r = 0; r< m_robots.size(); r++)
    {
      if (m_robots[r]->m_cfgPath.size()<=0)
      {
        return false;
      }
      if (Algebra2D::PointDist(m_robots[r]->m_cfgPath.back(),m_robots[r]->GetGoalCfg()) > 2.5)
      {
        return false;
      }
    }
    return true;
  }


  void MPCoordPlanner::Draw()
  {
    m_scene.Draw();
    m_abstract->Draw();
    for (int i = 0; i< m_robots.size(); i++)
    {
      m_robots[i]->Draw();
    }
  }
}
