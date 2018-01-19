#include "MP/MPCoopPlanner.hpp"
#include "Utils/Timer.hpp"

namespace MP
{
  MPCoopPlanner::MPCoopPlanner(void)
  {
    m_vidSolved               = -1;
    m_probSteer               = 1;
    m_probSelectNearestVertex = Constants::PLANNER_PROBABILITY_SELECT_NEAREST_VERTEX;
    m_dsel                    = Constants::PLANNER_SELECTION_PENALTY;
    m_robotType = 0;
    m_depthSearch = 5;
    m_searchType = 0;
  }

  void MPCoopPlanner::CompleteSetup()
  {
    Stats::GetSingleton()->AddValue("TimePreprocess", 0.0);
    Stats::GetSingleton()->AddValue("SimulateTime", 0.0);
    Stats::GetSingleton()->AddValue("ValidateCheckTime", 0.0);
    Stats::GetSingleton()->AddValue("FindPathTime", 0.0);
    Stats::GetSingleton()->AddValue("NrFindPath", 0.0);
    Stats::GetSingleton()->AddValue("CollisionWithRobots", 0.0);
    Stats::GetSingleton()->AddValue("CollisionWithObstacle", 0.0);
    Stats::GetSingleton()->AddValue("RobotType", m_robotType);
    Stats::GetSingleton()->AddValue("SearchType", m_searchType);
    Stats::GetSingleton()->AddValue("PathDepth", m_depthSearch);

    m_scene.CompleteSetup();

    // m_abstract = new MPTriAbstraction();
    m_abstract = new MPPrmAbstraction();
    m_abstract->SetScene(&m_scene);
    Timer::Clock clk;
    Timer::Start(&clk);
    m_abstract->CompleteSetup();
    Stats::GetSingleton()->AddValue("TimePreprocess", Timer::Elapsed(&clk));

    m_robots.resize(m_scene.m_robotInit.size());

    if (m_searchType == COOPASTART)
    {
      m_discreteSearch = new MPCoopAStar();
      m_discreteSearch->m_depth = m_depthSearch;

    }
    else if (m_searchType == PAS)
    {
      m_discreteSearch = new MPPushAndSwap();
    }
    else if (m_searchType == SIPP)
    {
      m_discreteSearch = new MPSIPP();
    }

    m_discreteSearch->SetAbstraction(m_abstract);
    m_discreteSearch->CompleteSetup();

    for (int i = 0; i< m_robots.size(); i++)
    {
      m_robots[i] = new RobotPlanner(m_abstract,i,m_robotType);
      m_discreteSearch->m_goals[i] = m_abstract->LocateRegion(m_scene.m_goals[i]->m_cfg);
    }
    Initialize();
  }

  void MPCoopPlanner::Run(const int nrIters)
  {
    Group *g;
    for(int i = 0; i < 100 && m_vidSolved < 0; ++i)
    {
      g = SelectGroup();
      int vid = SelectTargetAndVertex(g);

      ExtendFrom(g, vid);

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
        }
      }
    }
  }

  bool MPCoopPlanner::ExtendFrom(Group *g, const int vid)
  {
    int                 parent = vid;
    std::vector<bool>   steer;
    int                 i = 0;
    const int  nrSteps = RandomUniformInteger(m_minNrSteps, m_maxNrSteps);
    std::vector<double*> m_newGroupCfg;
    m_newGroupCfg.resize(m_robots.size());
    steer.resize(m_robots.size());

    for (int r = 0; r < m_robots.size(); r++)
    {
      m_robots[r]->m_sim->SetState(m_vertices[vid]->m_groupState[r]);
      m_robots[r]->StartSteerToTarget();
    }
    std::vector<int> robotorder;
    PermuteRobots(robotorder);

    for( i = 0; i < nrSteps && GetSolved() < 0; ++i)
    {
      for (int r = 0; r < m_robots.size(); r++)
      {
        int robotid = robotorder[r];
        m_robots[robotid]->SteerToTarget();
        if (m_robots[robotid]->HasReachTarget() == true)
        {
          m_robots[robotid]->GenerateTarget();
          m_robots[robotid]->StartSteerToTarget();
        }
      }
      Timer::Clock vclk;
      Timer::Start(&vclk);
      bool valid = AreValidStates();
      Stats::GetSingleton()->AddValue("ValidateCheckTime", Timer::Elapsed(&vclk));

      if(valid == false)
      {
        return false;
      }

      GroupVertex *vnew   = new GroupVertex();
      vnew->m_parent = parent;
      bool add = AddGroupVertex(vnew);

      if (add == false)
      {
        delete vnew;
        return false;
      }

      parent = m_vertices.size() - 1;
    }

    return true;
  }

  bool MPCoopPlanner::AreValidStates()
  {
    //check collision with obs
    for (int r = 0; r< m_robots.size(); r++)
    {
      if((!(m_robots[r]->m_sim->IsStateValid())) || (m_robots[r]->GetCurrentRegion() == -1))
      {
        Stats::GetSingleton()->AddValue("CollisionWithObstacle", 1.0);
        return false;
      }
    }

    // for (int r = 0; r< m_robots.size(); r++)
    // {
    //   if((!(m_robots[r]->m_sim->IsCfgValid())) || (m_robots[r]->GetCurrentRegion() == -1))
    //   {
    //     Stats::GetSingleton()->AddValue("CollisionWithObstacle", 1.0);
    //     Stats::GetSingleton()->AddValue("ValidateCheckTime", Timer::Elapsed(&clk));
    //     return false;
    //   }
    // }
    //check collison with robots
    for (int a = 0; a< m_robots.size()-1; a++)
    {
      for (int b = a+1; b< m_robots.size(); b++)
      {
        double d = Algebra2D::PointDistSquared(m_robots[a]->m_sim->GetCfg(),m_robots[b]->m_sim->GetCfg());
        if (d < Constants::ROBOT_LENGTH*Constants::ROBOT_LENGTH)
        {
          Stats::GetSingleton()->AddValue("CollisionWithRobots", 1.0);
          return false;
        }
      }
    }
    return true;
  }

  MPCoopPlanner::Group* MPCoopPlanner::SelectGroup(void)
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

  int MPCoopPlanner::SelectTargetAndVertex(Group *g)
  {
    //generate target
    for (int r = 0; r< m_robots.size(); r++)
    {
      m_robots[r]->SetCurrPathToGoal(g->m_pathsToGoal[r]);
      m_robots[r]->GenerateTarget();

    }

    if(RandomUniformReal() < 1 - m_probSelectNearestVertex)
    return g->m_vids[RandomUniformInteger(0, g->m_vids.size() - 1)];

    double dmin = HUGE_VAL;
    int    imin = -1;
    double d;

    for(auto &i : g->m_vids)
    {
      double sumDist = 0;
      for (int r = 0; r< m_robots.size(); ++r)
      {
        sumDist += Algebra2D::PointDistSquared(m_robots[r]->m_target,m_vertices[i]->m_groupCfg[r]);
      }
      if(sumDist < dmin)
      {
        dmin = sumDist;
        imin = i;
      }
    }
    return imin;
  }

  void MPCoopPlanner::Initialize(void)
  {
    for (int i = 0; i< m_robots.size(); i++)
    {
      if(!(m_robots[i]->m_sim->IsStateValid()))
      {
        printf("initial state is in collision %d\n",i);
        exit(0);
      }
    }

    GroupVertex *v   = new GroupVertex();
    if(AddGroupVertex(v) < 0)
    {
      printf("initial state invalid\n");
      delete v;
      exit(0);
    }
  }

  int MPCoopPlanner::AddGroupVertex(GroupVertex * const v)
  {
    v->m_groupCfg.resize(m_robots.size());
    v->m_groupState.resize(m_robots.size());

    for (int i = 0; i< m_robots.size(); i++)
    {
      v->m_groupCfg[i] = m_robots[i]->m_sim->NewCfg();
      m_robots[i]->m_sim->GetCfg(v->m_groupCfg[i]);

      v->m_groupState[i] = m_robots[i]->m_sim->NewState();
      m_robots[i]->m_sim->GetState(v->m_groupState[i]);
    }

    std::vector<int> groupid;
    groupid.resize(m_robots.size());
    double w = 0.0;
    for (int r = 0; r< m_robots.size(); r++)
    {
      groupid[r] = m_robots[r]->GetCurrentRegion();
      w += m_abstract->m_regions[groupid[r]]->m_distToGoals[r]*m_abstract->m_regions[groupid[r]]->m_distToGoals[r];
    }

    v->m_groupid = groupid;
    v->m_weight  = w;

    m_vertices.push_back(v);
    int vid = m_vertices.size() - 1;
    AddGroupData(vid);

    if(AllRobotsAtGoals()==true)
    {
      m_vidSolved = vid;
    }
    if(m_vidSolved > 0)
    {
      GetPaths();
    }

    return vid;
  }

  void MPCoopPlanner::AddGroupData(const int vid)
  {
    for (int r = 0; r < m_robots.size(); r++)
    {
      int regionid = m_robots[r]->GetCurrentRegion();
      if (m_abstract->m_regions[regionid]->m_pathsToGoal[r].size()==0)
      return;
    }

    Group *g;

    auto it = m_groups.find(m_vertices[vid]->m_groupid);

    if(it == m_groups.end())
    {
      g = new Group();
      g->m_id             = m_vertices[vid]->m_groupid;
      m_discreteSearch->Clear();
      if (m_discreteSearch->RunSearch(g->m_id) == false)
      {
        return;
      }

      g->m_pathsToGoal = m_discreteSearch->m_pathsToGoal;
      g->m_weight = 1000 / m_discreteSearch->GetPathsCost();
      g->m_vids.push_back(vid);
      m_groups.insert(std::make_pair(g->m_id, g));
    }
    else
    {
      g = it->second;
      g->m_vids.push_back(vid);
    }
  }

  void MPCoopPlanner::GetPaths(void)
  {
    m_path.clear();
    int pid = m_vidSolved;
    while(pid >= 0)
    {
      m_path.push_back(pid);

      if(pid >= (int) m_vertices.size())
      printf("path is wrong ... pid = %d nv = %d size=%d\n ", pid, (int) m_vertices.size(), (int) m_path.size());

      pid = m_vertices[pid]->m_parent;
    }
    std::reverse(m_path.begin(),m_path.end());    // 9 8 7 6 5 4 3 2 1

    for (int r = 0; r < m_robots.size(); r++)
    {
      m_robots[r]->m_statePath.clear();
      m_robots[r]->m_cfgPath.clear();
      m_robots[r]->m_path.clear();
      for (int i = 0; i<m_path.size();++i)
      {
        m_robots[r]->m_path.push_back(m_path[i]);
        m_robots[r]->m_statePath.push_back(m_vertices[m_path[i]]->m_groupState[r]);
        m_robots[r]->m_cfgPath.push_back(m_vertices[m_path[i]]->m_groupCfg[r]);
      }
    }
  }

  void MPCoopPlanner::Draw()
  {
    m_scene.Draw();
    // m_abstract->Draw();
    for (int i = 0; i< m_robots.size(); i++)
    {
      m_robots[i]->Draw();
      // m_robots[i]->DrawPathToGoal();
    }

    if (IsSolved() == false)
    {
      // DrawGroupVertex();
    }
    else
    {
      // DrawGroupVertex();
      // DrawCfgPaths();
    }
  }

  void MPCoopPlanner::DrawGroupVertex()
  {
    const bool is2D = GDrawIs2D();
    GDrawLineWidth(4.0);
    GDrawMultTrans(0, 0, m_abstract->m_scene->m_maxGroundHeight+0.1);

    for(int i = m_vertices.size() - 1; i >= 1; --i)
    {
      for (int r = 0; r< m_robots.size(); r++)
      {
        GDrawIndexColor(r);
        GDrawSegment2D(m_vertices[i]->m_groupCfg[r], m_vertices[m_vertices[i]->m_parent]->m_groupCfg[r]);
      }
    }
    GDrawPopTransformation();
    if(!is2D)
    GDraw3D();
  }

  void MPCoopPlanner::DrawCfgPaths()
  {
    for (int r = 0; r< m_robots.size(); r++)
    {
      m_robots[r]->DrawCfgPath();
    }
  }

}
