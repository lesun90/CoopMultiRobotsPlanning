#include "MP/MPCentralizedPlanner.hpp"

namespace MP
{
  void MPCentralizedPlanner::Draw()
  {
    m_scene.Draw();
    for (int i = 0; i< m_robots.size(); i++)
    {
      m_robots[i]->Draw();
    }

    if (m_vidSolved<0)
    {
      DrawGroupVertex();
    }
    else
    {
      DrawCfgPaths();
    }

  }

  void MPCentralizedPlanner::CompleteSetup()
  {
    Stats::GetSingleton()->AddValue("TimePreprocess", 0.0);
    Stats::GetSingleton()->AddValue("SimulateTime", 0.0);
    Stats::GetSingleton()->AddValue("ValidateCheckTime", 0.0);
    Stats::GetSingleton()->AddValue("FindPathTime", 0.0);
    Stats::GetSingleton()->AddValue("NrFindPath", 0.0);
    Stats::GetSingleton()->AddValue("PathDepth", 0.0);
    Stats::GetSingleton()->AddValue("CollisionWithRobots", 0.0);
    Stats::GetSingleton()->AddValue("CollisionWithObstacle", 0.0);
    Stats::GetSingleton()->AddValue("RobotType", m_robotType);

    m_scene.CompleteSetup();
    m_abstract = new MPTriAbstraction();
    m_abstract->SetScene(&m_scene);
    m_abstract->CompleteSetup();
    m_robots.resize(m_scene.m_robotInit.size());
    for (int i = 0; i< m_robots.size(); i++)
    {
      m_robots[i] = new RobotPlanner(m_abstract,i,m_robotType);
    }
  }

  void MPCentralizedPlanner::Run(const int nrIters)
  {
    if(m_vertices.size() == 0)
    Initialize();

    for(int i = 0; i < nrIters && GetSolved()<0; ++i)
    {
      GenerateTargets();
      const int vid  = SelectGroupVertex();
      ExtendFrom(vid);
    }
  }

  void MPCentralizedPlanner::GenerateTargets()
  {
    for (int i = 0; i < m_robots.size(); i++)
    {
      m_robots[i]->GenerateRandomTarget();
      if(RandomUniformReal() < 0.1)
      {
        m_robots[i]->SampleNextTargetAtGoal();
      }
    }
  }

  int MPCentralizedPlanner::SelectGroupVertex()
  {
    double   dmin    = HUGE_VAL;
    int      imin    = -1;
    for(int i = m_vertices.size()-1 ; i >=0; --i)
    {
      double sumDist =0;
      for (int r = 0; r< m_robots.size(); ++r)
      {
        sumDist += Algebra2D::PointDist(m_robots[r]->m_target,m_vertices[i]->m_groupCfg[r]);
      }
      if(sumDist < dmin)
      {
        dmin = sumDist;
        imin = i;
      }
    }
    return imin;
  }

  void MPCentralizedPlanner::Initialize(void)
  {
    for (int i = 0; i< m_robots.size(); i++)
    {
      if(!(m_robots[i]->m_sim->IsStateValid()))
      {
        printf("initial state is in collision\n");
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

  int MPCentralizedPlanner::AddGroupVertex(GroupVertex * const v)
  {
    v->m_groupState.resize(m_robots.size());
    v->m_groupCfg.resize(m_robots.size());

    for (int i = 0; i< m_robots.size(); i++)
    {
      v->m_groupCfg[i] = m_robots[i]->m_sim->NewCfg();
      m_robots[i]->m_sim->GetCfg(v->m_groupCfg[i]);

      v->m_groupState[i] = m_robots[i]->m_sim->NewState();
      m_robots[i]->m_sim->GetState(v->m_groupState[i]);
    }

    m_vertices.push_back(v);
    int vid = m_vertices.size() - 1;

    if(ReachGoals()==true)
    {
      m_vidSolved = vid;
      GetPaths();
    }

    return vid;
  }

  void MPCentralizedPlanner::GetPaths(void)
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

  int MPCentralizedPlanner::ExtendFrom(const int vid)
  {
    int        parent  = vid;
    std::vector<bool>  steer;
    int i = 0;
    const int  nrSteps = RandomUniformInteger(m_minNrSteps, m_maxNrSteps);
    int        count   = 0;
    std::vector<double*> m_newGroupCfg;

    m_newGroupCfg.resize(m_robots.size());

    for (int r = 0; r < m_robots.size(); r++)
    {
      m_robots[r]->m_sim->SetState(m_vertices[vid]->m_groupState[r]);
      m_robots[r]->StartSteerToTarget();
    }

    for(i = 0; i < nrSteps && GetSolved() < 0; ++i)
    {
      for (int r = 0; r < m_robots.size(); r++)
      {
        m_robots[r]->SteerToTarget();
      }

      Timer::Clock clk;
      Timer::Start(&clk);
      bool valid = AreValidStates();
      Stats::GetSingleton()->AddValue("ValidateCheckTime", Timer::Elapsed(&clk));

      if(valid == false)
      {
        return i;
      }

      GroupVertex *vnew   = new GroupVertex();
      vnew->m_parent = parent;

      if(AddGroupVertex(vnew) < 0)
      {
        delete vnew;
        return i;
      }

      if (GroupReachTarget())
      {
        return i;
      }

      parent = m_vertices.size() - 1;
    }
    return i;

  }

  bool MPCentralizedPlanner::GroupReachTarget()
  {
    for (int i = 0; i< m_robots.size(); i++)
    {
      if(m_robots[i]->HasReachTarget() == false)
      {
        return false;
      }
    }
    return true;
  }

  bool MPCentralizedPlanner::AreValidStates()
  {

    for (int i = 0; i< m_robots.size(); i++)
    {
      if(!(m_robots[i]->m_sim->IsStateValid()))
      {
        Stats::GetSingleton()->AddValue("CollisionWithObstacle", 1.0);
        return false;
      }
    }
    //check robots collision here
    //
    for (int a = 0; a < m_robots.size()-1; a++)
    {
      for (int b = a+1; b< m_robots.size(); b++)
      {
        double d = Algebra2D::PointDist(m_robots[a]->m_sim->GetCfg(),m_robots[b]->m_sim->GetCfg());
        if (d < 5)
        {
          Stats::GetSingleton()->AddValue("CollisionWithRobots", 1.0);
          return false;
        }
      }
    }

    return true;
  }


  bool MPCentralizedPlanner::ReachGoals()
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

  void MPCentralizedPlanner::DrawGroupVertex()
  {
    const bool is2D = GDrawIs2D();
    GDrawLineWidth(4.0);
    GDrawColor(0, 0, 1);
    GDrawMultTrans(0, 0, 0.4);

    for(int i = m_vertices.size() - 1; i >= 1; --i)
    {
      for (int r = 0; r< m_robots.size(); r++)
      {
        GDrawSegment2D(m_vertices[i]->m_groupCfg[r], m_vertices[m_vertices[i]->m_parent]->m_groupCfg[r]);
      }
    }

    GDrawPopTransformation();
    if(!is2D)
    GDraw3D();
  }

  void MPCentralizedPlanner::DrawCfgPaths()
  {
    for (int r = 0; r< m_robots.size(); r++)
    {
      m_robots[r]->DrawCfgPath();
    }
  }


}
