#include "MP/MPPriorityPlannerRRT.hpp"

namespace MP
{
  void MPPriorityPlannerRRT::CompleteSetup()
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

    GetRobotID();
    m_reserveTable.resize(m_robots.size());
    for(int r=0; r <m_reserveTable.size(); r++)
    {
      m_reserveTable[r].clear();
    }
  }

  void MPPriorityPlannerRRT::GetRobotID()
  {
    //PermuteItem
    m_robotids.clear();
    for (int r = 0; r < m_robots.size(); r++)
    {
      if (m_robots[r]->m_cfgPath.size()>0)
      {
        continue;
      }
      m_robotids.push_back(r);
    }
    PermuteItems<int>(&m_robotids,m_robotids.size());
    m_currentrobotid = m_robotids[0];
  }

  void MPPriorityPlannerRRT::ReadyForNewRobot(void)
  {
    m_vertices.clear();
  }

  void MPPriorityPlannerRRT::ReserveTable(int robotid)
  {
    m_reserveTable[robotid] = m_robots[robotid]->m_cfgPath;
  }

  void MPPriorityPlannerRRT::Initialize(void)
  {
    if(m_cfgTarget == NULL)
    m_cfgTarget = m_robots[m_currentrobotid]->m_sim->NewCfg();

    if(!(m_robots[m_currentrobotid]->m_sim->IsStateValid()))
    {
      printf("initial state is in collision\n");
      exit(0);
    }

    Vertex *v   = NewVertex();
    if(AddVertex(v) < 0)
    {
      printf("initial state invalid\n");
      delete v;
      exit(0);
    }
    v->m_timeid = 0;
  }

  int MPPriorityPlannerRRT::AddVertex(Vertex * const v)
  {
    if(v->m_cfg == NULL)
    {
      v->m_cfg = m_robots[m_currentrobotid]->m_sim->NewCfg();
    }
    if(v->m_s == NULL)
    {
      v->m_s = m_robots[m_currentrobotid]->m_sim->NewState();
    }
    m_robots[m_currentrobotid]->m_sim->GetCfg(v->m_cfg);
    m_robots[m_currentrobotid]->m_sim->GetState(v->m_s);

    // int rid = m_abstract->LocateRegion(v->m_cfg);
    //
    // if(rid < 0)
    // {
    //   return Constants::ID_UNDEFINED;;
    // }
    // v->m_rid = rid;

    m_vertices.push_back(v);
    const int vid = m_vertices.size() - 1;

    if(m_robots[m_currentrobotid]->HasReachGoal())
    {
      m_vidSolved = vid;
      bool res = GetCfgPath(m_currentrobotid);
      if (res == false)
      {
        m_vidSolved = -1;
      }
    }
    //AddVertexToARData(vid);
    return vid;
  }

  void MPPriorityPlannerRRT::Run(const int nrIters)
  {
    if (IsSolved() == true)
    {
      return;
    }
    if (m_robots[m_currentrobotid]->m_cfgPath.size()>0)
    {
      ReserveTable(m_currentrobotid);
      GetRobotID();
      ReadyForNewRobot();
    }

    if(m_vertices.size() == 0)
    {
      Initialize();
    }

    m_robots[m_currentrobotid]->SampleRandomTarget();

    for(int i = 0; i < nrIters && m_robots[m_currentrobotid]->m_cfgPath.size()<=0; ++i)
    {
      // m_robots[m_currentrobotid]->m_sim->SampleCfg(m_cfgTarget);
      m_robots[m_currentrobotid]->SampleRandomTarget();
      if(RandomUniformReal() < 0.1)
      {
        m_robots[m_currentrobotid]->SampleNextTargetAtGoal();
      }
      const int vid  = SelectVertex();
      ExtendFrom(vid);
    }
  }

  bool MPPriorityPlannerRRT::IsSolved(void)
  {
    for (int r = 0 ; r < m_robots.size(); r++)
    {
      if (m_robots[r]->m_cfgPath.size()<=0)
      {
        return false;
      }
    }
    return true;
  }

  int MPPriorityPlannerRRT::SelectVertex()
  {
    double dmin = HUGE_VAL, d;
    int imin = -1;
    for(int i = 0; i < m_vertices.size(); ++i)
    {
      d = m_robots[m_currentrobotid]->m_sim->DistanceCfg(m_robots[m_currentrobotid]->m_target, m_vertices[i]->m_cfg);
      if(d < dmin)
      {
        imin = i;
        dmin = d;
      }
    }
    return imin;
  }

  int MPPriorityPlannerRRT::ExtendFrom(const int vid)
  {
    int        parent  = vid;
    int        i       = 0;
    const bool steer   = RandomUniformReal() < m_probSteer;
    const int  nrSteps = RandomUniformInteger(m_minNrSteps, m_maxNrSteps);
    int        count   = 0;

    m_robots[m_currentrobotid]->m_sim->SetState(m_vertices[vid]->m_s);
    m_robots[m_currentrobotid]->StartSteerToTarget();
    
    for(i = 0; i < nrSteps && m_robots[m_currentrobotid]->m_cfgPath.size()<=0; ++i)
    {
      Timer::Clock sclk;
      Timer::Start(&sclk);
      m_robots[m_currentrobotid]->SteerToTarget();

      Timer::Clock clk;
      Timer::Start(&clk);
      if(!m_robots[m_currentrobotid]->m_sim->IsStateValid())
      {
        Stats::GetSingleton()->AddValue("CollisionWithObstacle", 1.0);
        Stats::GetSingleton()->AddValue("ValidateCheckTime", Timer::Elapsed(&clk));
        return i;
      }
      int timeid = m_vertices[parent]->m_timeid + 1;

      if(CheckWithReserveTable(timeid)==false)
      {
        Stats::GetSingleton()->AddValue("CollisionWithRobots", 1.0);
        Stats::GetSingleton()->AddValue("ValidateCheckTime", Timer::Elapsed(&clk));
        return i;
      }
      Stats::GetSingleton()->AddValue("ValidateCheckTime", Timer::Elapsed(&clk));

      Vertex *vnew = NewVertex();
      vnew->m_parent = parent;
      vnew->m_timeid = timeid;

      if(AddVertex(vnew) < 0)
      {
        delete vnew;
        return i;
      }

      if(steer && m_robots[m_currentrobotid]->HasReachTarget())
      return i;

      parent = m_vertices.size() - 1;
    }

    return i;
  }
  bool MPPriorityPlannerRRT::CheckWithReserveTable(int timeid)
  {
    int checkid;

    for (int r = 0 ; r < m_robots.size(); r++)
    {
      if ((r == m_currentrobotid) || ((int)m_reserveTable[r].size()==0))
      {
        //printf("m_currentrobotid %d\n",m_currentrobotid );
        continue;
      }
      else
      {
        if (timeid > ((int)m_reserveTable[r].size()-1))
        {
          checkid = (int)m_reserveTable[r].size()-1;
        }
        else
        {
          checkid = timeid;
        }

        double d = Algebra2D::PointDistSquared(m_robots[m_currentrobotid]->m_sim->GetCfg(),m_reserveTable[r][checkid]);
        if (d < 5*5)
        {
          return false;
        }
      }

    }
    return true;
  }

  void MPPriorityPlannerRRT::GetPath(int robotid)
  {
    m_robots[robotid]->m_path.clear();
    int pid = m_vidSolved;
    while(pid >= 0)
    {
      m_robots[robotid]->m_path.push_back(pid);

      if(pid >= (int) m_vertices.size())
      printf("path is wrong ... pid = %d nv = %d size=%d\n ", pid, (int) m_vertices.size(), (int) m_robots[robotid]->m_path.size());

      pid = m_vertices[pid]->m_parent;
    }
    std::reverse(m_robots[robotid]->m_path.begin(),m_robots[robotid]->m_path.end());    // 9 8 7 6 5 4 3 2 1
    for (int i = 0; i<m_robots[robotid]->m_path.size();++i)
    {
      m_robots[robotid]->m_statePath.push_back(m_vertices[m_robots[robotid]->m_path[i]]->m_s);
    }
  }

  bool MPPriorityPlannerRRT::GetCfgPath(int robotid)
  {
    GetPath(robotid);
    for (int i = 0; i<m_robots[robotid]->m_path.size();++i)
    {
      m_robots[robotid]->m_cfgPath.push_back(m_vertices[m_robots[robotid]->m_path[i]]->m_cfg);
    }

    for (int r = 0; r < m_robots.size(); r++)
    {
      if (r == robotid)
      {
        continue;
      }
      if (m_robots[robotid]->m_cfgPath.size() <= m_reserveTable[r].size())
      {
        for (int i = m_robots[robotid]->m_cfgPath.size() ; i < m_reserveTable[r].size(); i++)
        {
          if (Algebra2D::PointDist(m_robots[robotid]->m_cfgPath.back(),m_reserveTable[r][i])<5)
          {
            m_robots[robotid]->m_cfgPath.clear();
            m_robots[robotid]->m_statePath.clear();
            m_robots[robotid]->m_path.clear();
            return false;
          }
        }
      }
    }
    return true;
  }

  void MPPriorityPlannerRRT::Draw()
  {
    m_scene.Draw();
    for (int i = 0; i< m_robots.size(); i++)
    {
      m_robots[i]->Draw();
      m_robots[i]->DrawCfgPath();
      m_robots[i]->Draw();

    }
    if (IsSolved() == false)
    DrawVertices();
  }

}
