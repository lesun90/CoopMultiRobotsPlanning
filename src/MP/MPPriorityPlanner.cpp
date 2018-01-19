#include "MP/MPPriorityPlanner.hpp"

namespace MP
{
  void MPPriorityPlanner::CompleteSetup()
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

    // m_abstract = new MPTriAbstraction();
    m_abstract = new MPPrmAbstraction();
    m_abstract->SetScene(&m_scene);
    Timer::Clock clk;
    Timer::Start(&clk);
    m_abstract->CompleteSetup();
    Stats::GetSingleton()->AddValue("TimePreprocess", Timer::Elapsed(&clk));
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
    Initialize();

  }

  void MPPriorityPlanner::GetRobotID()
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

  void MPPriorityPlanner::ReadyForNewRobot(void)
  {
    m_vertices.clear();
    for(auto it = m_map.begin(); it != m_map.end(); ++it)
    if(it->second)
    delete it->second;
    m_map.clear();
  }

  void MPPriorityPlanner::ReserveTable(int robotid)
  {
    m_reserveTable[robotid] = m_robots[robotid]->m_cfgPath;
  }

  void MPPriorityPlanner::Initialize(void)
  {
    if(m_robots[m_currentrobotid]->m_target == NULL)
    m_robots[m_currentrobotid]->m_target = m_robots[m_currentrobotid]->m_sim->NewCfg();

    if(!(m_robots[m_currentrobotid]->m_sim->IsStateValid()))
    {
      printf("initial state is in collision\n");
      exit(0);
    }

    Vertex *v   = NewVertex();
    v->m_timeid = 0;
    if(AddVertex(v) < 0)
    {
      printf("initial state invalid\n");
      delete v;
      exit(0);
    }
  }

  int MPPriorityPlanner::AddVertex(Vertex * const v)
  {
    if(v->m_cfg == NULL)
    {
      v->m_cfg = m_robots[m_currentrobotid]->m_sim->NewCfg();
    }
    m_robots[m_currentrobotid]->m_sim->GetCfg(v->m_cfg);

    v->m_s = m_robots[m_currentrobotid]->m_sim->NewState();
    m_robots[m_currentrobotid]->m_sim->GetState(v->m_s);

    int rid = m_abstract->LocateRegion(v->m_cfg);

    if((rid < 0)||(m_abstract->m_regions[rid]->m_valid == false))
    {
      return Constants::ID_UNDEFINED;;
    }
    v->m_rid = rid;

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
    AddVertexToARData(vid);
    return vid;
  }

  void MPPriorityPlanner::Run(const int nrIters)
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
    ARData *data;
    int     rid;
    for(int i = 0; i < nrIters && m_robots[m_currentrobotid]->m_cfgPath.size()<=0; ++i)
    {
      ARData   *data = SelectARData(&rid);
      SelectTarget(rid);
      const int vid  = SelectVertexFromARData(data);
      ExtendFrom(vid, m_robots[m_currentrobotid]->m_target);
      data->m_weight *= m_discountSelect;
      if(data->m_weight <= m_wmin)
      {
        for(auto it = m_map.begin(); it != m_map.end(); ++it)
        it->second->m_weight /= m_discountSelect;
      }
    }
  }

  void MPPriorityPlanner::CompleteARData(const int key, ARData * const data)
  {
    m_map.insert(std::make_pair(key, data));

    const double dmax = m_abstract->m_maxDistToGoal[m_currentrobotid];
    double       d    = m_abstract->m_regions[key]->m_distToGoals[m_currentrobotid];
    double       w    = d > dmax ? 0 : (dmax - d) / (dmax - m_abstract->m_minDistToGoal[m_currentrobotid]);
    const double alpha = 100000;
    const double eps  = 0.00000000001;

    w = eps + (1 - eps) * w;
    w = alpha * (pow(w, m_hexp));// * pow(m_abstract->m_regions[key]->m_vol, 3);
    if(w < m_wmin)
    w = m_wmin;
    data->m_weight = w;
  }


  bool MPPriorityPlanner::IsSolved(void)
  {
    for (int r = 0 ; r < m_robots.size(); r++)
    {
      if (m_robots[r]->m_path.size()<=0)
      {
        return false;
      }
    }
    return true;
  }

  void MPPriorityPlanner::SelectTarget(const int rid)
  {
    m_robots[m_currentrobotid]->m_sim->SampleCfg(m_robots[m_currentrobotid]->m_target);

    double      coin = RandomUniformReal();
    std::vector<int> *path = &(m_abstract->m_regions[rid]->m_pathsToGoal[m_currentrobotid]);
    m_robots[m_currentrobotid]->m_pathToGoal = m_abstract->m_regions[rid]->m_pathsToGoal[m_currentrobotid];

    if(coin < m_goalBias)
    {
      m_robots[m_currentrobotid]->m_target[0] = m_robots[m_currentrobotid]->m_sim->m_goal[0];
      m_robots[m_currentrobotid]->m_target[1] = m_robots[m_currentrobotid]->m_sim->m_goal[1];
    }
    else if(coin < 0.9 && path->size() > 1)
    {
      const int n     = path->size();
      const int start = 1;//std::min<int>(5, n - 1);
      const int end   = std::min<int>(10, n - 1);
      const int sampleinregion = (*path)[RandomUniformInteger(start, end)];
      // if ((sampleinregion == m_robots[m_currentrobotid]->GetGoalRegion()) && (RandomUniformReal(0,1)<0.5))
      // {
      //   m_robots[m_currentrobotid]->m_target[0] = m_robots[m_currentrobotid]->m_sim->m_goal[0];
      //   m_robots[m_currentrobotid]->m_target[1] = m_robots[m_currentrobotid]->m_sim->m_goal[1];
      // }
      // else
      // {
      //   m_robots[m_currentrobotid]->SampleNextTargetInRegion(sampleinregion);
      // }
      m_abstract->SamplePointInsideRegion(sampleinregion, m_robots[m_currentrobotid]->m_target, m_dtol);

    }
  }

  int MPPriorityPlanner::SelectVertexFromARData(const ARData * const data)
  {
    const int    n    = data->m_vids.size();

    if(RandomUniformReal() < 1 - Constants::PLANNER_PROBABILITY_SELECT_NEAREST_VERTEX)
    {
      return data->m_vids[RandomUniformInteger(0, data->m_vids.size() - 1)];
    }

    double dmin = HUGE_VAL, d;
    int imin = -1;
    for(int i = 0; i < n; ++i)
    {
      d = m_robots[m_currentrobotid]->m_sim->DistanceCfg(m_robots[m_currentrobotid]->m_target, m_vertices[data->m_vids[i]]->m_cfg);
      if(d < dmin)
      {
        imin = i;
        dmin = d;
      }

    }
    return data->m_vids[imin];
  }

  int MPPriorityPlanner::ExtendFrom(const int vid,const double target[])
  {
    int        parent  = vid;
    int        i       = 0;
    const bool steer   = RandomUniformReal() < m_probSteer;
    const int  nrSteps = RandomUniformInteger(m_minNrSteps, m_maxNrSteps);
    int        count   = 0;

    m_robots[m_currentrobotid]->m_sim->SetState(m_vertices[vid]->m_s);

    if(steer)
    m_robots[m_currentrobotid]->m_sim->StartSteerToPosition();
    else
    m_robots[m_currentrobotid]->m_sim->SampleControl();

    for(i = 0; i < nrSteps && m_robots[m_currentrobotid]->m_cfgPath.size()<=0; ++i)
    {

      if(steer)
      m_robots[m_currentrobotid]->m_sim->SteerToPosition(target);
      Timer::Clock sclk;
      Timer::Start(&sclk);
      m_robots[m_currentrobotid]->m_sim->SimulateOneStep();
      Stats::GetSingleton()->AddValue("SimulateTime", Timer::Elapsed(&sclk));

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

      if(steer && m_robots[m_currentrobotid]->m_sim->HasReachedPosition(target, 1))
      return i;

      parent = m_vertices.size() - 1;
    }

    return i;
  }
  bool MPPriorityPlanner::CheckWithReserveTable(int timeid)
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
  void MPPriorityPlanner::GetPath(int robotid)
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
  bool MPPriorityPlanner::GetCfgPath(int robotid)
  {
    // std::vector<int> path;
    // path.clear();
    // int pid = m_vidSolved;
    // while(pid >= 0)
    // {
    //   path.push_back(pid);
    //
    //   if(pid >= (int) m_vertices.size())
    //   printf("path is wrong ... pid = %d nv = %d size=%d\n ", pid, (int) m_vertices.size(), (int) path.size());
    //
    //   pid = m_vertices[pid]->m_parent;
    // }
    // std::reverse(path.begin(),path.end());    // 9 8 7 6 5 4 3 2 1
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

  void MPPriorityPlanner::Draw()
  {
    m_scene.Draw();
    m_robots[0]->DrawAbstract();
    for (int i = 0; i< m_robots.size(); i++)
    {
      m_robots[i]->Draw();
    }

    if (IsSolved() == false)
    {
      DrawVertices();
    }
    else
    {
      for (int i = 0; i< m_robots.size(); i++)
      {
        m_robots[i]->DrawCfgPath();
      }
    }
  }

}
