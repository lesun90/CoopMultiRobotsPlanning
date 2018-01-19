#include "MP/MPFollowPlanner.hpp"
#include <algorithm>
#include "Utils/Geometry.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/GDraw.hpp"

namespace MP
{
  MPFollowPlanner::MPFollowPlanner(void)
  {
    m_radius = 5.0;
    m_tolReach = 1.0;
    m_wbase = 100000000.0;
    m_extendMaxNrSteps  = 80;
    m_extendMinNrSteps  = 40;
    m_probSelectNearestVertex = 0.85;
    m_vidSolved         = -1;
    m_dsel              = 0.99;
    m_nrIters = 0;
    m_maxiters = 120;
    m_maxTime = -1;
  }

  void MPFollowPlanner::Initialize(void)
  {
    if(!(m_sim->IsStateValid()))
    {
      printf("initial state is in collision \n");
      exit(0);
    }

    Vertex *v   = new Vertex();
    if(AddVertex(v) < 0)
    {
      printf("initial state could not be added\n");
      delete v;
      exit(0);
    }
  }

  int MPFollowPlanner::AddVertex(MPFollowPlanner::Vertex * const v)
  {
    if(v->m_cfg == NULL)
    {
      v->m_cfg = m_sim->NewCfg();
    }
    if(v->m_s == NULL)
    {
      v->m_s = m_sim->NewState();
    }

    m_sim->GetCfg(v->m_cfg);
    m_sim->GetState(v->m_s);
    v->m_timeid = v->m_parent >= 0 ? m_vertices[v->m_parent]->m_timeid+1 : 0;
    v->m_nextWaypt = v->m_parent >= 0 ? m_vertices[v->m_parent]->m_nextWaypt : 0;

    if (CheckWithReserveTable(v) == false)
    {
      return -1;
    }

    int lastWpt = std::max(v->m_nextWaypt-1,0);
    if (IsReachedWaypt(v->m_cfg,v->m_nextWaypt) == true)
    {
      if(v->m_nextWaypt < m_waypts.size()-1)
      {
        ++(v->m_nextWaypt);
      }
    }
    else if(IsInside(v->m_nextWaypt,lastWpt,v->m_cfg) == false)
    {
      return -1;
    }

    m_vertices.push_back(v);

    v->m_id = m_vertices.size()-1;

    Group *g;
    auto it = m_groups.find(v->m_nextWaypt);
    if(it == m_groups.end())
    {
      g = new Group();
      g->m_id = v->m_nextWaypt;
      g->m_weight = Weight(g->m_id);
      g->m_dist2goal = DistFromWpToGoal(v->m_nextWaypt);
      m_groups.insert(std::make_pair(g->m_id, g));
    }
    else
    {
      g = it->second;
    }

    g->m_vids.push_back(m_vertices.size() - 1);

    if((IsReachedGoal(v->m_cfg) == true) && (m_sim->GetSpeed()<0.1))
    {
      m_vidSolved = m_vertices.size() - 1;
    }

    return m_vertices.size() - 1;
  }

  void MPFollowPlanner::Run()
  {
    m_nrIters = 0;
    if(m_vertices.size() == 0)
    {
      Initialize();
    }

    Group *g;
    double target[100];
    int    vid;

    while(IsSolved()==false)
    {
      g = SelectGroup();

      SelectTarget(g, target);
      vid = SelectVertex(g, target);
      ExtendFrom(vid, target);
      m_nrIters++;

      g->m_weight *= m_dsel;
      if(g->m_weight < Constants::EPSILON)
      for(auto & it : m_groups)
      it.second->m_weight /= m_dsel;
    }
    if (IsSolved() == true)
    {
      GetPath();
    }
  }

  void MPFollowPlanner::Run(const int nrIters)
  {
    if(m_vertices.size() == 0)
    {
      Initialize();
    }

    Group *g;
    double target[100];
    int    vid;

    for(int i = 0; i < nrIters && m_vidSolved < 0; ++i)
    {
      g = SelectGroup();
      SelectTarget(g, target);
      vid = SelectVertex(g, target);
      ExtendFrom(vid, target);

      g->m_weight *= m_dsel;
      if(g->m_weight < Constants::EPSILON)
      for(auto & it : m_groups)
      it.second->m_weight /= m_dsel;

      m_nrIters++;
      if (IsSolved() == true)
      {
        GetPath();
      }
    }
  }

  bool MPFollowPlanner::IsSolved(void)
  {
    // printf("m_nrIters %d m_vidSolved %d\n",m_nrIters,m_vidSolved);
    return ((m_nrIters>=m_maxiters) || (m_vidSolved >=0));
  }

  bool MPFollowPlanner::IsReachedGoal(double *cfg)
  {
    return Algebra2D::PointDist(cfg,m_waypts.back().m_cfg) < m_waypts.back().m_tolReach;
  }

  MPFollowPlanner::Group* MPFollowPlanner::SelectGroup(void)
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

  MPFollowPlanner::Group* MPFollowPlanner::SelectBestGroup(void)
  {
    double wmin = HUGE_VAL;
    Group *gmax = NULL;

    for(auto & it : m_groups)
    if(it.second->m_dist2goal < wmin)
    {
      wmin = it.second->m_dist2goal;
      gmax = it.second;
    }
    return gmax;
  }

  int MPFollowPlanner::SelectVertex(const Group * g, const double target[])
  {
    if(RandomUniformReal() < 1 - m_probSelectNearestVertex)
    return g->m_vids[RandomUniformInteger(0, g->m_vids.size() - 1)];

    double dmin = HUGE_VAL;
    int    imin = -1;
    double d;

    for(auto &it : g->m_vids)
    if((d = Algebra2D::PointDist(target, m_vertices[it]->m_cfg)) < dmin)
    {
      dmin = d;
      imin = it;
    }
    return imin;
  }

  void MPFollowPlanner::SelectTarget(MPFollowPlanner::Group *g, double target[])
  {
    const double *wpt = m_waypts[g->m_id].m_cfg;
    const double *lastwpt = m_waypts[GetLastWaypt(g->m_id)].m_cfg;

    if (g->m_id == (m_waypts.size()-1))
    {
      if(RandomUniformReal() < 0.8)
      SampleRandomPointInsideCircle2D(wpt, m_waypts[g->m_id].m_tolReach, target);
      else
      SampleRandomPointInsideCircle2D(wpt, m_waypts[g->m_id].m_radius, target);
      return;
    }

    if ((GetLastWaypt(g->m_id) == 0) || (Algebra2D::PointDist(wpt,lastwpt) < 1.0 ))
    {
      if(RandomUniformReal() < 0.1)
      SampleRandomPointInsideCircle2D(wpt, m_waypts[g->m_id].m_tolReach, target);
      else
      SampleRandomPointInsideCircle2D(wpt, m_waypts[g->m_id].m_radius, target);
      return;
    }

    SampleRandomPointInsideCircle2D(wpt, m_waypts[g->m_id].m_radius, target);
    for (int i = 0; i < 40 && (Algebra2D::PointDist(target,lastwpt)<m_waypts[g->m_id].m_radius); i++)
    {
      if(RandomUniformReal() < 0.1)
      SampleRandomPointInsideCircle2D(wpt, m_waypts[g->m_id].m_tolReach, target);
      else
      SampleRandomPointInsideCircle2D(wpt, m_waypts[g->m_id].m_radius, target);
    }


  }

  void MPFollowPlanner::ExtendFrom(const int vid, const double target[])
  {
    int          parent  = vid;
    const int    nrSteps = RandomUniformInteger(m_extendMinNrSteps, m_extendMaxNrSteps);

    m_sim->SetState(m_vertices[vid]->m_s);
    if(m_vertices[parent]->m_nextWaypt == GetNrWaypts()-1)
    {
      m_sim->StartSteerToStop();
    }
    else
    {
      m_sim->StartSteerToPosition();
    }

    for(int i = 0; i < nrSteps && m_vidSolved < 0; ++i)
    {
      if(m_vertices[parent]->m_nextWaypt == GetNrWaypts()-1)
      {
        m_sim->SteerToStop(target);
      }
      else
      {
        m_sim->SteerToPosition(target);
      }
      m_sim->SimulateOneStep();
      if(!m_sim->IsStateValid())
      return;

      Vertex *vnew = new Vertex();
      vnew->m_parent = parent;

      if(AddVertex(vnew) < 0)
      {
        delete vnew;
        return;
      }

      if(m_sim->HasReachedPosition(target, m_tolReach))
      return;
      parent = m_vertices.size() - 1;
    }
  }

  bool MPFollowPlanner::CheckWithReserveTable(Vertex* v)
  {
    int checkid = v->m_timeid;
    for (int r = 0 ; r < m_reserveTable.size(); r++)
    {
      if (checkid > ((int)m_reserveTable[r].size()-1))
      {
        checkid = (int)m_reserveTable[r].size()-1;
      }
      if (Algebra2D::PointDist(v->m_cfg,m_reserveTable[r][checkid]) < 5)
      {
        printf("id :%d\n",m_sim->m_id );
        return false;
      }
    }

    return true;
  }

  bool MPFollowPlanner::IsInside(int wpt1, int wpt2,const double p[])
  {
    double        pmin[2];
    return DistSquaredPointSegment2D(p, m_waypts[wpt1].m_cfg, m_waypts[wpt2].m_cfg, pmin)
    <= m_waypts[wpt1].m_radius * m_waypts[wpt2].m_radius;
  }

  void MPFollowPlanner::GetPath()
  {
    if ((m_vidSolved < 0) && (m_nrIters >= m_maxiters))
    {
      Group *g;
      g = SelectBestGroup();
      m_vidSolved = SelectVertex(g,m_waypts.back().m_cfg);
    }

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

    for (int i = 0; i<m_path.size();++i)
    {
      m_statePath.push_back(m_vertices[m_path[i]]->m_s);
      m_cfgPath.push_back(m_vertices[m_path[i]]->m_cfg);
    }

    if (m_maxTime < 0)
    return;
    
    if((m_nrIters < m_maxiters) && (m_maxTime < m_path.size()))
    {
      for (int i = m_path.size(); i<m_maxTime;++i)
      {
        m_statePath.push_back(m_statePath.back());
        m_cfgPath.push_back(m_cfgPath.back());
      }
    }
  }


  void MPFollowPlanner::Draw()
  {
    // m_sim->m_scene->Draw();
    m_sim->Draw();
    for(int i = m_vertices.size() - 1; i >= 1; --i)
    {
      GDrawSegment2D(m_vertices[i]->m_cfg, m_vertices[m_vertices[i]->m_parent]->m_cfg);
    }

    const int n = GetNrWaypts();
  	for(int i = 0; i < n - 1; ++i)
  	{
  	    const double *p1   = m_waypts[i].m_cfg;
  	    const double *p2   = m_waypts[i+1].m_cfg;
  	    const double  d    = m_waypts[i].m_radius;
  	    const double  vx   = p2[0] - p1[0];
  	    const double  vy   = p2[1] - p1[1];
  	    const double  norm = sqrt(vx * vx + vy * vy);
  	    const double  ux   = -vy * d / norm;
  	    const double  uy   =  vx * d / norm;

  	    GDrawSegment2D(p1, p2);
  	    GDrawSegment2D(p1[0] + ux, p1[1] + uy, p2[0] + ux, p2[1] + uy);
  	    GDrawSegment2D(p1[0] - ux, p1[1] - uy, p2[0] - ux, p2[1] - uy);
  	}

  	GDrawWireframe(true);
  	for(int i = 0; i < n; ++i)
    {
      // GDrawCircle2D(m_waypts[i].m_cfg, m_waypts[i].m_radius);
    }
  	GDrawWireframe(false);

  	for(int i = 0; i < n; ++i)
    {
      // GDrawCircle2D(m_waypts[i].m_cfg, m_waypts[i].m_tolReach);
    }

  	char msg[100];

  	GDrawColor(0, 0, 0);
  	for(int i = 0; i < n; ++i)
  	{
  	    sprintf(msg, "%d", i);
  	    GDrawString2D(msg, m_waypts[i].m_cfg[0], m_waypts[i].m_cfg[1]);
  	}

  }

}
