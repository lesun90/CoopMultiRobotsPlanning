#include "MP/MPFollowPlanner.hpp"
#include <algorithm>
#include "Utils/Geometry.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/GDraw.hpp"

namespace MP
{
  MPFollowPlanner::MPFollowPlanner(void)
  {
    m_tolReach = 1.0;
    m_radius = 3.0;
    m_wbase = 100000000.0;
    m_extendMaxNrSteps  = 80;
    m_extendMinNrSteps  = 40;
    m_probSelectNearestVertex = 0.9;
    m_vidSolved         = -1;
    m_dsel              = 0.99;
  }

  void MPFollowPlanner::Initialize(void)
  {
    if(!(m_sim->IsStateValid()))
    {
      printf("initial state is in collision\n");
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

    v->m_nextWaypt = v->m_parent >= 0 ? m_vertices[v->m_parent]->m_nextWaypt : 0;
    int lastWpt = std::max(v->m_nextWaypt-1,0);
    if (Algebra2D::PointDistSquared(v->m_cfg,m_waypts[v->m_nextWaypt])< m_tolReach*m_tolReach )
    {
      ++(v->m_nextWaypt);
    }
    else if(IsInside(m_waypts[v->m_nextWaypt],m_waypts[lastWpt],v->m_cfg) == false)
    {
      return -1;
    }

    m_vertices.push_back(v);

    Group *g;
    auto it = m_groups.find(v->m_nextWaypt);
    if(it == m_groups.end())
    {
      g = new Group();
      g->m_id = v->m_nextWaypt;
      g->m_weight = Weight(g->m_id);
      m_groups.insert(std::make_pair(g->m_id, g));
    }
    else
    g = it->second;
    g->m_vids.push_back(m_vertices.size() - 1);

    if(v->m_nextWaypt >= GetNrWaypts())
    m_vidSolved = m_vertices.size() - 1;
    return m_vertices.size() - 1;
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
    }
  }

  MPFollowPlanner::Group* MPFollowPlanner::SelectGroup(void)
  {
    double wmax = -HUGE_VAL;
    Group *gmax = NULL;

    for(auto & it : m_groups)
    if(it.second->m_weight > wmax)
    {
      wmax = it.second->m_weight;
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
    const double *wpt = m_waypts[g->m_id];
    if(RandomUniformReal() < 0.1)
    SampleRandomPointInsideCircle2D(wpt, m_tolReach, target);
    else
    SampleRandomPointInsideCircle2D(wpt, m_radius, target);
  }

  void MPFollowPlanner::ExtendFrom(const int vid, const double target[])
  {
    //printf("extending from %d/%d\n", vid, m_vertices.size());

    int          parent  = vid;
    const int    nrSteps = RandomUniformInteger(m_extendMinNrSteps, m_extendMaxNrSteps);

    m_sim->SetState(m_vertices[vid]->m_s);
    m_sim->StartSteerToPosition();

    for(int i = 0; i < nrSteps && m_vidSolved < 0; ++i)
    {
      m_sim->SteerToPosition(target);
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
  	    const double *p1   = m_waypts[i];
  	    const double *p2   = m_waypts[i+1];
  	    const double  d    = m_radius;
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
      GDrawCircle2D(m_waypts[i], m_radius);
    }
  	GDrawWireframe(false);

  	for(int i = 0; i < n; ++i)
    {
      GDrawCircle2D(m_waypts[i], m_tolReach);
    }

  	char msg[100];

  	GDrawColor(0, 0, 0);
  	for(int i = 0; i < n; ++i)
  	{
  	    sprintf(msg, "%d", i);
  	    GDrawString2D(msg, m_waypts[i][0], m_waypts[i][1]);
  	}

  }

}
