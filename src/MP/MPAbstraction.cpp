#include "MP/MPAbstraction.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Colormap.hpp"
#include <algorithm>

namespace MP
{
  MPAbstraction::MPAbstraction(void)
  {
    m_sim = new MPRobotCar();
    m_minEdgeDist = 5;
    m_maxEdgeDist = 8;
    m_maxNeight = 10;
    m_minClearance = 4;
  }

  void MPAbstraction::CompleteSetup(void)
  {
    m_grid.Setup(2, m_scene->m_grid.GetDims(), m_scene->m_grid.GetMin(), m_scene->m_grid.GetMax());

    //UpdateNeighs();
    // UpdateClearance();
    for(int r = 0 ; r < m_nrRobot; r++)
    {
      m_ridInits[r] = LocateRegion(m_inits[r]);
      m_ridGoals[r] = LocateRegion(m_goals[r]);
    }
    for(int i = 0; i < GetNrRegions(); ++i)
    {
      m_regions[i]->m_colors.resize(m_nrRobot);
      m_regions[i]->m_distFromStarts.resize(m_nrRobot);
      m_regions[i]->m_distToGoals.resize(m_nrRobot);
      m_regions[i]->m_pathsToGoal.resize(m_nrRobot);
    }
    HCosts();
  }

  void MPAbstraction::HCosts(void)
  {
    for (int r = 0 ; r < m_nrRobot; r++)
    {
      HCosts(r);
    }
  }

  void MPAbstraction::HCosts(int rid)
  {
    m_gsi.m_abstraction = this;;
    m_gs.m_info         = &m_gsi;
    m_gsi.m_ridGoal     = Constants::ID_UNDEFINED;

    Region   *r  = NULL;
    const int nr = m_regions.size();
    int       goal;
    double    d;

    m_minDistToGoal[rid] = HUGE_VAL;
    m_maxDistToGoal[rid] = -HUGE_VAL;
    m_gs.AStar(m_ridGoals[rid], false, &goal);

    for(int i = 0; i < nr; ++i)
    {
      r = m_regions[i];

      d = r->m_distToGoals[rid] = m_gs.GetPathCostFromStart(i);
      r->m_pathsToGoal[rid].clear();
      m_gs.GetReversePathFromStart(i, &(r->m_pathsToGoal[rid]));
      if(r->m_pathsToGoal[rid].size() > 0)
      {
        if(m_minDistToGoal[rid] > d)
        {
          m_minDistToGoal[rid] = d;
        }
        if(m_maxDistToGoal[rid] < d)
        {
          m_maxDistToGoal[rid] = d;
        }
      }
    }
    m_gs.AStar(m_ridInits[rid], false, &goal);
    for(int i = 0; i < nr; ++i)
    {
      m_regions[i]->m_distFromStarts[rid] = m_gs.GetPathCostFromStart(i);
      // m_regions[i]->m_colors[rid] = 0;
      m_regions[i]->m_colors[rid] =
      m_regions[i]->m_distFromStarts[rid] <= m_maxDistToGoal[rid] ?
      pow((m_maxDistToGoal[rid] - m_regions[i]->m_distToGoals[rid]) /
      (m_maxDistToGoal[rid] - m_minDistToGoal[rid]), 1.0) : 0;
    }
  }

      bool MPAbstraction::LineOfSight(const int rid1, const int rid2)
      {
        for(int i = 0; i < m_scene->m_obstacles.m_polys.size(); ++i)
        {
          if (m_scene->m_obstacles.m_polys[i]->IntersectSegment(m_regions[rid1]->m_cfg,m_regions[rid2]->m_cfg))
          {
            return false;
          }
        }
        return true;
      }

      void MPAbstraction::Draw()
      {
        // DrawEdges();
        DrawRegions();
      }

      void MPAbstraction::DrawEdges()
      {
        glDisable(GL_LIGHTING);
        GDrawWireframe(true);
        GDrawLineWidth(1.0);
        GDrawColor(1.0, 1.0, 0);
        GDrawPushTransformation();

        GDrawMultTrans(0, 0, m_scene->m_maxGroundHeight+0.35);

        const int n = m_regions.size();
        for(int i = 0; i < n; ++i)
        {
          GDrawIndexColor(1);
          for(auto neigh = m_regions[i]->m_neighs.begin(); neigh != m_regions[i]->m_neighs.end(); ++neigh)
          if(i > (*neigh))
          {
            GDrawSegment2D(m_regions[i]->m_cfg, m_regions[*neigh]->m_cfg);
          }
        }

        GDrawPopTransformation();
        GDrawLineWidth(1.0);
        GDrawWireframe(false);
        glEnable(GL_LIGHTING);
      }

      void MPAbstraction::UpdateClearance(void)
      {
        PQPTriMesh tmesh;
        double factor = 1;
        double offset = 0.01;
        for(int i = 0; i < GetNrRegions(); ++i)
        {
          Region *r = m_regions[i];
          const double *c1 = r->m_cfg;
          const int k = m_regions[i]->m_neighs.size();
          m_regions[i]->m_weights.resize(k);
          for (int j = 0; j < k; j++)
          {
            Region *e = m_regions[m_regions[i]->m_neighs[j]];
            const double *c2 = e->m_cfg;
            tmesh.Clear();
            tmesh.AddQuad(c1,c1,c2,c2);
            double d = tmesh.Distance(NULL, NULL, &m_scene->m_obstacles.m_tmesh, NULL, NULL);
            double w = m_regions[i]->m_dists[j]/(pow(d,factor));
            if (w < Constants::EPSILON)
            {
              w = Constants::EPSILON;
            }
            m_regions[i]->m_weights[j] = w;
            printf("%f\n",w );
          }
        }
      }

      void MPAbstraction::UpdateNeighs(void)
      {

      }

      bool MPAbstraction::EdgesCheck(int fromNode, int toNode, int fromCheckNode, int toCheckNode)
      {
        if ((fromNode == toCheckNode) && (toNode == fromCheckNode))
        {
          return false;
        }

        if (toNode == toCheckNode)
        {
          return false;
        }

        if (Dist2Region(fromNode,fromCheckNode) < m_minClearance)
        {
          double d = Dist2Region(toNode,toCheckNode);
          if (d < m_minClearance)
          {
            return false;
          }
        }
        else
        {
          double cd = DistSegments(m_regions[fromNode]->m_cfg,
                    m_regions[toNode]->m_cfg,
                    m_regions[fromCheckNode]->m_cfg,
                    m_regions[toCheckNode]->m_cfg);
          if (cd<2.5)
          {
            return false;
          }
        }
        return true;
      }

    }
