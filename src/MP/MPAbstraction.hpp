#ifndef MP_ABSTRACTION_HPP_
#define MP_ABSTRACTION_HPP_

#include "MP/MPScene.hpp"
#include "MP/Constants.hpp"
#include "MP/MPSimulator.hpp"
#include "MP/MPRobotCar.hpp"
#include "Utils/GraphSearch.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/Algebra.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Grid.hpp"
#include "Utils/Flags.hpp"
#include "Utils/TriMesh.hpp"
#include "External/PQP/PQPTriMesh.hpp"
#include <vector>
#include <cstdio>

namespace MP
{
  class MPAbstraction
  {
  public:
    MPAbstraction(void);

    virtual ~MPAbstraction(void)
    {
      DeleteItems<Region*>(&m_regions);
    }

    virtual void CompleteSetup(void);

    virtual void SetScene(MPScene *scene)
    {
      m_scene = scene;
      m_sim->SetScene(m_scene);
      m_nrRobot = m_scene->m_robotInit.size();
      m_inits.resize(m_nrRobot);
      m_ridInits.resize(m_nrRobot);
      m_goals.resize(m_nrRobot);
      m_ridGoals.resize(m_nrRobot);
      m_minDistToGoal.resize(m_nrRobot);
      m_maxDistToGoal.resize(m_nrRobot);

      for(int r = 0 ; r < m_nrRobot; r++)
      {
        m_goals[r] = m_scene->m_goals[r]->m_cfg;
        m_inits[r] = m_scene->m_robotInit[r]->m_cfg;
      }

    }

    virtual void HCosts(void);
    virtual void HCosts(int rid);

    virtual int  LocateRegion(const double p[]) = 0;
    virtual bool IsInsideRegion(const int rid, const double p[])
    {
      return LocateRegion(p) == rid;
    }
    virtual bool SplitRegion(const int rid, std::vector<int> *new_rids) = 0;
    virtual void SamplePointInsideRegion(const int rid, double p[], const double dtol) const = 0;

    virtual void DrawRegion(const int rid) = 0;
    virtual void DrawRegions() = 0;
    virtual void Draw();
    virtual void DrawEdges();

    virtual void UpdateNeighs(void);

    virtual void MaxZIntersect(TriMesh * const tmesh)
    {
    }

    virtual int GetNrRegions()
    {
      return m_regions.size();
    }
    virtual void UpdateClearance(void);

    virtual bool LineOfSight(const int rid1, const int rid2);

    virtual bool EdgesCheck(int fromNode, int toNode, int fromCheckNode, int toCheckNode);

    virtual double Dist2Region(int a, int b)
    {
      if (a==b)
      return 0;
      return Algebra2D::PointDist(m_regions[a]->m_cfg,m_regions[b]->m_cfg);
    }

    virtual double MaxCostToNeighbors(int k)
    {
      double maxVal = -1;
      for (int i = 0 ; i < (int)m_regions[k]->m_weights.size(); i++)
      {
        if (maxVal < m_regions[k]->m_weights[i])
        {
          maxVal = m_regions[k]->m_weights[i];
        }
      }
      return maxVal;
    }

    virtual double MinCostToNeighbors(int k)
    {
      double minVal = HUGE_VAL;
      for (int i = 0 ; i < (int)m_regions[k]->m_weights.size(); i++)
      {
        if (minVal > m_regions[k]->m_weights[i])
        {
          minVal = m_regions[k]->m_weights[i];
        }
      }
      return minVal;
    }

    struct Region
    {
      Region(void)
      {
        m_label         = Constants::ID_UNDEFINED;
        m_vol           = 0;
        m_cfg           = new double[2];
        m_valid         = true;
        m_visited       = false;
        m_hasSample     = false;
      }

      virtual ~Region(void)
      {
        if(m_cfg)
        delete[] m_cfg;
      }

      double                       *m_cfg;
      std::vector<int>              m_neighs;
      std::vector<double>           m_dists;
      std::vector<double>           m_weights;
      double                        m_dclear;
      int                           m_label;
      double                        m_vol;
      bool                          m_valid;
      std::vector<double>           m_colors;
      std::vector<double>           m_distFromStarts;
      std::vector<double>           m_distToGoals;
      std::vector<std::vector<int>> m_pathsToGoal;
      bool                          m_visited;
      bool                          m_hasSample;
      std::vector<std::pair<int,int>> m_intervals;
    };

    MPScene                *m_scene;
    MPSimulator          *m_sim;
    std::vector<Region*>  m_regions;
    int                   m_nrRobot;
    std::vector<double*>  m_inits;
    std::vector<int>      m_ridInits;
    std::vector<double*>  m_goals;
    std::vector<int>      m_ridGoals;

    std::vector<double>   m_minDistToGoal;
    std::vector<double>   m_maxDistToGoal;

    double m_minEdgeDist;
    double m_maxEdgeDist;
    double m_minClearance;
    int   m_maxNeight;
    double tttt;
    class RegionGraphSearchInfo : public GraphSearchInfo<int>
    {
    public:
      RegionGraphSearchInfo(void) : GraphSearchInfo<int>(),
      m_abstraction(NULL),
      m_ridGoal(Constants::ID_UNDEFINED)
      {
      }

      void GetOutEdges(const int u,
        std::vector<int> * const edges,
        std::vector<double> * const costs = NULL) const
        {
          if(edges)
          edges->assign(m_abstraction->m_regions[u]->m_neighs.begin(),
          m_abstraction->m_regions[u]->m_neighs.end());
          if(costs)
          costs->assign(m_abstraction->m_regions[u]->m_weights.begin(),
          m_abstraction->m_regions[u]->m_weights.end());
        }

      bool IsGoal(const int key) const
      {
        return key == m_ridGoal;
      }

      double HeuristicCostToGoal(const int u) const
      {
        return m_ridGoal >= 0 ? Algebra2D::PointDist(m_abstraction->m_regions[u]->m_cfg,
            m_abstraction->m_regions[m_ridGoal]->m_cfg) : 0;
      }

      MPAbstraction *m_abstraction;
      int            m_ridGoal;
    };



    RegionGraphSearchInfo  m_gsi;
    GraphSearch<int>       m_gs;
    Grid                   m_grid;

    };
  }

  #endif
