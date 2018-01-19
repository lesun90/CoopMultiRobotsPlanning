#include "MP/MPPrmAbstraction.hpp"
#include "MP/MPSimulator.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/Timer.hpp"
#include "External/ShewchukTriangle.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Colormap.hpp"
#include "Utils/Definitions.hpp"
#include "Utils/Map.hpp"
#include "Utils/Heap.hpp"
#include "Utils/Misc.hpp"
#include <stack>
#include <queue>
#include <cstdio>

namespace MP
{
  void MPPrmAbstraction::SamplePointInsideRegion(const int rid, double p[], const double dtol) const
  {
    if(m_sim->GetDimPos() == 2)
    SampleRandomPointInsideCircle2D(m_regions[rid]->m_cfg, dtol, p);
    else
    SampleRandomPointInsideSphere3D(m_regions[rid]->m_cfg, dtol, p);
  }

  MPPrmAbstraction::MPPrmAbstraction(void) : MPAbstraction()
  {
    m_proximity.m_distFn     = DistFn;
    m_proximity.m_distFnData = this;
    m_step                   = 0.1;
    m_ridConnect             = 0;
    m_method                 = METHOD_STANDARD;
    m_auxCfg                 = NULL;
    m_nrSamples              = Constants::ABSTRACTION_NRSAMPLE;
    m_nrNeighs               = 10;
    m_minEdgeDist            = 5;
    m_maxEdgeDist            = 8;
    m_nrTry = 1000;
    m_factor = 0.95;
    m_minconnectEdgeDist = m_minEdgeDist;
    m_maxconnectEdgeDist = m_maxEdgeDist;

  }

  MPPrmAbstraction::~MPPrmAbstraction(void)
  {
    if(m_auxCfg)
    delete[] m_auxCfg;
  }


  void MPPrmAbstraction::CompleteSetup(void)
  {
    m_triAbs = new MPTriAbstraction();
    m_triAbs->SetScene(m_scene);
    m_triAbs->m_grid.Setup(2, m_scene->m_grid.GetDims(), m_scene->m_grid.GetMin(), m_scene->m_grid.GetMax());
    m_triAbs->m_areaConstraint = 25;
    m_triAbs->Triangulate();
    m_triAbs->LocatorCompleteSetup();

    m_nrTry = m_triAbs->m_regions.size()*2;
    // printf("m_triAbs->m_regions.size(): %d\n", m_triAbs->m_regions.size());
    m_grid.Setup(2, m_scene->m_grid.GetDims(), m_scene->m_grid.GetMin(), m_scene->m_grid.GetMax());
    m_minTempEdges.resize(m_nrRobot);
    for (int r = 0 ; r < m_nrRobot; r++)
    {
      m_minTempEdges[r].m_dist = 10000;
    }
    m_auxCfg = m_sim->NewCfg();
    m_sim->SampleCfg(m_auxCfg);
    GenerateSamples();

    Timer::Clock clk;
    Timer::Start(&clk);
    int count = 0;
    while ((IsAllInitConnectedToGoal() == false) && (Timer::Elapsed(&clk)<25))
    {
      // GenerateRandomSample();
      // GenerateEdges(m_nrNeighs);
      m_minconnectEdgeDist = m_minconnectEdgeDist * 0.8;
      // m_maxconnectEdgeDist = m_maxconnectEdgeDist * 1.25;
      if (m_minconnectEdgeDist < m_minEdgeDist*0.5)
      {
        m_minconnectEdgeDist = m_minEdgeDist;
      }
      // if (m_maxconnectEdgeDist > m_maxconnectEdgeDist*1.5)
      // {
      //   m_maxconnectEdgeDist = m_maxEdgeDist;
      // }
      for(int i = 0 ; i < m_triAbs->m_regions.size(); i++)
      {
        if (m_triAbs->m_regions[i]->m_hasSample == false)
        {
          GenerateSampleAtRegion(i);
          GenerateEdges(10);
        }
      }

      if (IsAllInitConnectedToGoal() == false)
      {
        GenerateRandomSample();
        GenerateEdges(10);
      }
    }
    // Stats::GetSingleton()->AddValue("TimeFix", Timer::Elapsed(&clk));

    // if (m_disjointSet.GetNrComponents()>1)
    // {
    //   FixEdges();
    //   printf("m_nrComps %d\n",m_disjointSet.GetNrComponents() );
    // }
    MPAbstraction::CompleteSetup();
  }


  int MPPrmAbstraction::GenerateSamples()
  {
    std::queue<int>  q;
    m_minconnectEdgeDist = 0.5;
    std::vector<int> goalinitregions;
    for(int r = 0; r < m_nrRobot; ++r)
    {
      int regionLocator;
      regionLocator = m_triAbs->LocateRegion(m_goals[r]);
      goalinitregions.push_back(regionLocator);
      AddNewRegion(m_goals[r]);
      m_triAbs->m_regions[regionLocator]->m_visited = true;
      for (int i = 0 ; i < m_triAbs->m_regions[regionLocator]->m_neighs.size(); i++)
      {
        int key = m_triAbs->m_regions[regionLocator]->m_neighs[i];

        if (m_triAbs->m_regions[key]->m_visited == true)
        {
          continue;
        }
        q.push(key);
      }
      GenerateEdges(m_nrNeighs);
      m_ridInits[r] = m_regions.size() - 1;


      regionLocator = m_triAbs->LocateRegion(m_inits[r]);
      bool add = true;
      for (int k = 0 ; k < goalinitregions.size(); k++)
      {
        if (goalinitregions[k] == regionLocator)
        {
          add = false;
        }
      }
      if (add == false)
      {
        continue;
      }
      goalinitregions.push_back(regionLocator);

      AddNewRegion(m_inits[r]);
      m_triAbs->m_regions[regionLocator]->m_visited = true;
      for (int i = 0 ; i < m_triAbs->m_regions[regionLocator]->m_neighs.size(); i++)
      {
        int key = m_triAbs->m_regions[regionLocator]->m_neighs[i];

        if (m_triAbs->m_regions[key]->m_visited == true)
        {
          continue;
        }
        q.push(key);
      }
      GenerateEdges(m_nrNeighs);
      m_ridGoals[r] = m_regions.size() - 1;
    }
    m_minconnectEdgeDist = m_minEdgeDist;

    int u;
    while(!q.empty())
    {
      u = q.front();
      q.pop();

      if (m_triAbs->m_regions[u]->m_visited == true)
      {
          continue;
      }
      GenerateSampleAtRegion(u);
      GenerateEdges(m_nrNeighs);
      m_triAbs->m_regions[u]->m_visited = true;

      for (int i = 0 ; i < m_triAbs->m_regions[u]->m_neighs.size(); i++)
      {
        int key = m_triAbs->m_regions[u]->m_neighs[i];

        if (m_triAbs->m_regions[key]->m_visited == true)
        {
          continue;
        }
        q.push(key);
      }
    }



    for(int i = 0 ; i < m_triAbs->m_regions.size(); i++)
    {
      if (m_triAbs->m_regions[i]->m_hasSample == false)
      {
        GenerateSampleAtRegion(i);
        GenerateEdges(m_nrNeighs);
      }
    }
  }

  void MPPrmAbstraction::GenerateSampleAtRegion(const int region)
  {
    if (m_triAbs->m_regions[region]->m_valid == false)
    {
      return;
    }
    int nrTry = RandomUniformInteger(10,20);
    ProximityQuery<int> q;
    q.SetKey(-1);

    for (int i = 0 ; i < nrTry; i++)
    {
      double *cfg;
      cfg = m_sim->NewCfg();
      m_sim->SampleCfg(cfg);
      m_triAbs->SamplePointInsideRegion(region,cfg,5);

      if (m_sim->IsCfgValid(cfg) == false)
      {
        continue;
      }
      m_sim->CopyCfg(m_auxCfg, cfg);
      if(!m_proximity.IsDataStructureConstructed())
      m_proximity.ConstructDataStructure();
      // const double dobs = m_sim->MinDistFromCfgToObstacles(cfg);
      const int neigh = m_proximity.Neighbor(&q);
      const double d = Algebra2D::PointDist(cfg, m_regions[neigh]->m_cfg);
      const double dobs = m_sim->MinDistFromCfgToObstacles(cfg);

      if ((d > m_minconnectEdgeDist) && (d < m_maxconnectEdgeDist)&&(dobs > 0.5))
      {
        AddNewRegion(cfg);
        m_triAbs->m_regions[region]->m_hasSample = true;
        return;
      }
    }
    return;
  }


  void MPPrmAbstraction::GenerateRandomSample()
  {
    ProximityQuery<int> q;
    q.SetKey(-1);
    double *cfg;
    cfg = m_sim->NewCfg();
    m_sim->SampleCfg(cfg);
    m_sim->CopyCfg(m_auxCfg, cfg);

    if(!m_proximity.IsDataStructureConstructed())
    m_proximity.ConstructDataStructure();

    const int neigh = m_proximity.Neighbor(&q);
    const double d = m_sim->DistanceCfg(cfg, m_regions[neigh]->m_cfg);
    const double dobs = m_sim->MinDistFromCfgToObstacles(m_sim->GetCfg());
    if ((d >= (m_minconnectEdgeDist)) && (dobs > 0.5))
    {
      AddNewRegion(cfg);
    }
    return;
  }

  int MPPrmAbstraction::LocateRegion(const double p[])
  {
    if(!m_proximity.IsDataStructureConstructed())
    m_proximity.ConstructDataStructure();

    m_sim->CopyCfg(m_auxCfg, p);

    ProximityQuery<int> q;
    q.SetKey(-1);

    return m_proximity.Neighbor(&q);
  }

  double MPPrmAbstraction::DistFn(const int rid1,
    const int rid2,
    MPPrmAbstraction * const aprm)
    {

      const double *cfg1 = rid1 >= 0 ? aprm->m_regions[rid1]->m_cfg : aprm->m_auxCfg;
      const double *cfg2 = rid2 >= 0 ? aprm->m_regions[rid2]->m_cfg : aprm->m_auxCfg;

      return Algebra2D::PointDist(cfg1, cfg2);
    }

    int MPPrmAbstraction::GenerateEdges(const int nneighs)
    {
      if(!m_proximity.IsDataStructureConstructed())
      m_proximity.ConstructDataStructure();

      ProximityQuery<int>   query;
      ProximityResults<int> res;

      query.SetNrNeighbors(nneighs);

      while(m_ridConnect < m_regions.size())
      {
        query.SetKey(m_ridConnect);
        m_proximity.Neighbors(&query, &res);

        const int n = res.GetNrResults();
        for(int i = 0; i < n; ++i)
        {
          const int rid = res.GetKey(i);
          GenerateEdge(m_ridConnect, rid);
        }

        ++m_ridConnect;
      }

      return m_regions.size() - m_ridConnect;
    }

    void MPPrmAbstraction::FixEdges()
    {
      while (IsAllInitConnectedToGoal() == false)
      {
        for (int r = 0; r < m_nrRobot; r++)
        {
          if (IsInitConnectedToGoal(r) == true)
          {
            continue;
          }

          int rid1 = LocateRegion(m_inits[r]);
          int rid2 = LocateRegion(m_goals[r]);
          PrmRegion *reg1 = dynamic_cast<PrmRegion*>(m_regions[rid1]);
          PrmRegion *reg2 = dynamic_cast<PrmRegion*>(m_regions[rid2]);

          double minDist;
          int tempA, tempB;

          minDist = 100000;
          tempA = -1;
          tempB = -1;
          for (int i = 0; i < m_regions.size(); i++)
          {
            PrmRegion *regA = dynamic_cast<PrmRegion*>(m_regions[i]);
            if (m_disjointSet.Same(regA->m_dset,reg1->m_dset) == true)
            {
              for (int j = 0; j < m_regions.size(); j++)
              {
                if ((LineOfSight(i,j) == false) || (i == j))
                {
                  continue;
                }
                PrmRegion *regB = dynamic_cast<PrmRegion*>(m_regions[j]);
                if (m_disjointSet.Same(regB->m_dset,reg1->m_dset) == false)
                {
                  double d = Algebra2D::PointDist(regA->m_cfg,regB->m_cfg);
                  if (d < minDist)
                  {
                    minDist = d;
                    tempA = i;
                    tempB = j;
                  }
                }
              }
            }
          }
          if (minDist!=100000)
          {
            printf("fix at robot %d %d %d\n", r, tempA,tempB );
            PrmRegion *regf1 = dynamic_cast<PrmRegion*>(m_regions[tempA]);
            PrmRegion *regf2 = dynamic_cast<PrmRegion*>(m_regions[tempB]);
            regf1->m_neighs.push_back(tempB);
            regf1->m_dists.push_back(minDist);
            regf1->m_weights.push_back(minDist);

            regf2->m_neighs.push_back(tempA);
            regf2->m_dists.push_back(minDist);
            regf2->m_weights.push_back(minDist);

            m_disjointSet.Join(regf1->m_dset, regf2->m_dset);
          }


          minDist = 100000;
          tempA = -1;
          tempB = -1;
          for (int i = 0; i < m_regions.size(); i++)
          {
            PrmRegion *regA = dynamic_cast<PrmRegion*>(m_regions[i]);
            if (m_disjointSet.Same(regA->m_dset,reg2->m_dset) == true)
            {
              for (int j = 0; j < m_regions.size(); j++)
              {
                if ((LineOfSight(i,j) == false) || (i == j))
                {
                  continue;
                }
                PrmRegion *regB = dynamic_cast<PrmRegion*>(m_regions[j]);
                if (m_disjointSet.Same(regB->m_dset,reg2->m_dset) == false)
                {
                  double d = Algebra2D::PointDist(regA->m_cfg,regB->m_cfg);
                  if (d < minDist)
                  {
                    minDist = d;
                    tempA = i;
                    tempB = j;
                  }
                }
              }
            }
          }
          if (minDist!=100000)
          {
            printf("fix at robot %d %d %d\n", r, tempA,tempB );
            PrmRegion *regf3 = dynamic_cast<PrmRegion*>(m_regions[tempA]);
            PrmRegion *regf4 = dynamic_cast<PrmRegion*>(m_regions[tempB]);
            regf3->m_neighs.push_back(tempB);
            regf3->m_dists.push_back(minDist);
            regf3->m_weights.push_back(minDist);

            regf4->m_neighs.push_back(tempA);
            regf4->m_dists.push_back(minDist);
            regf4->m_weights.push_back(minDist);

            m_disjointSet.Join(regf3->m_dset, regf4->m_dset);
          }




        }
      }

    }

    bool MPPrmAbstraction::GenerateEdge(const int rid1, const int rid2)
    {
      if (LineOfSight(rid1,rid2) == false)
      {
        return false;
      }

      PrmRegion *reg1 = dynamic_cast<PrmRegion*>(m_regions[rid1]);
      PrmRegion *reg2 = dynamic_cast<PrmRegion*>(m_regions[rid2]);

      if(FindItem<int>(&(reg1->m_attempts), rid2) >= 0)
      return false;

      reg1->m_attempts.push_back(rid2);
      reg2->m_attempts.push_back(rid1);

      const double *cfg1    = reg1->m_cfg;
      const double *cfg2    = reg2->m_cfg;
      const double  d       = Algebra2D::PointDist(cfg1,cfg2);
      const int     nrSteps = d / 0.1;

      if ((d > m_maxconnectEdgeDist) || (d < m_minconnectEdgeDist))
      {
        return false;
      }
      reg1->m_neighs.push_back(rid2);
      reg1->m_dists.push_back(d);
      reg1->m_weights.push_back(d);

      reg2->m_neighs.push_back(rid1);
      reg2->m_dists.push_back(d);
      reg2->m_weights.push_back(d);

      if (m_minClearance > d)
      {
        m_minClearance = d;
      }

      m_disjointSet.Join(reg1->m_dset, reg2->m_dset);

      return true;
    }


      void MPPrmAbstraction::AddNewRegion(const double cfg[])
      {
        PrmRegion *r = new PrmRegion();
        r->m_cfg = m_sim->NewCfg();
        m_sim->CopyCfg(r->m_cfg, cfg);
        r->m_dset = m_disjointSet.Make();
        m_regions.push_back(r);
        m_proximity.AddKey(m_regions.size() - 1);
      }


      void MPPrmAbstraction::DrawRegion(const int rid)
      {
        GDrawWireframe(true);
        const double c = m_regions[rid]->m_colors[0];
        GDrawColor(Colormap::m_singleton->GetRed(c),
        Colormap::m_singleton->GetGreen(c),
        Colormap::m_singleton->GetBlue(c));
        // GDrawLineWidth(1.0);
        m_sim->DrawCfgShape(m_regions[rid]->m_cfg);
        GDrawWireframe(false);
      }

      void MPPrmAbstraction::DrawRegions(void)
      {
        glDisable(GL_LIGHTING);
        GDrawPushTransformation();
        // GDraw2D();
        GDrawMultTrans(0, 0, m_scene->m_maxGroundHeight+0.45);
        for (int i = 0; i < m_regions.size(); i++)
        {
          DrawRegion(i);
        }
        GDrawPopTransformation();
        glEnable(GL_LIGHTING);
        // m_triAbs->DrawTriangles();
      }

    }
