#ifndef MP_PRM_ABSTRACTION_HPP_
#define MP_PRM_ABSTRACTION_HPP_

#include "MP/MPAbstraction.hpp"
#include "MP/MPTriAbstraction.hpp"
#include "Utils/ProximityGNAT.hpp"
#include "Utils/TriMesh.hpp"
#include "Utils/DisjointSet.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Polygon2D.hpp"
#include "Utils/Misc.hpp"

namespace MP
{
  class MPPrmAbstraction : public MPAbstraction
  {
  public:
    MPPrmAbstraction(void);

    virtual ~MPPrmAbstraction(void);

    virtual void CompleteSetup(void);

    virtual int LocateRegion(const double p[]);

    virtual void SamplePointInsideRegion(const int rid, double p[], const double dtol) const;

    virtual bool SplitRegion(const int rid, std::vector<int> *new_rids)
    {
      return false;
    }
    virtual void GenerateRandomSample();

    virtual bool IsAllInitConnectedToGoal(void)
    {
      for (int r = 0 ; r < m_nrRobot; r++)
      {
        if (m_disjointSet.Same(dynamic_cast<PrmRegion*>(m_regions[m_ridInits[r]])->m_dset,
              dynamic_cast<PrmRegion*>(m_regions[m_ridGoals[r]])->m_dset) == false)
              {
                return false;
              }
      }
      return true;

    }

    virtual bool IsInitConnectedToGoal(int rid)
    {
      if (m_disjointSet.Same(dynamic_cast<PrmRegion*>(m_regions[m_ridInits[rid]])->m_dset,
      dynamic_cast<PrmRegion*>(m_regions[m_ridGoals[rid]])->m_dset) == false)
      {
        return false;
      }
      return true;
    }

    struct TriRegion
    {
      double          *m_cfg;
      Polygon2D        m_shape;
      std::vector<int> m_cellsInside;
      std::vector<int> m_cellsIntersect;
      double m_color;

    };

    struct PrmRegion : public Region
    {
      PrmRegion(void) : Region()
      {
      }

      virtual ~PrmRegion(void)
      {
        if(m_dset)
        delete m_dset;
      }

      DisjointSet::Elem  *m_dset;
      std::vector<int>    m_attempts;
    };

    enum Method
    {
      METHOD_STANDARD = 0,
      METHOD_OPTIMAL  = 1,
      METHOD_INCREMENTAL = 2
    };


    MPTriAbstraction *m_triAbs;
    double       m_step;
    int          m_ridConnect;
    DisjointSet  m_disjointSet;
    int          m_method;
    int          m_nrSamples;
    int          m_nrNeighs;
    int          m_nrTry;
    double       m_factor;
    double m_minconnectEdgeDist;
    double m_maxconnectEdgeDist;
    
    struct tempEdge
    {
      double m_dist;
      int    m_rid1;
      int    m_rid2;
    };

    std::vector<tempEdge> m_minTempEdges;


    virtual int GenerateSamples();

    virtual int GenerateEdges(const int nneighs);

    virtual bool GenerateEdge(const int rid1, const int rid2);

    virtual void FixEdges();

    virtual void DrawRegion(const int rid);
    virtual void DrawRegions(void);

    virtual void GenerateSampleAtRegion(const int region);

    std::vector<int> m_waitlist;

  protected:
    virtual void AddNewRegion(const double pos[]);

    static double DistFn(const int rid1,
      const int rid2,
      MPPrmAbstraction * const aprm);

      ProximityGNAT<int, MPPrmAbstraction*> m_proximity;
      double                               *m_auxCfg;
    };
  }

  #endif
