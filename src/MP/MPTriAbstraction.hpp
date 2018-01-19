#ifndef MP_TRI_ABSTRACTION_HPP_
#define MP_TRI_ABSTRACTION_HPP_

#include "MP/MPAbstraction.hpp"
#include "Utils/Polygon2D.hpp"
#include "Utils/Misc.hpp"

namespace MP
{
  class MPTriAbstraction : public MPAbstraction
  {
  public:

    struct TriRegion : public Region
    {
      TriRegion(void) : Region()
      {
      }

      virtual ~TriRegion(void)
      {
      }

      Polygon2D        m_shape;
      std::vector<int> m_cellsInside;
      std::vector<int> m_cellsIntersect;
    };

    MPTriAbstraction(void) : MPAbstraction(),
    m_useConformingDelaunay(false),
    m_angleConstraint(20),
    m_areaConstraint(100),
    m_minAreaToAdd(Constants::DECOMPOSITION_MIN_AREA_TO_ADD)
    {
    }

    virtual ~MPTriAbstraction(void)
    {
      DeleteItems<InsideIntersectRegions*>(&m_cellsToRegions);
    }

    virtual void HCosts(void);
    // virtual void HCosts(int rid);

    virtual void CompleteSetup(void);
    virtual int  LocateRegion(const double p[]);
    virtual bool IsInsideRegion(const int rid, const double p[])
    {
      return dynamic_cast<TriRegion*>(m_regions[rid])->m_shape.IsPointInside(p);
    }
    virtual bool SplitRegion(const int rid, std::vector<int> *new_rids);

    virtual void SamplePointInsideRegion(const int rid, double p[], const double dtol) const;

    virtual void DrawRegion(const int rid);
    virtual void DrawRegions(void);
    virtual void DrawTriangle(const int rid);
    virtual void DrawTriangles(void);
    virtual double GetRegionArea(const int rid)
    {
      return dynamic_cast<TriRegion*>(m_regions[rid])->m_shape.GetArea();
    }


    double m_areaConstraint;
    double m_angleConstraint;
    bool   m_useConformingDelaunay;
    double m_minAreaToAdd;

    struct InsideIntersectRegions
    {
      int              m_insideRegion;
      std::vector<int> m_intersectRegions;
    };

    std::vector< InsideIntersectRegions* > m_cellsToRegions;

    virtual void Triangulate(void);
    virtual void LocatorRemoveCells(const int rid);
    virtual void LocatorUpdateCells(const int rid);
    virtual void LocatorCompleteSetup(void);

    virtual void ConsTructWeight()
    {
      //construct weights
      int nedges = 0;
      for(int i = 0; i < m_regions.size(); ++i)
      {
        const int k = m_regions[i]->m_neighs.size();
        m_regions[i]->m_dists.resize(k);
        m_regions[i]->m_weights.resize(k);
        for(int j = 0; j < k; ++j)
        {
          m_regions[i]->m_dists[j] = Algebra2D::PointDist(m_regions[i]->m_cfg, m_regions[m_regions[i]->m_neighs[j]]->m_cfg);
          m_regions[i]->m_weights[j] = m_regions[i]->m_dists[j];
        }

        nedges += k;
      }
    }


  };
}

#endif
