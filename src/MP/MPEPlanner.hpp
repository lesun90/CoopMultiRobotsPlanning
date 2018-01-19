#ifndef MP_EPLANNER_HPP_
#define MP_EPLANNER_HPP_

#include "MP/MPTreePlanner.hpp"
#include "Utils/Map.hpp"

namespace MP
{
  class MPEPlanner : public MPTreePlanner
  {
  public:
    MPEPlanner(void);

    virtual ~MPEPlanner(void);

    virtual void Run(const int nrIters);

    double m_probSplit;
    double m_discountSelect;
    double m_hexp;
    double m_wexp;
    double m_cutoff;

    struct ARData
    {
      enum Type
      {
        REGION  = 0,
        POINT  = 1
      };

      ARData(void)
      {
        m_gCostSum = m_gCostTotalWeight = 0;
        m_weight   = 0;
        m_level    = 1;
      }

      virtual ~ARData(void)
      {
      }

      std::vector<int> m_vids;
      double           m_gCostSum;
      double           m_gCostTotalWeight;
      double           m_weight;
      int              m_level;


    };

    virtual int     AddVertex(Vertex * const v);
    virtual void    Initialize(void);

    virtual ARData* SelectARData(int * const key);
    virtual int     SelectVertexFromARData(const ARData * const data);
    virtual void    CompleteARData(const int key, ARData * const data);
    virtual void    AddVertexToARData(const int vid);
    virtual void    SelectTarget(const int rid);

    UseMap(int, ARData*)     m_map;
    UseMap(int, ARData*)     m_split;
    double                   m_wmin;
    int                      m_iter;
    int                       m_robotid;



  };
}

#endif
