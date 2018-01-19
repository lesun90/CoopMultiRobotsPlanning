#ifndef MP_PROXIMITY_PLANNER_HPP_
#define MP_PROXIMITY_PLANNER_HPP_

#include "MP/MPTreePlanner.hpp"
#include "Utils/ProximityGNAT.hpp"

namespace MP
{
  class MPProximityPlanner : public MPTreePlanner
  {
  public:
    MPProximityPlanner(void) : MPTreePlanner()
    {
      m_proximity.m_distFn     = DistFn;
      m_proximity.m_distFnData = this;
    }

    virtual ~MPProximityPlanner(void)
    {
      m_proximity.Clear();
    }

    virtual int AddVertex(Vertex * const v)
    {
      const int vid = MPTreePlanner::AddVertex(v);

      if(vid >= 0)
      m_proximity.AddKey(vid);
      return vid;
    }

  protected:
    static double DistFn(const int vid1,
      const int vid2,
      MPProximityPlanner * const pp)
      {
        const double *cfg1  = vid1 >= 0 ? pp->m_vertices[vid1]->m_cfg : pp->m_cfgTarget;
        const double *cfg2  = vid2 >= 0 ? pp->m_vertices[vid2]->m_cfg : pp->m_cfgTarget;

        return pp->m_sim->DistanceCfg(cfg1, cfg2);
      }


      ProximityGNAT<int, MPProximityPlanner*> m_proximity;
    };
  }

  #endif
