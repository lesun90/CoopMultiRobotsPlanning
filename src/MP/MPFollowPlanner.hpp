#ifndef MP_FOLLOW_HPP_
#define MP_FOLLOW_HPP_

#include "MP/MPSimulator.hpp"

namespace MP
{
  class MPFollowPlanner
  {
  public:
    MPFollowPlanner(void);
    virtual ~MPFollowPlanner(void)
    {

    }

    struct Group
    {
      Group(void) : m_id(Constants::ID_UNDEFINED),m_weight(0.0)
      {
      }

      int              m_id;
      double           m_weight;
      std::vector<int> m_vids;
    };

    struct Vertex
    {
      Vertex(void)
      {
        m_parent       = Constants::ID_UNDEFINED;
        m_cfg          = NULL;
        m_s            = NULL;
        m_nextWaypt    = Constants::ID_UNDEFINED;
      }

      virtual ~Vertex(void)
      {
      }

      int       m_parent;
      MPState  *m_s;
      double   *m_cfg;
      int       m_nextWaypt;
    };

    virtual void SetSimulator(MPSimulator *sim)
    {
      m_sim = sim;
    }

    virtual void Run(const int nrIters);
    virtual void Initialize(void);
    virtual int AddVertex(Vertex * const v);
    virtual void ExtendFrom(const int vid, const double target[]);
    virtual Group* SelectGroup(void);
    virtual void SelectTarget(Group *g, double target[]);
  	virtual int SelectVertex(const Group * g, const double target[]);

    virtual bool IsInside(double wpt1[], double wpt2[],const double p[])
    {
      double        pmin[2];
      return DistSquaredPointSegment2D(p, wpt1, wpt2, pmin) <= m_radius * m_radius;
    }

    virtual double Weight(const int i) const
    {
      return pow(m_wbase, ((double) i) / (GetNrWaypts() - 1));
    }

    virtual int GetNrWaypts(void) const
    {
      return m_waypts.size();
    }

    virtual void Draw(void);

    MPSimulator          *m_sim;
  	std::vector<Vertex*>  m_vertices;
  	int                   m_vidSolved;
  	int                   m_extendMinNrSteps;
  	int                   m_extendMaxNrSteps;
  	double                m_probSelectNearestVertex;
  	UseMap(int, Group*)   m_groups;
  	double                m_dsel;
    std::vector<double*>  m_waypts;
    double                m_tolReach;
    double                m_radius;
    double                m_wbase;

  };
}

#endif
