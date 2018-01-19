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
      Group(void)
      {
        m_id = Constants::ID_UNDEFINED;
        m_weight = 0.0;
        m_dist2goal = 0.0;
      }

      int              m_id;
      double           m_weight;
      std::vector<int> m_vids;
      double           m_dist2goal;
    };

    struct WayPoint
    {
      double* m_cfg;
      double  m_radius;
      double  m_tolReach;
    };

    struct Vertex
    {
      Vertex(void)
      {
        m_parent       = Constants::ID_UNDEFINED;
        m_cfg          = NULL;
        m_s            = NULL;
        m_nextWaypt    = Constants::ID_UNDEFINED;
        m_timeid       = Constants::ID_UNDEFINED;
        m_id           = Constants::ID_UNDEFINED;
      }

      virtual ~Vertex(void)
      {
      }

      int       m_parent;
      MPState  *m_s;
      double   *m_cfg;
      int       m_nextWaypt;
      int       m_timeid;
      int       m_id;
    };

    virtual void SetSimulator(MPSimulator *sim)
    {
      m_sim = sim;
    }

    virtual void Run();
    virtual void Run(const int nrIters);

    virtual void Initialize(void);
    virtual int AddVertex(Vertex * const v);
    virtual void ExtendFrom(const int vid, const double target[]);
    virtual Group* SelectGroup(void);
    virtual Group* SelectBestGroup(void);

    virtual void SelectTarget(Group *g, double target[]);
  	virtual int SelectVertex(const Group * g, const double target[]);

    virtual bool IsInside(int wpt1, int wpt2,const double p[]);

    virtual double Weight(const int i) const
    {
      if (GetNrWaypts() <= 1)
      {
        return 1;
      }
      return pow(m_wbase, ((double) i) / (GetNrWaypts() - 1));
    }

    virtual int GetNrWaypts(void) const
    {
      return m_waypts.size();
    }

    virtual int GetLastWaypt(int k) const
    {
      return k>=1 ? k-1 : 0;
    }

    virtual double DistFromWpToGoal(int k)
    {
      double dist2goal = 0.0;
      for (int i = k ; i < GetNrWaypts()-1;i++)
      {
        dist2goal += Algebra2D::PointDist(m_waypts[i].m_cfg,m_waypts[i+1].m_cfg);
      }
      return dist2goal;
    }

    virtual void Draw(void);

    virtual bool CheckWithReserveTable(Vertex* v);
    virtual bool IsSolved(void);

    virtual void GetPath();
    virtual bool IsReachedWaypt(double *cfg, int k)
    {
      return Algebra2D::PointDist(cfg,m_waypts[k].m_cfg) < m_waypts[k].m_tolReach;
    }

    virtual bool IsReachedGoal(double *cfg);

    virtual void Clear()
    {
      m_vidSolved = Constants::ID_UNDEFINED;
      m_nrIters = 0;
      m_cfgPath.clear();
      m_path.clear();
      m_statePath.clear();
      m_vertices.clear();
    }



    MPSimulator          *m_sim;
  	std::vector<Vertex*>  m_vertices;
  	int                   m_vidSolved;
  	int                   m_extendMinNrSteps;
  	int                   m_extendMaxNrSteps;
  	double                m_probSelectNearestVertex;
  	UseMap(int, Group*)   m_groups;
  	double                m_dsel;
    std::vector<WayPoint> m_waypts;
    double                m_tolReach;
    double                m_radius;
    double                m_wbase;
    int                   m_nrIters;
    int                   m_maxiters;

    std::vector<double*>   m_cfgPath;
    std::vector<int>       m_path;
    std::vector<MPState*>  m_statePath;
    std::vector<std::vector<double*>> m_reserveTable;
    int m_maxTime;
  };
}

#endif
