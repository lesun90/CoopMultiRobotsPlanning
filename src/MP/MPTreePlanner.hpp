#ifndef MP_TREE_PLANNER_HPP_
#define MP_TREE_PLANNER_HPP_

#include "MP/MPPlanner.hpp"

namespace MP
{
  class MPTreePlanner : public MPPlanner
  {
  public:
    MPTreePlanner(void);

    virtual ~MPTreePlanner(void);

    virtual int GetSolved(void) const
    {
      return m_vidSolved;
    }

    virtual bool IsSolved(void)
    {
      return m_vidSolved>0;
    }

    virtual void GetReversePath(const int vid, std::vector<int> * const rpath) const;

    virtual double PathCost(std::vector<int> * const path) const;
    virtual MPState* GetState(const int i)
  	{
  	    return m_vertices[i]->m_s;
  	}
    virtual void DrawVertices() const;
    virtual void GetCfgPath(void);

    double  m_dtol;
    int     m_maxNrSteps;
    int     m_minNrSteps;
    double  m_probSteer;
    double  m_goalBias;

    struct Vertex
    {
      Vertex(void)
      {
        m_parent = Constants::ID_UNDEFINED;
        m_gCost  = 0;
        m_cfg    = NULL;
        m_s      = NULL;
        m_rid    = Constants::ID_UNDEFINED;
        m_vid    = Constants::ID_UNDEFINED;
        m_timeid = 0;
      }

      virtual ~Vertex(void)
      {
        // if(m_cfg)
        // delete m_cfg;
        // if(m_s)
        // delete[] m_s;
      }
      int       m_parent;
      int       m_vid;
      int       m_rid;
      MPState  *m_s;
      double   *m_cfg;
      double    m_gCost;
      int m_from;
      int m_to;
      int m_timeid;
    };

    virtual int GetNrVertices(void) const
    {
      return m_vertices.size();
    }

    virtual Vertex* NewVertex(void) const
    {
      return new Vertex();
    }

    virtual void Initialize(void);

    virtual int ExtendFrom(const int vid, const double target[]);

    virtual int AddVertex(Vertex * const v);

    std::vector<Vertex*> m_vertices;
    int                  m_vidSolved;
    double              *m_cfgTarget;

    };
  }

  #endif
