#ifndef MP_SCENE_HPP_
#define MP_SCENE_HPP_

#include "Utils/Grid.hpp"
#include "Utils/Polygon2D.hpp"
#include "Utils/Algebra2D.hpp"
#include "External/PQP/PQPTriMesh.hpp"
#include <vector>
#include "Utils/Stats.hpp"

namespace MP
{
  class MPScene
  {
  public:
    MPScene(void);
    virtual ~MPScene(void);

    struct Object
    {
      double     m_cfg[3];
      Polygon2D  m_poly;
      PQPTriMesh m_tmesh;
    };

    struct Obstacles
    {
      std::vector<Polygon2D*> m_polys;
      PQPTriMesh              m_tmesh;
    };

    struct robotCfg
    {
      double m_cfg[3];
    };

    Obstacles          	   m_obstacles;
    std::vector<robotCfg*> m_robotInit;
    std::vector<Object*>   m_goals;
    Grid                   m_grid;
    int                    m_nrRobot;
    PQPTriMesh            *m_ground;
    PQPTriMesh            *m_outline;

    double m_maxGroundHeight;

    virtual void SetupFromFile(FILE * const in);
    virtual void SetupFromFile(FILE * const in,FILE * const query);
    virtual void SetupScene(FILE * const in);
    virtual void SetupQuery(FILE * const in);

    virtual void CompleteSetup(void)
    {
      ExtrudeObstacles();
    }

    virtual void DrawObstacles(void);
    virtual void DrawScene(void);
    virtual void DrawGround(void);

    virtual void DrawGoals(void);
    virtual void Draw(void);

    virtual Grid* GetGrid(void)
    {
      return &m_grid;
    }

    double eye[3];
    double center[3];
    double right[3];
    double forward[3];

  protected:
    virtual void ExtrudeObstacles(void);

  };
}

#endif
