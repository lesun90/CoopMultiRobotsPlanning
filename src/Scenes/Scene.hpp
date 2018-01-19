#ifndef MP_SCENE_HPP_
#define MP_SCENE_HPP_

#include "Utils/TriMesh.hpp"
#include "Utils/Polygon2D.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/GDraw.hpp"

namespace MP
{
    class Scene
    {
    public:
	Scene(void)
	{
	}

	virtual ~Scene(void)
	{
	    DeleteItems<Polygon2D*>(&m_polys);
	}

	virtual void Extrude(const int    start,
			     const int    end,
			     const double zmin,
			     const double zmax,
			     const bool   useGround = false);

  void DrawRoom(void)
  {
    char      msg[100];
    GMaterial gmat;

    GDraw2D();
    GDrawColor(1.0, 0.8, 0.8);
    GDrawPushTransformation();
    GDrawMultTrans(0, 0, 0.01);
    GDrawAABox2D(m_grid.GetMin(), m_grid.GetMax());
    GDrawPopTransformation();
  }

  void DrawObstacles(void)
  {
    GDraw3D();
    GMaterial gmat;
    gmat.SetTurquoise();
    GDrawMaterial(&gmat);
    m_tmesh.Draw();
  }

	Grid                     m_grid;
	std::vector<Polygon2D *> m_polys;
	TriMesh                  m_tmesh;
	TriMesh                  m_ground;
	TriMesh                  m_draw;


    };

    class AddToScene
    {
    public:
	AddToScene(void)
	{
	    m_zmin = 0;
	    m_zmax = 1.0;

	}

	virtual ~AddToScene(void)
	{
	}

	virtual void Add(Scene * const scene)
	{
	}

	double                   m_zmin;
	double                   m_zmax;
    };


    class AddToSceneRandomPolygons : public AddToScene
    {
    public:
	AddToSceneRandomPolygons(void) : AddToScene(),
					 m_perc(0.1),
					 m_dsep(1.0),
					 m_rmin(0.5),
					 m_rmax(2.0),
					 m_minNrVertices(3),
					 m_maxNrVertices(6),
					 m_maxNrPolys(-1),
					 m_startCriterion2(-1),
					 m_dsepCriterion2(-1)
	{
	}

	virtual ~AddToSceneRandomPolygons(void)
	{
	}

	virtual void Add(Scene * const scene);


	double m_perc;
	double m_dsep;
	double m_rmin;
	double m_rmax;
	int    m_minNrVertices;
	int    m_maxNrVertices;
	int    m_maxNrPolys;
	int    m_startCriterion2;
	double m_dsepCriterion2;



    };


    class AddToScenePolygonOnALine : public AddToScene
    {
    public:
      AddToScenePolygonOnALine(void) : AddToScene(),
      m_perc(0.1),
      m_dsep(1.0),
      m_rmin(0.5),
      m_rmax(2.0),
      m_minNrVertices(3),
      m_maxNrVertices(6),
      m_maxNrPolys(-1),
      m_startCriterion2(-1),
      m_dsepCriterion2(-1)
      {
      }

      virtual ~AddToScenePolygonOnALine(void)
      {
      }

      virtual void Add(Scene * const scene);

      double startx;
      double starty;
      double endx;
      double endy;
      double m_maxspace;
      double m_perc;
    	double m_dsep;
    	double m_rmin;
    	double m_rmax;
    	int    m_minNrVertices;
    	int    m_maxNrVertices;
    	int    m_maxNrPolys;
    	int    m_startCriterion2;
    	double m_dsepCriterion2;
    };

    class AddToSceneMaze : public AddToScene
    {
    public:
	AddToSceneMaze(void) : AddToScene(),
			       m_keepPerc(0.8),
			       m_width(1.0)
	{
	}

	virtual ~AddToSceneMaze(void)
	{
	}

	virtual void Add(Scene * const scene);

	virtual void AddFencesWhenEmpty(Scene * const scene,
					const double    prob,
					const int       minNrDims,
					const int       maxNrDims);


	virtual void AddSmallWallWhenEmpty(Scene * const scene,
					    const double prob);


	double m_keepPerc;
	double m_width;

    protected:


	struct Wall
	{
	    int m_cids[2];
	};

	virtual void GetWall(Scene * const scene, Wall * const wall, double min[2], double max[2]) const;

	std::vector<Wall> m_walls;
	std::vector<Wall> m_empty;
	int               m_nkeep;


    };

    class AddArc : public AddToScene
    {
    public:
      AddArc(void) : AddToScene(),
      m_thick(1.0),
      m_r(5.0),
      m_thetaMin(0),
      m_thetaMax(M_PI),
      m_nrRays(5),
      m_x(0.0),
      m_y(0.0)
      {
      }

      virtual ~AddArc(void)
      {
      }

      virtual void Add(Scene * const scene);

      double m_r;
      double m_x;
      double m_y;
      double m_thick;
    	int    m_nrRays;
      double m_thetaMin;
      double m_thetaMax;

    };

    class AddToSceneRadial : public AddToScene
    {
    public:
	AddToSceneRadial(void) : AddToScene(),
				 m_rmin(4),
				 m_rinc(4),
				 m_minGap(2),
				 m_maxGap(3),
				 m_thick(0.5),
				 m_nrRays(5),
				 m_nrSegs(4),
				 m_thetaMin(0 * Constants::DEG2RAD),
				 m_thetaMax(60 * Constants::DEG2RAD)
	{
	}

	virtual ~AddToSceneRadial(void)
	{
	}

	virtual void Add(Scene * const scene);

	double m_rmin;
	double m_rinc;
	double m_minGap;
	double m_maxGap;
	double m_thick;
	int    m_nrRays;
	int    m_nrSegs;
	double m_thetaMin;
	double m_thetaMax;
    };


    class AddToSceneRamp : public AddToScene
    {
    public:
	AddToSceneRamp(void) : AddToScene(),
			       m_width(1.0),
			       m_height(1.0),
			       m_left(1.0),
			       m_top(1.0),
			       m_right(1.0)
	{
	}

	virtual ~AddToSceneRamp(void)
	{
	}

	virtual void Add(Scene * const scene);

	double m_width;
	double m_height;
	double m_left;
	double m_top;
	double m_right;

    };

    class AddToSceneSpiral : public AddToScene
    {
    public:
	AddToSceneSpiral(void) : AddToScene(),
				 m_width(1),
				 m_height(1),
				 m_coeffx(1),
				 m_coeffy(1),
				 m_open(1),
				 m_angleStart(0),
				 m_angleInc(0.1),
				 m_zinc(0)
	{
	}

	virtual ~AddToSceneSpiral(void)
	{
	}

	virtual void Add(Scene * const scene);

	double m_width;
	double m_height;
	double m_coeffx;
	double m_coeffy;
	double m_open;
	double m_angleStart;
	double m_angleInc;
	double m_zinc;
    };

    class AddToSceneBumpyTiles : public AddToScene
    {
    public:
	AddToSceneBumpyTiles(void) : AddToScene(),
				     m_zmin(1),
				     m_zmax(2)
	{
	}

	virtual ~AddToSceneBumpyTiles(void)
	{
	}

	virtual void Add(Scene * const scene);

	double m_zmin;
	double m_zmax;
    };


    class AddToSceneHeightField : public AddToScene
    {
    public:
	AddToSceneHeightField(void) : AddToScene(),
				      m_fnEval(NULL),
				      m_zmin(1),
				      m_zmax(2)
	{
	}

	virtual ~AddToSceneHeightField(void)
	{
	}

	virtual void Add(Scene * const scene);

	static double EvalBumpy(HeightField * const hf,
				const int  i,
				const int  j);

	static double EvalRampX(HeightField * const hf,
				const int  i,
				const int  j);
	static double EvalRampY(HeightField * const hf,
				const int  i,
				const int  j);

	static double EvalRollingHills(HeightField * const hf,
				       const int  i,
				       const int  j);


	double (*m_fnEval) (HeightField * const hf, const int i, const int j);
	double m_zmin;
	double m_zmax;
    };



    class AddToSceneBumpyTerrain : public AddToScene
    {
    public:
	AddToSceneBumpyTerrain(void) : AddToScene(),
				       m_zmin(1),
				       m_zmax(2)
	{
	}

	virtual ~AddToSceneBumpyTerrain(void)
	{
	}

	virtual void Add(Scene * const scene);

	static double EvalBumpy(HeightField * const hf,
				const int  i,
				const int  j);

	double m_zmin;
	double m_zmax;
    };


    class AddToScenePotentialTerrain : public AddToScene
    {
    public:
	AddToScenePotentialTerrain(void) : AddToScene(),
					   m_nrPts(100),
					   m_zmin(1),
					   m_zmax(2)
	{
	}

	virtual ~AddToScenePotentialTerrain(void)
	{
	}

	virtual void Add(Scene * const scene);

	static double EvalPotential(HeightField * const hf,
				    const int  i,
				    const int  j);

	int    m_nrPts;
	double m_zmin;
	double m_zmax;

    protected:
	double m_dmin;
	double m_dmax;
	std::vector<double> m_pts;
	std::vector<double> m_peaks;
	double              m_hmin;


    };


}

#endif
