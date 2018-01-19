#ifndef MP_POLYGON2D_HPP_
#define MP_POLYGON2D_HPP_

#include "Utils/Grid.hpp"
#include "Utils/Geometry.hpp"
#include "Utils/PseudoRandom.hpp"
#include <vector>
#include <cstdio>
#include <cstdlib>

namespace MP
{
    class Polygon2D
    {
    public:
	Polygon2D(void)
	{
	    m_bboxRecompute = true;
	    m_triRecompute  = true;
	    m_convexityRecompute = true;
	    m_areasRecompute = true;
	    m_centroidRecompute = true;
	    m_area = 0.0;
	    m_triLargestArea = 0;
	}

	~Polygon2D(void)
	{
	}

	void Rectangle(const double minx,
		       const double miny,
		       const double maxx,
		       const double maxy)
	{
	    m_vertices.resize(8);
	    m_vertices[0] = minx; m_vertices[1] = miny;
	    m_vertices[2] = maxx; m_vertices[3] = miny;
	    m_vertices[4] = maxx; m_vertices[5] = maxy;
	    m_vertices[6] = minx; m_vertices[7] = maxy;
	    OnShapeChange();
	}

  void MakePolygonAt(const double s[], const double size)
{
  for (double a=0.0; a<2.0*M_PI;)
  {
    double x=s[0]+(size*cos(a));
    double y=s[1]+(size*sin(a));
    a+=(20.0+(40.0*RandomUniformReal()))*M_PI/180.0;
    m_vertices.push_back(x);
    m_vertices.push_back(y);
  }
  OnShapeChange();
}


	void MakeCCW(void)
	{
	    MakePolygonCCW2D(m_vertices.size() / 2, &m_vertices[0]);
	    OnShapeChange();
	}

	const double* GetBoundingBox(void)
	{
	    if(m_bboxRecompute)
	    {
		m_bboxRecompute = false;
		BoundingBoxPolygon2D(m_vertices.size() / 2, &m_vertices[0], &m_bbox[0], &m_bbox[2]);
	    }
	    return m_bbox;
	}

	bool IsConvex(void)
	{
	    if(m_convexityRecompute)
	    {
		m_convexityRecompute = false;
		m_isConvex = IsPolygonConvex2D(m_vertices.size() / 2, &m_vertices[0]);
	    }
	    return m_isConvex;
	}

	int GetNrTriangles(void);

	const std::vector<int>* GetTriangleIndices(void);

	void GetTriangleVertices(const int i, double tri[6]);

	const std::vector<double>* GetTriangleAreas(void);

	double GetArea(void);

	const double* GetCentroid(void);


	int SelectTriangleBasedOnArea(void);

	int GetTriangleWithLargestArea(void)
	{
	    GetTriangleAreas();
	    return m_triLargestArea;
	}

	double GetSomePointInside(double p[2], const bool halso = false);

	double SampleRandomPointInside(double p[2], const bool halso = false);

	void OccupiedGridCells(const Grid * const       grid,
			       std::vector<int> * const cellsInside,
			       std::vector<int> * const cellsIntersect);


	void OnShapeChange(void)
	{
	    m_bboxRecompute = true;
	    m_triRecompute  = true;
	    m_convexityRecompute = true;
	    m_areasRecompute = true;
	    m_centroidRecompute = true;
	}

	void OnPlacementChange(void)
	{
	    m_bboxRecompute = true;
	}

	void Draw(void);


	std::vector<double> m_vertices;
	std::vector<double> m_heights;


	double DistSquaredPoint(const double p[2], double pmin[2]) const
	{
	    return DistSquaredPointPolygon2D(p, m_vertices.size() / 2, &m_vertices[0], pmin);
	}

	double DistSquaredSegment(const double p1[2],
				  const double p2[2],
				  double       pmin1[2],
				  double       pmin2[2]) const
	{
	    return DistSquaredSegmentPolygon2D(p1, p2, m_vertices.size() / 2, &m_vertices[0], pmin1, pmin2);
	}

	double DistSquaredPolygon(const Polygon2D * const poly, double pmin1[2], double pmin2[2]) const
	{
	    return DistSquaredPolygons2D(m_vertices.size() / 2, &m_vertices[0],
					 poly->m_vertices.size() / 2, &(poly->m_vertices[0]), pmin1, pmin2);
	}

	bool IntersectSegment(const double p0[2],
			      const double p1[2]) const
	{
	    return IntersectSegmentPolygon2D(p0, p1, m_vertices.size() / 2, &m_vertices[0]);
	}

	bool IntersectPolygon(const Polygon2D * const poly) const
	{
	    return IntersectPolygons2D(m_vertices.size() / 2, &m_vertices[0],
				       poly->m_vertices.size() / 2, &(poly->m_vertices[0]));
	}

	bool IsPointInside(const double p[])
	{
	    if(IsConvex())
		return IsPointInsideConvexPolygon2D(p, m_vertices.size() / 2, &m_vertices[0]);
	    else
		return IsPointInsidePolygon2D(p, m_vertices.size() / 2, &m_vertices[0]);
	}

	bool IsInsideAABox2D(const double min[2], const double max[2]) const
	{
	    return IsPolygonInsideAABox2D(m_vertices.size() / 2, &m_vertices[0], min, max);
	}

	bool IsInsidePolygon(Polygon2D * const poly)
	{
	    if(poly->IsConvex())
		return IsPolygonInsideConvexPolygon2D(m_vertices.size() / 2, &m_vertices[0],
						      poly->m_vertices.size() / 2, &(poly->m_vertices[0]));
	    else
		return IsPolygonInsidePolygon2D(m_vertices.size() / 2, &m_vertices[0],
						poly->m_vertices.size() / 2, &(poly->m_vertices[0]));

	}

	bool CollisionSegment(const double p0[2], const double p1[2])
	{
	    if(IsConvex())
		return CollisionSegmentConvexPolygon2D(p0, p1, m_vertices.size() / 2, &m_vertices[0]);
	    else
		return CollisionSegmentPolygon2D(p0, p1, m_vertices.size() / 2, &m_vertices[0]);
	}

	bool CollisionPolygon(Polygon2D * const poly);

	void Print(FILE *out) const;
	void Read(FILE *in);



    protected:
	double              m_bbox[4];
	bool                m_bboxRecompute;
	std::vector<int>    m_triIndices;
	bool                m_triRecompute;
	std::vector<double> m_triAreas;
	double              m_area;
	int                 m_triLargestArea;
	bool                m_areasRecompute;
	bool                m_convexityRecompute;
	bool                m_centroidRecompute;
	bool                m_isConvex;
	double              m_centroid[3];

	double GetHeight(const double p[2], const int ti);



    };

    void ReadPolygons2D(FILE * in, std::vector<Polygon2D*> * const polys);
}

#endif
