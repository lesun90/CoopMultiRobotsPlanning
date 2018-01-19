#include "Utils/Polygon2D.hpp"
#include "External/ShewchukTriangle.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/PrintMsg.hpp"
#include <cstdio>

namespace MP
{
    int Polygon2D::GetNrTriangles(void)
    {
	return GetTriangleIndices()->size() / 3;
    }

    const std::vector<int>* Polygon2D::GetTriangleIndices(void)
    {
	if(m_triRecompute)
	{
	    m_triRecompute = false;
	    m_triIndices.clear();		
	    TriangulatePolygonWithNoHoles2D(false, -1, -1,
					    m_vertices.size() / 2, &m_vertices[0],
					    NULL, &m_triIndices, NULL);
	}	
	return &m_triIndices;
    }

    void Polygon2D::GetTriangleVertices(const int i, double tri[6])
    {
	GetTriangleIndices();
	tri[0] = m_vertices[2 * m_triIndices[3 * i    ]    ];
	tri[1] = m_vertices[2 * m_triIndices[3 * i    ] + 1];
	tri[2] = m_vertices[2 * m_triIndices[3 * i + 1]    ];
	tri[3] = m_vertices[2 * m_triIndices[3 * i + 1] + 1];
	tri[4] = m_vertices[2 * m_triIndices[3 * i + 2]    ];
	tri[5] = m_vertices[2 * m_triIndices[3 * i + 2] + 1];	
    }
    

    const std::vector<double>* Polygon2D::GetTriangleAreas(void)
    {
	if(m_areasRecompute)
	{
	    m_areasRecompute = false;
	    m_triAreas.clear();
	    const int n = GetNrTriangles();
	    double tri[6];
	    double area = 0;
	    
	    m_triAreas.resize(n);
	    m_area = 0;
	    for(int i = 0; i < n; ++i)
	    {
		GetTriangleVertices(i, tri);
		m_triAreas[i] = fabs(SignedAreaPolygon2D(3, tri));
		m_area += m_triAreas[i];

		if(m_triAreas[i] > area)
		{
		    m_triLargestArea = i;
		    area = m_triAreas[i];
		}		
	    }
	}

	return &m_triAreas;
    }
    
    double Polygon2D::GetArea(void)
    {
	GetTriangleAreas();
	return m_area;
    }

    const double* Polygon2D::GetCentroid(void)
    {
	if(m_centroidRecompute)
	{
	    m_centroidRecompute = false;
	    double x = 0, y = 0, z = 0;
	    int n = m_vertices.size() / 2;
	    for(int i = 0; i < n; ++i)
	    {
		x += m_vertices[2 * i];
		y += m_vertices[2 * i + 1];
		if(i < (int) m_heights.size())
	    	    z += m_heights[i];
	    }
	    m_centroid[0] = x / n;
	    m_centroid[1] = y / n;
	    m_centroid[2] = z / n;
	}
	
	return m_centroid;
	
    }
    
    

    double Polygon2D::GetSomePointInside(double p[2], const bool halso)
    {
	if(IsConvex())
	{
	    const double *c = GetCentroid();
	    p[0] = c[0];
	    p[1] = c[1];
	    return c[2];
	}
	else
	{
	    double tri[6];	
	    GetTriangleVertices(0, tri);
	    const double a1 = 0.35;
	    const double a2 = 0.45;
	    
	    p[0] = (tri[0] + tri[2] + tri[4]) / 3.0;
	    p[1] = (tri[1] + tri[3] + tri[5]) / 3.0;

	    if(halso)
		return GetHeight(p, 0);
	}
	
    }
    
    double Polygon2D::GetHeight(const double p[2], const int ti)
    {
	const int    i1 = m_triIndices[3 * ti];
	const int    i2 = m_triIndices[3 * ti + 1];
	const int    i3 = m_triIndices[3 * ti + 2];
	const double h1 = i1 < m_heights.size() ? m_heights[i1] : 0;
	const double h2 = i2 < m_heights.size() ? m_heights[i2] : 0;
	const double h3 = i2 < m_heights.size() ? m_heights[i3] : 0;
	const double d1 = Algebra2D::PointDist(&(m_vertices[2 * i1]), p);
 	const double d2 = Algebra2D::PointDist(&(m_vertices[2 * i2]), p);
 	const double d3 = Algebra2D::PointDist(&(m_vertices[2 * i3]), p);
	
	return (h1 * d1 + h2 * d2 + h3 * d3) / (d1 + d2 + d3);
	
    }
    
    double Polygon2D::SampleRandomPointInside(double p[2], const bool halso)
    {
	const int t = SelectTriangleBasedOnArea();
	SampleRandomPointInsideTriangle2D(&m_vertices[2 * m_triIndices[3 * t]],
					  &m_vertices[2 * m_triIndices[3 * t + 1]],
					  &m_vertices[2 * m_triIndices[3 * t + 2]], p);
	if(halso)
	    return GetHeight(p, t);
    }

    int Polygon2D::SelectTriangleBasedOnArea(void)
    {
	const int    n = GetNrTriangles();
	const double r = RandomUniformReal(0, GetArea());
	double       w = 0;
	
	GetTriangleAreas();
	for(int i = 0; i < n; ++i)
	{
	    w += m_triAreas[i];
	    if(w >= r)
		return i;
	}
	return n - 1;
    }
    

    void Polygon2D::OccupiedGridCells(const Grid * const       grid, 
				      std::vector<int> * const cellsInside,
				      std::vector<int> * const cellsIntersect)
    {		
	const double *bbox = GetBoundingBox();
	const int     n    = m_vertices.size() / 2;
	const double *poly = &m_vertices[0];
	const bool    checkIntersection = false;
	
	int coord_min[3]={0,0,0};
	int coord_max[3] ={0,0,0};
	int coords[3] = {0,0,0};
	double min[3], max[3];
	
	cellsInside->clear();
	cellsIntersect->clear();		
	grid->GetCoords(&bbox[0], coord_min);
	grid->GetCoords(&bbox[2], coord_max);       
	for(int x = coord_min[0]; x <= coord_max[0]; ++x)
	{
	    coords[0] = x;		    
	    for(int y = coord_min[1]; y <= coord_max[1]; ++y)
	    {
		coords[1] = y;
		
		grid->GetCellFromCoords(coords, min, max);
		double box[8];
		AABoxAsPolygon2D(min, max, box);
		
		if(IntersectPolygons2D(4, box, n, poly))
		    cellsIntersect->push_back(grid->GetCellIdFromCoords(coords));
		else if(IsPolygonInsidePolygon2D(4, box, n, poly, checkIntersection))
		    cellsInside->push_back(grid->GetCellIdFromCoords(coords));
		else if(IsPolygonInsideConvexPolygon2D(n, poly, 4, box))
		    cellsIntersect->push_back(grid->GetCellIdFromCoords(coords));
			
	    }
	    
	}
    }   

    bool Polygon2D::CollisionPolygon(Polygon2D * const poly)
    {
	const double *bbox1 = GetBoundingBox();
	const double *bbox2 = poly->GetBoundingBox();
	
	if(CollisionAABoxes2D(bbox1, &bbox1[2],
			      bbox2, &bbox2[2]) == false)
	    return false;
	
	
	const bool c1 = IsConvex();
	const bool c2 = poly->IsConvex();
	
	if(c1 && c2)
	    return CollisionConvexPolygons2D(m_vertices.size() / 2, &m_vertices[0],
					     poly->m_vertices.size() / 2, &(poly->m_vertices[0]));
	else if(c1 && !c2)
	    return CollisionPolygonConvexPolygon2D(poly->m_vertices.size() / 2, &(poly->m_vertices[0]),
						   m_vertices.size() / 2, &m_vertices[0]);
	else if(!c1 && c2)
	    return CollisionPolygonConvexPolygon2D(m_vertices.size() / 2, &m_vertices[0],
						   poly->m_vertices.size() / 2, &(poly->m_vertices[0]));
	else
	    return CollisionPolygons2D(m_vertices.size() / 2, &m_vertices[0],
				       poly->m_vertices.size() / 2, &(poly->m_vertices[0]));
    }
    
    
    void Polygon2D::Draw(void)
    {
	const double *heights = (GDrawIs2D() == false && (m_vertices.size() == m_heights.size() * 2)) ? &m_heights[0] : NULL;
	
	if(IsConvex())
	    GDrawConvexPolygon2D(m_vertices.size() / 2, &m_vertices[0], heights);
	else
	{
	    GetTriangleIndices();
	    GDrawPolygon2D(m_vertices.size() / 2, &m_vertices[0],
			   m_triIndices.size() / 3, &m_triIndices[0], heights);
	}
    }

    void Polygon2D::Print(FILE * out) const
    {
	int n = m_vertices.size() / 2;
	
	fprintf(out, "%d\n", n);
	for(int i = 0; i < 2 * n; ++i)
	    fprintf(out, "%f ", m_vertices[i]);
	fprintf(out, "\n");
	n = m_heights.size();
	fprintf(out, "%d ", n);
	for(int i = 0; i < n; ++i)
	    fprintf(out, "%f ", m_heights[i]);
	fprintf(out, "\n\n");
    }

    void Polygon2D::Read(FILE * in)
    {
	int n;
	if(fscanf(in, "%d", &n) != 1)
	    OnInputError(printf("expecting number of vertices\n"));
	
	m_vertices.resize(2 * n);
	for(int i = 0; i < 2 * n; ++i)
	{
	    if(fscanf(in, "%lf", &m_vertices[i]) != 1)
		OnInputError(printf("expecting %d-th value for polygon with %d vertices\n", i, n));
	}
	
	if(fscanf(in, "%d", &n) != 1)
	    OnInputError(printf("expecting number of height values (it could be 0)\n"));
	
	m_heights.resize(n);
	for(int i = 0; i < n; ++i)
	{
	    if(fscanf(in, "%lf", &m_heights[i]) != 1)
		OnInputError(printf("expecting %d-th height value out of %d\n", i, n));
	}
	MakeCCW();
    }
    
    
    void ReadPolygons2D(FILE * in, std::vector<Polygon2D*> * const polys)
    {
	Polygon2D *poly;
	int        n;
	
	if(fscanf(in, "%d", &n) != 1)
	    OnInputError(printf("expecting number of polygons\n"));
	
	for(int i = 0; i < n; ++i)
	{
	    poly = new Polygon2D();
	    poly->Read(in);
	    polys->push_back(poly);
	}
    }
    
}	



