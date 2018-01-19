#ifndef MP_GEOMETRY_HPP_
#define MP_GEOMETRY_HPP_

#include <vector>
#include <cstdio>
#include <algorithm>    // std::max
#include <cmath>

namespace MP
{
    static inline
    void TriangleAsPolygon2D(const double tA[2],
			     const double tB[2],
			     const double tC[2],
			     double       poly[6])
    {
	poly[0] = tA[0];
	poly[1] = tA[1];
	poly[2] = tB[0];
	poly[3] = tB[1];
	poly[4] = tC[0];
	poly[5] = tC[1];
    }

    static inline
    void AABoxAsPolygon2D(const double min[2],
			  const double max[2],
			  double       poly[8])
    {
	poly[0] = min[0];
	poly[1] = min[1];
	poly[2] = max[0];
	poly[3] = min[1];
	poly[4] = max[0];
	poly[5] = max[1];
	poly[6] = min[0];
	poly[7] = max[1];
    }

    static inline
    void QuadAsPolygon2D(const double qA[2],
			 const double qB[2],
			 const double qC[2],
			 const double qD[2],
			 double       poly[8])
    {
	poly[0] = qA[0];
	poly[1] = qA[1];
	poly[2] = qB[0];
	poly[3] = qB[1];
	poly[4] = qC[0];
	poly[5] = qC[1];
	poly[6] = qD[0];
	poly[7] = qD[1];
    }

    void CircleAsPolygon2D(const double cx,
			   const double cy,
			   const double r,
			   const int    n,
			   double       poly[]);


    double DistSquaredPointLine2D(const double p[2],
				  const double s0[2],
				  const double s1[2],
				  double       pmin[2]);

    double DistSquaredPointSegment2D(const double p[2],
				     const double s0[2],
				     const double s1[2],
				     double pmin[2]);

    double DistSquaredPointPolygon2D(const double p[2],
				     const int    n,
				     const double poly[],
				     double       pmin[2]);

    double DistSquaredSegments2D(const double p1[2],
				 const double p2[2],
				 const double p3[2],
				 const double p4[2],
				 double       pmin1[2],
				 double       pmin2[2]);

    double DistSegments2D(const double p1[2],
     				 const double p2[2],
     				 const double p3[2],
     				 const double p4[2]);

    double DistSquaredSegmentPolygon2D(const double p1[2],
				       const double p2[2],
				       const int    n,
				       const double poly[],
				       double       pmin1[2],
				       double       pmin2[2]);


    double DistSquaredPolygons2D(const int    n1,
				 const double poly1[],
				 const int    n2,
				 const double poly2[],
				 double       pmin1[2],
				 double       pmin2[2]);

    bool IntersectLines2D(const double x1, const double y1,
			  const double x2, const double y2,
			  const double x3, const double y3,
			  const double x4, const double y4,
			  double * const x,   double * const y,
			  double * const mua, double * const mub);


    static inline
    bool IntersectLineRay2D(const double x1, const double y1,
			    const double x2, const double y2,
			    const double x3, const double y3,
			    const double x4, const double y4,
			    double * const x, double * const y)
    {
	double mua, mub;

	return
	    IntersectLines2D(x1, y1, x2, y2, x3, y3, x4, y4, x, y, &mua, &mub) && mub >= 0;
    }

    static inline
    bool IntersectLineSegment2D(const double x1, const double y1,
				const double x2, const double y2,
				const double x3, const double y3,
				const double x4, const double y4,
				double * const x, double * const y)
    {
	double mua, mub;

	return
	    IntersectLines2D(x1, y1, x2, y2, x3, y3, x4, y4, x, y, &mua, &mub) && mub >= 0 && mub <= 1;
    }

    static inline
    bool IntersectRays2D(const double x1, const double y1,
			 const double x2, const double y2,
			 const double x3, const double y3,
			 const double x4, const double y4,
			 double * const x, double * const y)
    {
	double mua, mub;

	return
	    IntersectLines2D(x1, y1, x2, y2, x3, y3, x4, y4, x, y, &mua, &mub) &&
	    mua >= 0 && mub >= 0;
    }

    static inline
    bool IntersectRaySegment2D(const double x1, const double y1,
			       const double x2, const double y2,
			       const double x3, const double y3,
			       const double x4, const double y4,
			       double * const x, double * const y)
    {
	double mua, mub;

	return
	    IntersectLines2D(x1, y1, x2, y2, x3, y3, x4, y4, x, y, &mua, &mub) &&
	    mua >= 0 && mub >= 0 && mub <= 1;
    }

    static inline
    bool IntersectSegments2D(const double x1, const double y1,
			     const double x2, const double y2,
			     const double x3, const double y3,
			     const double x4, const double y4,
			     double * const x, double * const y)
    {
	double mua, mub;

	return
	    IntersectLines2D(x1, y1, x2, y2, x3, y3, x4, y4, x, y, &mua, &mub) &&
	    mua >= 0 && mua <= 1 && mub >= 0 && mub <= 1;
    }


    static inline
    bool IntersectSegments2D(const double p1[2], const double p2[2],
			     const double p3[2], const double p4[2],
			     double * const p)
    {
	return IntersectSegments2D(p1[0], p1[1], p2[0], p2[1],
				   p3[0], p3[1], p4[0], p4[1], &p[0], &p[1]);
    }

    bool IntersectSegments2D(const double x1, const double y1,
			     const double x2, const double y2,
			     const double x3, const double y3,
			     const double x4, const double y4);

    static inline
    bool IntersectSegments2D(const double p1[2], const double p2[2],
			     const double p3[2], const double p4[2])
    {
	return IntersectSegments2D(p1[0], p1[1], p2[0], p2[1],
				   p3[0], p3[1], p4[0], p4[1]);
    }


    bool IntersectSegmentPolygon2D(const double p0[2],
				   const double p1[2],
				   const int n,
				   const double poly[]);

    bool IntersectPolygons2D(const int n1,
			     const double poly1[],
			     const int n2,
			     const double poly2[]);

    int IntersectLineAABox2D(const double x1, const double y1,
			     const double x2, const double y2,
			     const double minx, const double miny,
			     const double maxx, const double maxy,
			     double * const interx1, double * const intery1,
			     double * const interx2, double * const intery2);


    static inline
    double Turn2D(const double p0[2], const double p1[2], const double p2[2])
    {
	return (p2[0] - p0[0]) * (p1[1] - p0[1]) - (p1[0] - p0[0]) * (p2[1] - p0[1]);
    }

    static inline
    bool IsPointLeftOfLine2D(const double p[2], const double p0[2], const double p1[2])
    {
	return Turn2D(p0, p1, p) <= 0.0;
    }

    static inline
    bool IsPointInsideTriangle2D(const double p[2],
				 const double tA[2],
				 const double tB[2],
				 const double tC[2])
    {
	return
	    IsPointLeftOfLine2D(p, tA, tB) &&
	    IsPointLeftOfLine2D(p, tB, tC) &&
	    IsPointLeftOfLine2D(p, tC, tA);
    }

    static inline
    bool IsPointInsideAABox2D(const double p[2],
			      const double min[2],
			      const double max[2])
    {
	return
	    p[0] >= min[0] && p[0] <= max[0] &&
	    p[1] >= min[1] && p[1] <= max[1];
    }

    bool IsPointInsideConvexPolygon2D(const double p[2],
				      const int    n,
				      const double poly[]);

    bool IsPointInsidePolygon2D(const double p[2],
				const int    n,
				const double poly[]);

    static inline
    bool IsPointInsideCircle2D(const double p[2],
			       const double cx,
			       const double cy,
			       const double r)
    {
	return
	    (cx - p[0]) * (cx - p[0]) +
	    (cy - p[1]) * (cy - p[1]) <= r * r;
    }

    static inline
    bool IsAABoxInsideAABox2D(const double min[2],
			      const double max[2],
			      const double min2[2],
			      const double max2[2])
    {
	return
	    min[0] >= min2[0] && max[0] <= max2[0] &&
	    min[1] >= min2[1] && max[1] <= max2[1];
    }

    bool IsPolygonInsideAABox2D(const int    n,
				const double poly[],
				const double min[2],
				const double max[2]);

    bool IsPolygonInsideConvexPolygon2D(const int    n1,
					const double poly1[],
					const int    n2,
					const double poly2[]);

    bool IsPolygonInsidePolygon2D(const int    n1,
				  const double poly1[],
				  const int    n2,
				  const double poly2[],
				  const bool   checkIntersection = true);

    bool IsPolygonInsideCircle2D(const int    n,
				 const double poly[],
				 const double cx,
				 const double cy,
				 const double r);



    static inline
    bool CollisionSegmentConvexPolygon2D(const double p0[2],
					 const double p1[2],
					 const int n,
					 const double poly[])
    {
	return
	    IsPointInsideConvexPolygon2D(p0, n, poly) ||
	    IsPointInsideConvexPolygon2D(p1, n, poly) ||
	    IntersectSegmentPolygon2D(p0, p1, n, poly);
    }

    static inline
    bool CollisionPolygonConvexPolygon2D(const int n1,
					 const double poly1[],
					 const int n2,
					 const double poly2[])
    {
	return
	    IsPointInsideConvexPolygon2D(poly1, n2, poly2) ||
	    IsPointInsidePolygon2D(poly2, n1, poly1) ||
	    IntersectPolygons2D(n1, poly1, n2, poly2);
    }

    static inline
    bool CollisionConvexPolygons2D(const int n1,
				   const double poly1[],
				   const int n2,
				   const double poly2[])
    {
	return
	    IsPointInsideConvexPolygon2D(poly1, n2, poly2) ||
	    IsPointInsideConvexPolygon2D(poly2, n1, poly1) ||
	    IntersectPolygons2D(n1, poly1, n2, poly2);
    }


    static inline
    bool CollisionSegmentPolygon2D(const double p0[2],
				   const double p1[2],
				   const int n,
				   const double poly[])
    {
	return
	    IsPointInsidePolygon2D(p0, n, poly) ||
	    IsPointInsidePolygon2D(p1, n, poly) ||
	    IntersectSegmentPolygon2D(p0, p1, n, poly);
    }

    static inline
    bool CollisionPolygons2D(const int n1,
			     const double poly1[],
			     const int n2,
			     const double poly2[])
    {
	return
	    IsPointInsidePolygon2D(poly1, n2, poly2) ||
	    IsPointInsidePolygon2D(poly2, n1, poly1) ||
	    IntersectPolygons2D(n1, poly1, n2, poly2);
    }

    static inline
    bool CollisionAABoxes2D(const double min1[2],
			    const double max1[2],
			    const double min2[2],
			    const double max2[2])
    {
	return
	    !(max2[0] < min1[0] || max1[0] < min2[0] ||
	      max2[1] < min1[1] || max1[1] < min2[1]);
    }


    double SignedAreaPolygon2D(const int n, const double poly[]);

    void BoundingBoxPolygon2D(const int    n,
			      const double poly[],
			      double       min[2],
			      double       max[2]);

    static inline
    void BoundingBoxCircle2D(const double center[2],
			     const double r,
			     double       min[2],
			     double       max[2])
    {
	min[0] = center[0] - r;
	min[1] = center[1] - r;
	max[0] = center[0] + r;
	max[1] = center[1] + r;
    }

    void ChangeOrientationPolygon2D(const int n, double poly[]);

    bool IsConvexPolygonCCW2D(const int n, const double poly[]);

    static inline
    bool IsPolygonCCW2D(const int n, const double poly[])
    {
	return SignedAreaPolygon2D(n, poly) >= 0.0;
    }

    static inline
    void MakeConvexPolygonCCW2D(const int n, double poly[])
    {
	if(!IsConvexPolygonCCW2D(n, poly))
	    ChangeOrientationPolygon2D(n, poly);
    }

    static inline
    void MakePolygonCCW2D(const int n, double poly[])
    {
	if(!IsPolygonCCW2D(n, poly))
	    ChangeOrientationPolygon2D(n, poly);
    }

    bool IsPolygonConvex2D(const int n, const double poly[]);

    void PolygonCentroid2D(const int n, const double poly[], double centroid[2]);


    int TriangulateConvexPolygon2D(const int n, const double poly[], int triangles[]);


    void SampleRandomPointInsideAABox2D(const double min[2],
					const double max[2],
					double       p[2]);

    void SampleRandomPointInsideTriangle2D(const double tA[2],
					   const double tB[2],
					   const double tC[2],
					   double       p[2]);

    void SampleRandomPointInsideConvexPolygon2D(const int    n,
						const double poly[],
						double       p[2],
						const double min[2] = NULL,
						const double max[2] = NULL);

    void SampleRandomPointInsidePolygon2D(const int    n,
					  const double poly[],
					  double       p[2],
					  const double min[2] = NULL,
					  const double max[2] = NULL);


    void SampleRandomPointInsideCircle2D(const double center[2],
					 const double r,
					 double       p[2]);


    void SampleRandomPointOnSphereSurface3D(const double center[3],
					    const double r,
					    double       p[3]);

    void SampleRandomPointOnSphereSurfacePatch3D(const double center[3],
						 const double r,
						 const double min_cos_phi,
						 const double max_cos_phi,
						 const double min_theta,
						 const double max_theta,
						 double       p[3]);

    void SampleRandomPointInsideSphere3D(const double center[], const double r, double p[3]);

    void SampleRandomPointOnSphereSurface(const int    nrDims,
					   const double center[],
					   const double r,
					   double       p[]);


    void SampleRandomPointOnEllipseSurface(const int    nrDims,
					   const double center[],
					   const double r[],
					   double       p[]);

    void ApplyTransRotToPolygon2D(const double TR[],
				  const int    n,
				  const double origPoly[],
				  double       newPoly[]);

    bool SelfIntersectPolygon2D(const int n, const double poly[]);

    int  HasVertexPolygon2D(const double x,
			    const double y,
			    const int    n,
			    const double poly[],
			    const double dtol);

    int  HasEdgePolygon2D(const double x1,
			  const double y1,
			  const double x2,
			  const double y2,
			  const int    n,
			  const double poly[],
			  const double dtol);

    bool HaveCommonEdgePolygons2D(const int    n1,
				  const double poly1[],
				  const int    n2,
				  const double poly2[],
				  const double dtol);

    bool ReadPolygon2D(FILE * const in, std::vector<double> * const poly);
    bool ReadPolygons2D(FILE * const in, std::vector< std::vector<double>* > * const polys);
    void PrintPolygon2D(FILE * const out, const int n, const double poly[]);

    void StabSkeleton2D(const int n,
			const double skeleton[],
			const double dist,
			std::vector<double> * const newsk);

    bool FromSkeletonToPolygon2D(const int n,
				const double skeleton[],
				const double thick,
				std::vector<double> * const poly);

    void GenerateArcAsPolygon2D(const double x,
				const double y,
				const double r,
				const double thetaStart,
				const double thetaInc,
				const int    nrPtsSkel,
				const double thick,
				std::vector<double> * const poly);


    void GenerateRadialPatternAsPolygons2D(const double x,
					   const double y,
					   const double rmin,
					   const double rmax,
					   const double thetaStart,
					   const double thetaInc,
					   const int    nrRays,
					   const double thick,
					   std::vector< std::vector<double>* > * const polys);





    int IntersectRayTriangle3D(const double p0Ray[3],
			       const double p1Ray[3],
			       const double tri0[3],
			       const double tri1[3],
			       const double tri2[3],
			       double       pinter[3]);

  static bool onSegment(double p[], double q[], double r[])
  {
    if (q[0] <= std::max(p[0], r[0]) && q[0] >= std::min(p[0], r[0]) &&
    q[1] <= std::max(p[1], r[1]) && q[1] >= std::min(p[1], r[1]))
    {
      return true;
    }
    return false;
  }

  static int orientation(double p[], double q[], double r[])
  {
    // See http://www.geeksforgeeks.org/orientation-3-ordered-points/
    // for details of below formula.
    double val = (q[1] - p[1]) * (r[0] - q[0]) -
    (q[0] - p[0]) * (r[1] - q[1]);

    if (val == 0) return 0;  // colinear

    return (val > 0)? 1: 2; // clock or counterclock wise
  }

  static bool doIntersect(double p1[], double p2[], double p3[], double p4[])
  {
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, p2, p3);
    int o2 = orientation(p1, p2, p4);
    int o3 = orientation(p3, p4, p1);
    int o4 = orientation(p3, p4, p2);

    // General case
    if (o1 != o2 && o3 != o4)
    return true;

    // Special Cases
    // p1, p2 and p3 are colinear and p3 lies on segment p1p2
    if (o1 == 0 && onSegment(p1, p3, p2)) return true;

    // p1, p2 and p3 are colinear and p4 lies on segment p1p2
    if (o2 == 0 && onSegment(p1, p4, p2)) return true;

    // p3, p4 and p1 are colinear and p1 lies on segment p3p4
    if (o3 == 0 && onSegment(p3, p1, p4)) return true;

    // p3, p4 and p2 are colinear and p2 lies on segment p3p4
    if (o4 == 0 && onSegment(p3, p2, p4)) return true;

    return false; // Doesn't fall in any of the above cases
  }

  static double DistSegments(double p1[], double p2[], double p3[], double p4[])
  {
    double p[2];
    if (doIntersect(p1,p2,p3,p4) == true)
    {
      return 0;
    }
    double d1 = DistSquaredPointSegment2D(p1,p3,p4,p);
    double d2 = DistSquaredPointSegment2D(p2,p3,p4,p);
    double d3 = DistSquaredPointSegment2D(p3,p1,p2,p);
    double d4 = DistSquaredPointSegment2D(p4,p1,p2,p);
    return sqrt(( std::min(d1,std::min(d2,std::min(d3,d4))) ));
  }


}

#endif
