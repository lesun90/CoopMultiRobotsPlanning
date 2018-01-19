#include "Utils/Geometry.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/Constants.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/PrintMsg.hpp"
#include "Utils/Algebra3D.hpp"

namespace MP
{

    void CircleAsPolygon2D(const double cx,
			   const double cy,
			   const double r,
			   const int    n,
			   double       poly[])
    {
	const double theta     = 2 * M_PI / n;
	const double cos_theta = cos(theta);
	const double sin_theta = sin(theta);
	double       x         = 1;
	double       y         = 0;

	for(int j = 0; j < n; ++j)
	{
	    poly[2 * j]     = cx + r * x;
	    poly[2 * j + 1] = cy + r * y;

	    const double tmp = x;
	    x = cos_theta * tmp - sin_theta * y;
	    y = sin_theta * tmp + cos_theta * y;
	}
    }

    double DistSquaredPointLine2D(const double p[2],
				  const double s0[2],
				  const double s1[2],
				  double       pmin[])
    {
	const double vx = s1[0] - s0[0];
	const double vy = s1[1] - s0[1];
	const double a  = vx * (s0[1] - p[1]) - vy * (s0[0] - p[0]);
	const double b  = a / (vx * vx + vy * vy);

	pmin[0] = p[0] - b * vy;
	pmin[1] = p[1] + b * vx;

	return a * b;
    }

    double DistSquaredPointSegment2D(const double p[2],
				     const double s0[2],
				     const double s1[2],
				     double       pmin[2])
    {
	double a, b;
	const double vx = s1[0] - s0[0];
	const double vy = s1[1] - s0[1];

	if((a = (vx * (p[0] - s0[0]) + vy * (p[1] - s0[1]))) <= 0)
	{
	    pmin[0] = s0[0];
	    pmin[1] = s0[1];
	}
	else if((b = (vx * vx + vy * vy)) <= a)
	{
	    pmin[0] = s1[0];
	    pmin[1] = s1[1];
	}
	else
	{
	    a   /= b;
	    pmin[0] = s0[0] + a * vx;
	    pmin[1] = s0[1] + a * vy;
	}

	return (p[0] - pmin[0]) * (p[0] - pmin[0]) +  (p[1] - pmin[1]) * (p[1] - pmin[1]);
    }

    double DistSquaredPointPolygon2D(const double p[2],
				     const int    n,
				     const double poly[],
				     double       pmin[2])
    {
	double dmin = DistSquaredPointSegment2D(p, &poly[2 * n - 2], &(poly[0]), pmin);
	double d;
	double ptmp[2];
	int    i;

	for(i = 0; i < n - 1; ++i)
	    if((d = DistSquaredPointSegment2D(p, &poly[2 * i], &poly[2 * i + 2], ptmp)) < dmin)
	    {
		pmin[0] = ptmp[0];
		pmin[1] = ptmp[1];
		dmin = d;
	    }
	return dmin;
    }

    double DistSquaredSegments2D(const double p1[2],
				 const double p2[2],
				 const double p3[2],
				 const double p4[2],
				 double       pmin1[2],
				 double       pmin2[2])
    {

	const double u[] = {p2[0] - p1[0], p2[1] - p1[1]};
	const double v[] = {p4[0] - p3[0], p4[1] - p3[1]};
	const double w[] = {p1[0] - p3[0], p1[1] - p3[1]};
	const double a = u[0] * u[0] + u[1] * u[1];
	const double b = u[0] * v[0] + u[1] * v[1];
	const double c = v[0] * v[0] + v[1] * v[1];
	const double d = u[0] * w[0] + u[1] * w[1];
	const double e = v[0] * w[0] + v[1] * w[1];
	const double D = a * c - b * b;    // always >= 0
	double    sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
	double    tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

	// compute the line parameters of the two closest points
	if (D < Constants::EPSILON)
	{ // the lines are almost parallel
	    sN = 0.0;        // force using point P0 on segment S1
	    sD = 1.0;        // to prevent possible division by 0.0 later
	    tN = e;
	    tD = c;
	}
	else
	{                // get the closest points on the infinite lines
	    sN = (b*e - c*d);
	    tN = (a*e - b*d);
	    if (sN < 0.0)
	    {       // sc < 0 => the s=0 edge is visible
		sN = 0.0;
		tN = e;
		tD = c;
	    }
	    else if (sN > sD)
	    {  // sc > 1 => the s=1 edge is visible
		sN = sD;
		tN = e + b;
		tD = c;
	    }
	}

	if (tN < 0.0)
	{           // tc < 0 => the t=0 edge is visible
	    tN = 0.0;
	    // recompute sc for this edge
	    if (-d < 0.0)
		sN = 0.0;
	    else if (-d > a)
		sN = sD;
	    else
	    {
		sN = -d;
		sD = a;
	    }
	}
	else if (tN > tD)
	{      // tc > 1 => the t=1 edge is visible
	    tN = tD;
	    // recompute sc for this edge
	    if ((-d + b) < 0.0)
		sN = 0;
	    else if ((-d + b) > a)
		sN = sD;
	    else
	    {
		sN = (-d + b);
		sD = a;
	    }
	}
	// finally do the division to get sc and tc
	sc = (fabs(sN) < Constants::EPSILON ? 0.0 : sN / sD);
	tc = (fabs(tN) < Constants::EPSILON ? 0.0 : tN / tD);

	pmin1[0] = p1[0] + sc * u[0];
	pmin1[1] = p1[1] + sc * u[1];

	pmin2[0] = p3[0] + tc * v[0];
	pmin2[1] = p3[1] + tc * v[1];

	const double dx = pmin2[0] - pmin1[0];
	const double dy = pmin2[1] - pmin1[1];
	return dx * dx + dy * dy;
    }

    double DistSegments2D(const double p1[2],
     				 const double p2[2],
     				 const double p3[2],
     				 const double p4[2])
    {
      double a[2];
      double b[2];
      double c[2];
      double d[2];

      a[0] = p1[0];
      a[1] = p1[0];
      b[0] = p2[0];
      b[1] = p2[1];
      c[0] = p3[0];
      c[1] = p3[1];
      d[0] = p4[0];
      d[1] = p4[1];

      double p5[2],p6[2];
      //there is a bug which make calculation err when p3 and p4 make a point
      if ((p3[0] == p4[0]) && (p3[1] == p4[1]))
      {
        return sqrt(DistSquaredSegments2D(c,d,a,b,p5,p6));
      }
      return sqrt(DistSquaredSegments2D(a,b,c,d,p5,p6));
    }

    double DistSquaredSegmentPolygon2D(const double p1[2],
				       const double p2[2],
				       const int    n,
				       const double poly[],
				       double       pmin1[2],
				       double       pmin2[2])
    {
	double ptmp1[2], ptmp2[2];

	double dmin = DistSquaredSegments2D(p1, p2, &poly[2 * n - 2], &poly[0], pmin1, pmin2);
	double d;

	for(int i = 0; i < n - 1; ++i)
	{
	    if((d = DistSquaredSegments2D(p1, p2, &poly[2 * i], &poly[2 * i + 2], ptmp1, ptmp2)) < dmin)
	    {
		dmin = d;
		pmin1[0] = ptmp1[0];
		pmin1[1] = ptmp1[1];
		pmin2[0] = ptmp2[0];
		pmin2[1] = ptmp2[1];
	    }
	}

	return dmin;
    }


    double DistSquaredPolygons2D(const int    n1,
				 const double poly1[],
				 const int    n2,
				 const double poly2[],
				 double       pmin1[2],
				 double       pmin2[2])
    {
	double ptmp1[2], ptmp2[2];

	double dmin = DistSquaredSegmentPolygon2D(&poly1[2 * n1 - 2], &poly1[0], n2, poly2, pmin1, pmin2);
	double d;

	for(int i = 0; i < n1 - 1; ++i)
	    if((d = DistSquaredSegmentPolygon2D(&poly1[2 * i], &poly1[2 * i + 2], n2, poly2, ptmp1, ptmp2)) < dmin)
	    {
		dmin = d;
		pmin1[0] = ptmp1[0];
		pmin1[1] = ptmp1[1];
		pmin2[0] = ptmp2[0];
		pmin2[1] = ptmp2[1];
	    }
	return dmin;
    }

    bool IntersectLines2D(const double x1, const double y1,
			  const double x2, const double y2,
			  const double x3, const double y3,
			  const double x4, const double y4,
			  double * const x,   double * const y,
			  double * const mua, double * const mub)
    {
	const double denom  = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);

	if(fabs(denom) < Constants::EPSILON)
	    return false;
	*mua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
	*mub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;
	*x   = x1 + (*mua) * (x2 - x1);
	*y   = y1 + (*mua) * (y2 - y1);

	return true;

//	const double numera = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
//	const double numerb = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);
	/*
	// Are the line coincident? or para
	if (fabs(numera) < Constants::EPSILON &&
	    fabs(numerb) < Constants::EPSILON &&
	    fabs(denom) < Constants::EPSILON)
	{
	    *x = (x1 + x2) / 2;
	    *y = (y1 + y2) / 2;
	    return false;
	}

	// Are the line parallel
	if (fabs(denom) < Constants::EPSILON)
	{
	    *x = *y = 0;
	    return false;
	}
	*/

	// Is the intersection along the segments
	//*mua = numera / denom;
	//*mub = numerb / denom;
  }

    bool IntersectSegments2D(const double x1, const double y1,
			     const double x2, const double y2,
			     const double x3, const double y3,
			     const double x4, const double y4)
    {
	double Cx,Ay,By,Cy,d,e,f;
	double x1lo,x1hi,y1lo,y1hi;
	double Ax = x2 - x1;
	double Bx = x3 - x4;

	if(Ax < 0) { x1lo = x2; x1hi = x1; }
	else       { x1hi = x2; x1lo = x1; }

	if(Bx > 0) { if(x1hi < x4 || x3 < x1lo) return false; }
	else       { if(x1hi < x3 || x4 < x1lo) return false; }

	Ay = y2 - y1;
	By = y3 - y4;

	/* Y bound box test*/
	if(Ay < 0)  { y1lo = y2; y1hi = y1; }
	else        { y1hi = y2; y1lo = y1; }

	if(By > 0)  { if(y1hi < y4 || y3 < y1lo) return false; }
	else          if(y1hi < y3 || y4 < y1lo) return false;

	f  = Ay * Bx - Ax * By;					/* both denominator*/
	if(f == 0)
	    return false;

	Cx = x1 - x3;
	Cy = y1 - y3;

	d  = By * Cx - Bx * Cy;					/* alpha numerator*/
	/* alpha tests*/
	if(f > 0) { if(d < 0 || d > f) return false; }
	else if(d > 0 || d < f) return false;

	e = Ax * Cy - Ay * Cx;					/* beta numerator*/
	/* beta tests*/
	if(f > 0) { if(e < 0 || e > f) return false; }
	else if(e > 0 || e < f) return false;

//    printf("inter = %f %f\n", x1 + (d/f) * (x2 - x1), y1 + (d/f) * (y2 - y1));

	return true;
    }


    bool IntersectSegmentPolygon2D(const double p0[2],
				   const double p1[2],
				   const int n,
				   const double poly[])
    {
	for(int i = 0; i < n - 1; ++i)
	    if(IntersectSegments2D(p0, p1, &(poly[2 * i]), &(poly[2 * i + 2])))
		return true;
	return IntersectSegments2D(p0, p1, &(poly[2 * n - 2]), &(poly[0]));
    }


    bool IntersectPolygons2D(const int n1,
			     const double poly1[],
			     const int n2,
			     const double poly2[])
    {
	for(int i = 0; i < n1 - 1; ++i)
	    if(IntersectSegmentPolygon2D(&(poly1[2 * i]), &(poly1[2 * i + 2]), n2, poly2))
		return true;
	return IntersectSegmentPolygon2D(&(poly1[2 * n1 - 2]), &(poly1[0]), n2, poly2);
    }

    int IntersectLineAABox2D(const double x1, const double y1,
			     const double x2, const double y2,
			     const double minx, const double miny,
			     const double maxx, const double maxy,
			     double * const interx1, double * const intery1,
			     double * const interx2, double * const intery2)
    {
	double p[4]  = {HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL};
	int    count = 0;

	if(IntersectLineSegment2D(x1, y1, x2, y2, minx, miny, maxx, miny, &p[0], &p[1]))
	    ++count;

	if(count < 2 &&
	   IntersectLineSegment2D(x1, y1, x2, y2, maxx, miny, maxx, maxy, &p[2 * count], &p[2 * count + 1]))
	    ++count;

	if(count < 2 &&
	   IntersectLineSegment2D(x1, y1, x2, y2, maxx, maxy, minx, maxy, &p[2 * count], &p[2 * count + 1]))
	    ++count;

	if(count < 2 &&
	   IntersectLineSegment2D(x1, y1, x2, y2, minx, maxy, minx, miny, &p[2 * count], &p[2 * count + 1]))
	    ++count;

	*interx1 = p[0];
	*intery1 = p[1];
	*interx2 = p[2];
	*intery2 = p[3];

	return count;
    }





    bool IsPointInsideConvexPolygon2D(const double p[2],
				      const int    n,
				      const double poly[])
    {
	for(int i = 0; i < 2 * n - 2; i = i + 2)
	    if(IsPointLeftOfLine2D(p, &poly[i], &poly[i + 2]) == false)
		return false;
	return IsPointLeftOfLine2D(p, &poly[2 * n - 2], &poly[0]);
    }

    bool IsPointInsidePolygon2D(const double p[2],
				const int    n,
				const double poly[])
    {
	if(n == 3)
	    return IsPointInsideTriangle2D(p, &poly[0], &poly[2], &poly[4]);
//	else if (n == 2)
//	    return PointSegmentDistSquared2D(p, &poly[0], &poly[2], pmin) <= Constants::EPSILON_SQUARED;
//
	int i, i1, lcross = 0, rcross = 0;
	double px = p[0];
	double py = p[1];
	double x = 0.0;
	bool testr;
	bool testl;

	for(i = 0; i < n; i++)
	{
	    if(poly[2 * i] == px && poly[2 * i + 1] == py)
		return true;

	    i1 = (i + n - 1) % n;

	    testr = (poly[2 * i + 1] > py) != (poly[2 * i1 + 1] > py);
	    testl = (poly[2 * i + 1] < py) != (poly[2 * i1 + 1] < py);

	    if(testr || testl)
		x = (poly[2 * i] * poly[2 * i1 + 1] - poly[2 * i1] * poly[2 * i + 1] +
		     py * (poly[2 * i1] - poly[2 * i])) /
		    (poly[2 * i1 + 1] - poly[2 * i + 1]);

	    if(testr && x > px)
		rcross++;
	    if(testl && x < px)
		lcross++;
	}

	if( (rcross & 1) != (lcross & 1) )
	    return true;
	return (rcross & 1);

    }

    bool IsPolygonInsideAABox2D(const int n,
				const double poly[],
				const double min[2],
				const double max[2])
    {
	for(int i = 0; i < n; ++i)
	    if(IsPointInsideAABox2D(&(poly[i << 1]), min, max) == false)
		return false;
	return true;
    }

    bool IsPolygonInsideConvexPolygon2D(const int n1,
					const double poly1[],
					const int n2,
					const double poly2[])
    {
	for(int i = 0; i < n1; ++i)
	    if(IsPointInsideConvexPolygon2D(&poly1[2 * i], n2, poly2) == false)
		return false;
	return true;
    }

    bool IsPolygonInsidePolygon2D(const int    n1,
				  const double poly1[],
				  const int    n2,
				  const double poly2[],
				  const bool   checkIntersection)
    {
	for(int i = 0; i < n1; ++i)
	    if(IsPointInsidePolygon2D(&poly1[2 * i], n2, poly2) == false)
		return false;
	return !checkIntersection || IntersectPolygons2D(n1, poly1, n2, poly2) == false;
    }

    bool IsPolygonInsideCircle2D(const int    n,
				 const double poly[],
				 const double cx,
				 const double cy,
				 const double r)
    {
	for(int i = 0; i < n; ++i)
	    if(IsPointInsideCircle2D(&poly[2 * i], cx, cy, r) == false)
		return false;
	return true;
    }



    double SignedAreaPolygon2D(const int n, const double poly[])
    {
	int    inext;
	double a = 0;
	for(int i = 0; i < n; ++i)
	{
	    inext = (i + 1) % n;
	    a    += poly[(i << 1)] * poly[(inext << 1) + 1] - poly[(inext << 1)] * poly[(i << 1) + 1];
	}
	return a / 2;
    }


    void BoundingBoxPolygon2D(const int n,
			      const double poly[],
			      double       min[2],
			      double       max[2])
    {
	min[0] = max[0] = poly[0];
	min[1] = max[1] = poly[1];

	for(int i = 1; i < n; ++i)
	{
	    if(poly[2 * i] < min[0])
		min[0] = poly[2 * i];
	    else if(poly[2 * i] > max[0])
		max[0] = poly[2 * i];

	    if(poly[2 * i + 1] < min[1])
		min[1] = poly[2 * i + 1];
	    else if(poly[2 * i + 1] > max[1])
		max[1] = poly[2 * i + 1];
	}
    }

    void ChangeOrientationPolygon2D(const int n, double poly[])
    {
	double t;
	for(int i = 0, j = 2 * n - 2; i < j; i += 2, j -= 2)
	{
	    t = poly[i];     poly[i]     = poly[j];      poly[j]     = t;
	    t = poly[i + 1]; poly[i + 1] = poly[j + 1];  poly[j + 1] = t;
	}
    }


    bool IsConvexPolygonCCW2D(const int n, const double poly[])
    {
	int    i;
	double z;

	if(n < 3)
	    return true;

	for(int i = 0; i < n; i++)
	{
	    z = Turn2D(&poly[2 * i], &poly[2 * ((i + 1) % n)], &poly[2 * ((i + 2) % n)]);
	    if(z < -Constants::EPSILON)
		return true;
	    if(z > Constants::EPSILON)
		return false;
	}
	return true;
    }

    bool IsPolygonConvex2D(const int n, const double poly[])
    {
	if(n <= 3)
	    return true;

	const bool turn = Turn2D(&poly[0], &poly[2], &poly[4]) <= Constants::EPSILON;

	for(int i = 1; i < n; i++)
	    if(turn != (Turn2D(&poly[2 * i],
			       &poly[2 * ((i + 1) % n)],
			       &poly[2 * ((i + 2) % n)]) <= Constants::EPSILON))
		return false;
	return true;
    }


    void PolygonCentroid2D(const int n, const double poly[], double centroid[2])
    {
	centroid[0] = centroid[1] = 0;
	for(int i = 0; i < n; i++)
	{
	    centroid[0] += poly[2 * i];
	    centroid[1] += poly[2 * i + 1];
	}
	centroid[0] /= n;
	centroid[1] /= n;
    }
    int TriangulateConvexPolygon2D(const int n, const double poly[], int triangles[])
    {
	for(int i = 0; i < n - 2; ++i)
	{
	    triangles[3 * i]     = 0;
	    triangles[3 * i + 1] = i + 1;
	    triangles[3 * i + 2] = i + 2;
	}
	return n - 2;
    }

    void SampleRandomPointInsideAABox2D(const double min[2],
					const double max[2],
					double       p[2])
    {
	p[0] = RandomUniformReal(min[0], max[0]);
	p[1] = RandomUniformReal(min[1], max[1]);
    }


    void SampleRandomPointInsideTriangle2D(const double tA[2],
					   const double tB[2],
					   const double tC[2],
					   double       p[2])
    {
	double a1 = RandomUniformReal();
	double a2 = RandomUniformReal();

	if(a1 + a2 > 1.0)
	{
	    a1 = 1 - a1;
	    a2 = 1 - a2;
	}

	p[0] = tA[0] + a1 * (tB[0] - tA[0]) + a2 * (tC[0] - tA[0]);
	p[1] = tA[1] + a1 * (tB[1] - tA[1]) + a2 * (tC[1] - tA[1]);
    }


    void SampleRandomPointInsideConvexPolygon2D(const int    n,
						const double poly[],
						double       p[2],
						const double min[2],
						const double max[2])
    {
	double umin[2], umax[2];
	if(min == NULL || max == NULL)
	    BoundingBoxPolygon2D(n, poly, umin, umax);
	else
	{
#if DEBUG
	    assert(min != NULL);
	    assert(max != NULL);
#endif
	    umin[0] = min[0];
	    umin[1] = min[1];
	    umax[0] = max[0];
	    umax[1] = max[1];
	}

	do
	    SampleRandomPointInsideAABox2D(umin, umax, p);
	while(IsPointInsideConvexPolygon2D(p, n, poly) == false);

    }

    void SampleRandomPointInsidePolygon2D(const int    n,
					  const double poly[],
					  double       p[2],
					  const double min[2],
					  const double max[2])
    {
	double umin[2], umax[2];
	if(min == NULL || max == NULL)
	    BoundingBoxPolygon2D(n, poly, umin, umax);
	else
	{
#if DEBUG
	    assert(min != NULL);
	    assert(max != NULL);
#endif
	    umin[0] = min[0];
	    umin[1] = min[1];
	    umax[0] = max[0];
	    umax[1] = max[1];
	}

	int count = 0;

	do
	{
	    ++count;
	    SampleRandomPointInsideAABox2D(umin, umax, p);
	}
		while(IsPointInsidePolygon2D(p, n, poly) == false);
    }

    void SampleRandomPointInsideCircle2D(const double center[2], const double r, double p[2])
    {
	const double s     = r * sqrt(RandomUniformReal());
	const double theta = RandomUniformReal(0, 2 * M_PI);

	p[0] = s * cos(theta) + center[0];
	p[1] = s * sin(theta) + center[1];
    }

    void SampleRandomPointOnSphereSurface3D(const double center[3], const double r, double p[3])
    {
	double x, y, s;

	do
	{
	    x = RandomUniformReal(-1, 1);
	    y = RandomUniformReal(-1, 1);
	}
	while((s = (x * x + y * y)) >= 1.0);

	p[2] = center[0] + r - 2 * r * s;
	s    = 2 * r * sqrt(1 - s);
	p[0] = center[1] + x * s;
	p[1] = center[2] + y * s;
    }

    void SampleRandomPointInsideSphere3D(const double center[], const double r, double p[3])
    {
	double x, y, z;
	do
	{
	    x = RandomUniformReal(-1, 1);
	    y = RandomUniformReal(-1, 1);
	    z = RandomUniformReal(-1, 1);
	}
	while((x * x + y * y + z * z) >= 1.0);

	p[0] = center[0] + r * x;
	p[1] = center[1] + r * y;
	p[2] = center[2] + r * z;
    }



    void SampleRandomPointOnSphereSurfacePatch3D(const double center[3],
						 const double r,
						 const double min_cos_phi,
						 const double max_cos_phi,
						 const double min_theta,
						 const double max_theta,
						 double       p[3])

    {
	const double u     = RandomUniformReal(min_cos_phi, max_cos_phi);
	const double w     = sqrt(1 - u * u);
	const double theta = RandomUniformReal(min_theta, max_theta);

	p[0] = center[0] + r * w * cos(theta);
	p[1] = center[1] + r * w * sin(theta);
	p[2] = center[2] + r * u;
    }


    void SampleRandomPointOnSphereSurface(const int nrDims, const double center[], const double r, double p[])
    {
	double s = 0.0;

	for(int i = 0; i < nrDims; i++)
	{
	    p[i] = RandomGaussianReal(0.0, 1.0);
	    s   += p[i] * p[i];
	}
	s = r / sqrt(s);
	for(int i = 0; i < nrDims; ++i)
	    p[i] = center[i] + p[i] * s;
    }

    void SampleRandomPointOnEllipseSurface(const int    nrDims,
					   const double center[],
					   const double r[],
					   double       p[])
    {
	double s = 0.0;

	for(int i = 0; i < nrDims; ++i)
	{
	    p[i] = RandomGaussianReal(0.0, 1.0);
	    s   += p[i] * p[i];
	}
	s = 1.0 / sqrt(s);
	for(int i = 0; i < nrDims; ++i)
	    p[i] = center[i] + p[i] * r[i] * s;
    }



    void ApplyTransRotToPolygon2D(const double TR[],
				  const int    n,
				  const double origPoly[],
				  double       newPoly[])
    {
	for(int i = 0; i < n; ++i)
	    Algebra2D::TransRotMultPoint(TR, &origPoly[i << 1], &newPoly[i << 1]);
    }



    bool SelfIntersectPolygon2D(const int n, const double poly[])
    {
	for(int i = 0; i < n - 2; ++i)
	    for(int j = i + 2; j < n; ++j)
		if((i != 0 || j != (n - 1)) &&
		   IntersectSegments2D(&poly[2 * i], &poly[2 * i + 2],
				       &poly[2 * j], &poly[2 * ((j + 1) % n)]))
		    return true;
	return false;
    }

    int HasVertexPolygon2D(const double x,
			   const double y,
			   const int    n,
			   const double poly[],
			   const double dtol)
    {
	for(int i = 0; i < n; ++i)
	    if(dtol * dtol >= ((poly[2 * i    ] - x) * (poly[2 * i    ] - x) +
			       (poly[2 * i + 1] - y) * (poly[2 * i + 1] - y)))
		return i;
	return -1;
    }


    int HasEdgePolygon2D(const double x1,
			 const double y1,
			 const double x2,
			 const double y2,
			 const int    n,
			 const double poly[],
			 const double dtol)
    {
	int j;
	for(int i = 0; i < n; ++i)
	{
	    if(dtol * dtol >= ((poly[2 * i    ] - x1) * (poly[2 * i    ] - x1) +
			       (poly[2 * i + 1] - y1) * (poly[2 * i + 1] - y1)))

	    {
		j = (i + 1) % n;
		if(dtol * dtol >= ((poly[2 * j    ] - x2) * (poly[2 * j    ] - x2) +
				   (poly[2 * j + 1] - y2) * (poly[2 * j + 1] - y2)))
		    return i;
		else
		{
		    j = (n + i - 1) % n;
		    if(dtol * dtol >= ((poly[2 * j    ] - x2) * (poly[2 * j    ] - x2) +
				       (poly[2 * j + 1] - y2) * (poly[2 * j + 1] - y2)))
			return j;
		}
	    }

	}
	return -1;
    }


    bool HaveCommonEdgePolygons2D(const int    n1,
				  const double poly1[],
				  const int    n2,
				  const double poly2[],
				  const double dtol)
    {
	for(int i = 0; i < n1; ++i)
	{
	    const int j = (i + 1) % n1;
	    if(HasEdgePolygon2D(poly1[2 * i], poly1[2 * i + 1],
				poly1[2 * j], poly1[2 * j + 1],
				n2, poly2, dtol) >= 0)
		return true;
	}
	return false;
    }

    bool ReadPolygon2D(FILE * const in, std::vector<double> * const poly)
{
  int    n = 0;
  double v;

  if(fscanf(in, "%d", &n) != 1)
  {
    BeginPrint(PRINT_ERROR);
    printf("..could not read number of polygon vertices\n");
    EndPrint(PRINT_ERROR);
    return false;
  }

  n *= 2;

  poly->resize(n);
  for(int j = 0; j < n; ++j)
  {
    if(fscanf(in, "%lf", &v) != 1)
    {
      BeginPrint(PRINT_ERROR);
      printf("..could not read value %d out of %d values\n", j, n);
      EndPrint(PRINT_ERROR);
      return false;
    }

    (*poly)[j] = v;
  }

return true;
}

bool ReadPolygons2D(FILE * const in, std::vector< std::vector<double>* > * const polys)
{
int n = 0;

if(fscanf(in, "%d", &n) != 1)
{
  BeginPrint(PRINT_ERROR);
  printf("..could not read number of polygons\n");
  EndPrint(PRINT_ERROR);
  return false;
}

for(int i = 0; i < n; ++i)
{
  std::vector<double> *poly = new std::vector<double>();
  if(ReadPolygon2D(in, poly) == false)
  {
BeginPrint(PRINT_ERROR);
printf("..errors while reading polygon %d out of %d polygons\n", i, n);
EndPrint(PRINT_ERROR);
delete poly;
return false;
  }
  polys->push_back(poly);
}
return true;
}

void PrintPolygon2D(FILE * const out, const int n, const double poly[])
{
fprintf(out, "%d ", n);
for(int i = 0; i < 2 * n; ++i)
  fprintf(out, "%f ", poly[i]);
fprintf(out, "\n");
}

    bool FromSkeletonToPolygon2D(const int n,
				const double skeleton[],
				const double thick,
				std::vector<double> * const poly)
    {
	if (n < 2)
	    return false;

	double v[2], u[2], p[2], line1[4], line2[4], x, y, mua, mub;

//first point -- chain 1
	v[0] = skeleton[2] - skeleton[0];
	v[1] = skeleton[3] - skeleton[1];
	Algebra2D::VecNormal(v, u);
	Algebra2D::VecUnit(u, u);
	x = skeleton[0] + 0.5 * thick * u[0];
	y = skeleton[1] + 0.5 * thick * u[1];
	poly->push_back(x);
	poly->push_back(y);

	line1[0] = x;
	line1[1] = y;
	line1[2] = x + v[0];
	line1[3] = y + v[1];


	for(int i = 1; i < n - 1; ++i)
	{
	    v[0] = skeleton[2 * i + 2] - skeleton[2 * i];
	    v[1] = skeleton[2 * i + 3] - skeleton[2 * i + 1];
	    Algebra2D::VecNormal(v, u);
	    Algebra2D::VecUnit(u, u);
	    line2[0] = skeleton[2 * i    ] + 0.5 * thick * u[0];
	    line2[1] = skeleton[2 * i + 1] + 0.5 * thick * u[1];
	    line2[2] = line2[0] + v[0];
	    line2[3] = line2[1] + v[1];

	    if(IntersectLines2D(line1[0], line1[1], line1[2], line1[3],
				line2[0], line2[1], line2[2], line2[3],
				&x, &y, &mua, &mub))
	    {
		poly->push_back(x);
		poly->push_back(y);
	    }
	    else
	    {
		poly->push_back(line1[2]);
		poly->push_back(line1[3]);

		//return false;
	    }

	    line1[0] = line2[0];
	    line1[1] = line2[1];
	    line1[2] = line2[2];
	    line1[3] = line2[3];


	}

//last point chain 1
	v[0] = skeleton[2 * n - 2] - skeleton[2 * n - 4];
	v[1] = skeleton[2 * n - 1] - skeleton[2 * n - 3];
	Algebra2D::VecNormal(v, u);
	Algebra2D::VecUnit(u, u);
	x = skeleton[2 * n - 2] + 0.5 * thick * u[0];
	y = skeleton[2 * n - 1] + 0.5 * thick * u[1];
	poly->push_back(x);
	poly->push_back(y);

//chain 2
	for(int i = n - 1; i >= 0; --i)
	{
	    v[0] = (*poly)[2 * i] - skeleton[2 * i];
	    v[1] = (*poly)[2 * i + 1] - skeleton[2 * i + 1];
	    poly->push_back(skeleton[2 * i] - v[0]);
	    poly->push_back(skeleton[2 * i + 1] - v[1]);
	}

	return true;

    }

    void GenerateArcAsPolygon2D(const double x,
				const double y,
				const double r,
				const double thetaStart,
				const double thetaInc,
				const int    nrPtsSkel,
				const double thick,
				std::vector<double> * const poly)
    {
	std::vector<double> skel;
	skel.resize(2 * nrPtsSkel);
	for(int i = 0; i < nrPtsSkel; ++i)
	{
	    skel[2 * i    ] = x + r * cos(thetaStart + i * thetaInc);
	    skel[2 * i + 1] = y + r * sin(thetaStart + i * thetaInc);
	}

	FromSkeletonToPolygon2D(nrPtsSkel, &skel[0], thick, poly);
    }

    void GenerateRadialPatternAsPolygons2D(const double x,
					   const double y,
					   const double rmin,
					   const double rmax,
					   const double thetaStart,
					   const double thetaInc,
					   const int    nrRays,
					   const double thick,
					   std::vector< std::vector<double>* > * const polys)
    {
	std::vector<double> *poly;
	double skel[4];

	for(int i = 0; i < nrRays; ++i)
	{
	    const double c = cos(thetaStart + i * thetaInc);
	    const double s = sin(thetaStart + i * thetaInc);

	    skel[0] = x + rmin * c;
	    skel[1] = y + rmin * s;
	    skel[2] = x + rmax * c;
	    skel[3] = y + rmax * s;

	    printf("skel %d: %f %f %f %f\n", i, skel[0], skel[1], skel[2], skel[3]);


	    poly = new std::vector<double>();
	    FromSkeletonToPolygon2D(2, skel, thick, poly);
	    polys->push_back(poly);
	}
    }

    void StabSkeleton2D(const int n,
			const double skeleton[],
			const double dist,
			std::vector<double> * const newsk)
    {
	for(int i = 0; i < n - 1; ++i)
	{
	    newsk->push_back(skeleton[2 * i]);
	    newsk->push_back(skeleton[2 * i + 1]);

	    const double d = Algebra2D::PointDist(&skeleton[2 * i], &skeleton[2 * i + 2]);
	    if(d > dist)
	    {
		const int nrSteps = (int) d / dist;
		for(int j = 1; j < nrSteps; ++j)
		{
		    const double t = (j + 0.0) / nrSteps;

		    newsk->push_back((1 - t) * skeleton[2 * i] + t * skeleton[2 * i + 2]);
		    newsk->push_back((1 - t) * skeleton[2 * i + 1] + t * skeleton[2 * i + 3]);
		}
	    }
	}

	newsk->push_back(skeleton[2 * n - 2]);
	newsk->push_back(skeleton[2 * n - 1]);


    }

    //code adapted from
    //http://geomalgorithms.com/a06-_intersect-2.html#intersect3D_RayTriangle()
//    Return: -1 = triangle is degenerate (a segment or point)
//             0 =  disjoint (no intersect)
//             1 =  intersect in unique point I1
//             2 =  are in the same plane

    int IntersectRayTriangle3D(const double p0[3],
			       const double p1[3],
			       const double tri0[3],
			       const double tri1[3],
			       const double tri2[3],
			       double       pinter[3])
    {
	const double u[] = {tri1[0] - tri0[0], tri1[1] - tri0[1], tri1[2] - tri0[2]};
	const double v[] = {tri2[0] - tri0[0], tri2[1] - tri0[1], tri2[2] - tri0[2]};
	double       uvc[3];

	Algebra3D::VecCrossProduct(u, v, uvc);
	if(uvc[0] == 0 && uvc[1] == 0 && uvc[2] == 0)
	    return -1;

	const double dir[] = {p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]};
	const double w0[]  = {p0[0] - tri0[0], p0[1] - tri0[1], p0[2] - tri0[2]};
	const double a     = -Algebra3D::VecDotProduct(uvc, w0);
	const double b     =  Algebra3D::VecDotProduct(uvc, dir);


	if(fabs(b) < Constants::EPSILON) //ray is parallel to triangle plane
	    return a == 0 ? 2 : 0;       //ray in plane / ray disjoint from plane

	const double r = a / b;


	if(r < 0)                        //ray goes away from triangle
	    return 0;
	//for a segment also test if (r > 1.0) => no intersect

//	printf("dir = %f %f %f [a=%f, b=%f, a/b=%f\n", dir[0], dir[1], dir[2], a, b, r);

	pinter[0] = p0[0] + r * dir[0];
	pinter[1] = p0[1] + r * dir[1];
	pinter[2] = p0[2] + r * dir[2];

//	printf("pinter here = %f %f %f [r=%f]\n", pinter[0], pinter[1], pinter[2], r);

	const double uu = Algebra3D::VecDotProduct(u, u);
	const double uv = Algebra3D::VecDotProduct(u, v);
	const double vv = Algebra3D::VecDotProduct(v, v);
	const double w[]= {pinter[0] - tri0[0], pinter[1] - tri0[1], pinter[2] - tri0[2]};
	const double wu = Algebra3D::VecDotProduct(w, u);
	const double wv = Algebra3D::VecDotProduct(w, v);
	const double D  = uv * uv - uu * vv;
	const double s  = (uv * wv - vv * wu) / D;
	if(s < 0 || s > 1)
	    return 0;
	const double t = (uv * wu - uu * wv) / D;
	if(t < 0 || (s + t) > 1)
	    return 0;

//	printf("dir = %f %f %f [a=%f, b=%f, a/b=%f] uvc=%f %f %f u=%f %f %f v = %f %f %f\n",
//	       dir[0], dir[1], dir[2], a, b, r, uvc[0], uvc[1], uvc[2], u[0], u[1], u[2], v[0], v[1], v[2]);
//	printf("pinter = %f %f %f [r=%f] triz = %f %f %f\n", pinter[0], pinter[1], pinter[2], r,
//	       tri0[2], tri0[1], tri0[2]);

	return 1;
    }



}
