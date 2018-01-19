#include "Utils/Curve.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/GDraw.hpp"

namespace MP
{

    void Curve::ToPolygon(const int n, const double thick, Polygon2D * const polygon)
    {
	std::vector<double> *poly    = &(polygon->m_vertices);
	std::vector<double>  skel;

	const int    dim  = GetDim();
	const double dur  = GetDuration();
	
	skel.resize(n * dim);
	GetFirstPoint(&skel[0]);
	for(int i = 1; i < n; ++i)
	    GetPointAtTime(dur * (i + 0.0) / (n - 1), &skel[i * dim]);

//skel to polygon
	double v[2], u[2], p[2], line1[4], line2[4], x, y, mua, mub;

//first point -- chain 1	
	v[0] = skel[dim + 0] - skel[0];
	v[1] = skel[dim + 1] - skel[1];
	Algebra2D::VecNormal(v, u);
	Algebra2D::VecUnit(u, u);
	x = skel[0] + 0.5 * thick * u[0];
	y = skel[1] + 0.5 * thick * u[1];
	poly->push_back(x);
	poly->push_back(y);
	if(dim == 3)
	    polygon->m_heights.push_back(skel[2]);
	
	line1[0] = x;
	line1[1] = y;
	line1[2] = x + v[0];
	line1[3] = y + v[1];
	

	for(int i = 1; i < n - 1; ++i)
	{
	    v[0] = skel[dim * i + dim] - skel[dim * i];
	    v[1] = skel[dim * i + dim + 1] - skel[dim * i + 1];
	    Algebra2D::VecNormal(v, u);
	    Algebra2D::VecUnit(u, u);
	    line2[0] = skel[dim * i    ] + 0.5 * thick * u[0];
	    line2[1] = skel[dim * i + 1] + 0.5 * thick * u[1];
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
	    
	    if(dim == 3)
		polygon->m_heights.push_back(skel[dim * i + 2]);
	    

	    line1[0] = line2[0];
	    line1[1] = line2[1];
	    line1[2] = line2[2];
	    line1[3] = line2[3];
	    
	    
	}

//last point chain 1
	v[0] = skel[dim * (n - 1)]     - skel[dim * (n - 2)];
	v[1] = skel[dim * (n - 1) + 1] - skel[dim * (n - 2) + 1];
	Algebra2D::VecNormal(v, u);
	Algebra2D::VecUnit(u, u);
	x = skel[dim * (n - 1)] + 0.5 * thick * u[0];
	y = skel[dim * (n - 1) + 1] + 0.5 * thick * u[1];
	poly->push_back(x);
	poly->push_back(y);
	if(dim == 3)
	    polygon->m_heights.push_back(skel[dim * (n - 1) + 2]);

//chain 2
	for(int i = n - 1; i >= 0; --i)
	{
	    v[0] = (*poly)[2 * i] - skel[dim * i];
	    v[1] = (*poly)[2 * i + 1] - skel[dim * i + 1];
	    poly->push_back(skel[dim * i] - v[0]);
	    poly->push_back(skel[dim * i + 1] - v[1]);
	    if(dim == 3)
		polygon->m_heights.push_back(skel[dim * i + 2]);
	    
	}

	polygon->MakeCCW();
	
    }
    
    void Curve::Draw(const int nrPts) const
    {
	const int    dim   = GetDim();
	const double dur   = GetDuration();
	
	std::vector<double> pts;
	pts.resize(2 * dim);
	
	GetFirstPoint(&pts[0]);
	for(int i = 1; i <= nrPts; ++i)
	{
	    GetPointAtTime(dur * (i + 0.0) / nrPts, &pts[dim]);
	    if(dim == 2)
		GDrawSegment2D(&pts[0], &pts[dim]);
	    else if(dim >= 3)
		GDrawSegment3D(&pts[0], &pts[dim]);
	    CopyArray<double>(&pts[0], dim, &pts[dim]);
	}
    }	
    
////TransformedCurve
    void TransformedCurve::Transform(double p[]) const
    {
	p[0] *= m_scaling[0];
	p[1] *= m_scaling[1];
	if(m_curve->GetDim() == 3)
	{
	    p[2] *= m_scaling[2];
	    Algebra3D::TransRotMultPoint(m_TR, p, p);
	}
	else
	    Algebra2D::TransRotMultPoint(m_TR, p, p);
    }
    
 
/////////////////////MultiCurve
    MultiCurve::~MultiCurve(void)
    {
	const int n = m_curves.size();
	for(int i = 0; i < n; ++i)
	    delete m_curves[i];
	m_curves.clear();
    }
    
    Curve* MultiCurve::Clone(void) const
    {
	MultiCurve *mc = new MultiCurve();
	const int   n  = m_curves.size();
	
	for(int i = 0; i < n; ++i)
	    mc->AddCurve(m_curves[i]->Clone());
	return mc;
    }
    
    void MultiCurve::AddCurve(Curve * const curve) 
    {
	m_curves.push_back(curve);
	m_duration += curve->GetDuration();
    }

    void MultiCurve::RemoveCurve(const int i)
    {
	const int n = m_curves.size();
	
	if(i >= 0 && i < n)
	{
	    Curve *c = m_curves[i];
	    for(int j = i + 1; j < n; ++j)
		m_curves[j - 1] = m_curves[j];
	    m_curves.pop_back();
	    m_duration -= c->GetDuration();

	    delete c;
	}
    }
    
        
    void MultiCurve::GetPointAtTime(const double t, double p[]) const
    {
	const int n = m_curves.size();
	double    d = 0;
	
	for(int i = 0; i < n; ++i)
	{
	    if((d + m_curves[i]->GetDuration()) >= t)
	    {
		m_curves[i]->GetPointAtTime(t - d, p);
		return;
	    }
	    d += m_curves[i]->GetDuration();
	}   
    }

    Curve* MultiCurve::Subcurve(const double tstart, const double tend) const
    {
	const int n      = m_curves.size();
	int       istart = -1;
	int       iend   = -1;
	double    d      = 0;
	double    t1     = 0;
	double    t2     = 0;
	
	
	for(int i = 0; i < n; ++i)
	{
	    if(istart < 0 && (d + m_curves[i]->GetDuration()) >= tstart)
	    {
		istart = i;
		t1     = d;
	    }
	    
	    if(istart >= 0 && (d + m_curves[i]->GetDuration()) >= tend)
	    {
		iend = i;
		t2   = d;
		break;
	    }
	    
	    d += m_curves[i]->GetDuration();
	}   

	if(istart < 0 || iend < 0)
	    return NULL;
	if(istart == iend)
	    return m_curves[istart]->Subcurve(tstart - t1, tend - t1);
	
	MultiCurve *mc = new MultiCurve();
	mc->AddCurve(m_curves[istart]->Subcurve(tstart - t1, m_curves[istart]->GetDuration()));
	for(int i = istart + 1; i < iend; ++i)
	    mc->AddCurve(m_curves[i]->Clone());
	mc->AddCurve(m_curves[iend]->Subcurve(0, tend - t2));
	return mc;
	
    }

    void MultiCurve::Draw(const int nrPts) const
    {
	const int n = m_curves.size();
	for(int i = 0; i < n; ++i)
	    m_curves[i]->Draw(nrPts);
    }

//SegmentCurve

    void SegmentCurve::GetPointAtTime(const double t, double p[]) const
    {
	const double s = t / m_duration;
	for(int i = 0; i < m_dim; ++i)
	    p[i] = (1 - s) * m_pts[i] + s * m_pts[m_dim + i];
    }

    Curve* SegmentCurve::Subcurve(const double tstart, const double tend) const
    {
	SegmentCurve *sc = new SegmentCurve();
	std::vector<double> tmp;
	tmp.resize(GetDim());
	
	sc->SetDuration(tend - tstart);
	sc->SetDim(GetDim());
	
	GetPointAtTime(tstart, &tmp[0]);
	sc->SetFirstPoint(&tmp[0]);
	GetPointAtTime(tend, &tmp[0]);
	sc->SetLastPoint(&tmp[0]);
	
	return sc;
    }
    

    void SegmentCurve::Draw(const int nrPts) const
    {
	if(m_dim == 2)
	    GDrawSegment2D(&m_pts[0], &m_pts[m_dim]);
	else if(m_dim >= 3)
	    GDrawSegment3D(&m_pts[0], &m_pts[m_dim]);
    }

//ArcCurve
    void ArcCurve::GetPointAtTime(const double t, double p[]) const
    {
	p[0] = m_center[0] + cos(t + m_start) * m_radius;
	p[1] = m_center[1] + sin(t + m_start) * m_radius;
    }
    
    Curve* ArcCurve::Subcurve(const double tstart, const double tend) const
    {
	ArcCurve *ac = new ArcCurve();
	ac->SetStart(m_start + tstart);
	ac->SetDuration(tend - tstart);
	ac->SetRadius(m_radius);
	ac->SetCenter(m_center);

	return ac;
    }

//Misc
    void PolygonFromCurves(const Curve * const curve1,
			   const Curve * const curve2,
			   const int n,
			   Polygon2D * const poly)
    {
	poly->m_vertices.clear();
	double p[3];
	
	for(int i = 0; i < n; ++i)
	{
	    curve1->GetPointAtTime(curve1->GetDuration() * (i + 0.0) / (n - 1), p);
	    poly->m_vertices.push_back(p[0]);
	    poly->m_vertices.push_back(p[1]);
	    poly->m_heights.push_back(p[2]);
	}

	for(int i = n - 1; i >= 0; --i)
	{
	    curve2->GetPointAtTime(curve2->GetDuration() * (i + 0.0) / (n - 1), p);
	    poly->m_vertices.push_back(p[0]);
	    poly->m_vertices.push_back(p[1]);
	    poly->m_heights.push_back(p[2]);
	}

	poly->MakeCCW();
    }
    

}
