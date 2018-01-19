#include "Utils/Grid.hpp"
#include <cstdlib>

namespace MP
{
    void Grid::Setup(const int     ndims,
		     const int     dims[],
		     const double  min[],
		     const double  max[])
    {
	m_ndims = ndims;	    
	m_dims.resize(ndims);
	m_min.resize(ndims);
	m_max.resize(ndims);
	m_units.resize(ndims);
	
	m_cvol   = 1;
	m_ncells = 1;
	for(int i = 0; i < ndims; ++i)
	{
	    m_dims[i] = dims[i];
	    m_min[i]  = min[i];
	    m_max[i]  = max[i];
	    m_units[i]= (m_max[i] - m_min[i]) / m_dims[i];
	    m_cvol   *= m_units[i];
	    m_ncells *= m_dims[i];
	}
    }
    
    
    int Grid::GetCellIdFromCoords(const int coords[]) const
    {
	int  factor = 1;
	int  id     = 0;;
	
	for(int i = 0; i < m_ndims; ++i)
	{
	    id     += coords[i] * factor;
	    factor *= m_dims[i];
	}
	return id;
    }
    
    void Grid::GetCoordsFromCellId(const int id, int coords[]) const
    {
	int factor = id;
	
	for(int i = 0; i < m_ndims; ++i)
	{
	    coords[i] = factor % m_dims[i];
	    factor   /= m_dims[i];
	}
    }
    
    void Grid::GetCoords(const double p[], int coords[]) const
    {
	for(int i = 0; i < m_ndims; ++i)
	    coords[i] = GetCoord(p[i], m_min[i], m_max[i], m_units[i], m_dims[i]);	
    }
    
    int Grid::GetCellId(const double p[]) const
    {
	int factor = 1;
	int id     = 0;;
	
	for(int i = 0; i < m_ndims; ++i)
	{
	    id     += factor * GetCoord(p[i], m_min[i], m_max[i], m_units[i], m_dims[i]);
	    factor *= m_dims[i];
	}
	return id;	
    }
    
    void Grid::GetCellFromCoords(const int coords[], 
				 double    min[], 
				 double    max[]) const
    {
	for(int i = 0; i < m_ndims; ++i)
	{
	    min[i] = m_min[i] + m_units[i] * coords[i];
	    max[i] = min[i]   + m_units[i];
	}
    }
    
    
    void Grid::GetCellFromId(const int id, double min[], double max[]) const
    {
	int factor  = id;
	int coord_i = 0;
	
	for(int i = 0; i < m_ndims; ++i)
	{
	    coord_i   = factor % m_dims[i];
	    factor   /= m_dims[i];    
	    
	    min[i]    = m_min[i] + m_units[i] * coord_i;
	    max[i]    = min[i]   + m_units[i];
	}
	
    }
    
    void Grid::GetCellCenterFromCoords(const int coords[], double c[]) const
    {
	for(int i = 0; i < m_ndims; ++i)
	    c[i] =  m_min[i] + (0.5 + coords[i]) * m_units[i];

    }
    
    void Grid::GetCellCenterFromId(const int id, double c[]) const
    {
	int factor  = id;
	int coord_i = 0;
	
	for(int i = 0; i < m_ndims; ++i)
	{
	    coord_i   = factor % m_dims[i];
	    factor   /= m_dims[i];    

	    c[i] = m_min[i] + (0.5 + coord_i) * m_units[i];
	}
    }
    
	
    bool Grid::IsPointInside(const double p[]) const
    {
	for(int i = 0; i < m_ndims; ++i)
	    if(p[i] < m_min[i] || p[i] > m_max[i])
		return false;
	return true;	
    }
    
    bool Grid::IsPointInsideCell(const int coords[], const double p[]) const
    {
	double min;
	
	for(int i = 0; i < m_ndims; ++i)
	{
	    min = m_min[i] + m_units[i] * coords[i];
	    if(p[i] < min || p[i] > (min + m_units[i]))
		return false;
	}
	return true;
    }

    
    void Grid::GetNeighs(const int id, const int coords[], std::vector<int> *neighs)
    {
	const int neigh3d[] =
	    {
		-1,  0, 0,
		+1,  0, 0,
		0, -1, 0,
		0, +1, 0,
		-1, -1, 0,
		-1, +1, 0,
		+1, -1, 0,
		+1, +1, 0,
		
		-1,  0, -1,
		+1,  0, -1,
		0, -1, -1,
		0, +1, -1,
		-1, -1, -1,
		-1, +1, -1,
		+1, -1, -1,
		+1, +1, -1,
		
		-1,  0, +1,
		+1,  0, +1,
		0, -1, +1,
		0, +1, +1,
		-1, -1, +1,
		-1, +1, +1,
		+1, -1, +1,
		+1, +1, +1,
		
		0, 0, -1,
		0, 0, +1
	    };
 
	
	
	
	const int n      = m_dims.size();
	int       factor = 1;
	
	if(n == 2)
	{
	    for(int i = 0; i < n; ++i)
	    {
		if(coords[i] + 1 < m_dims[i])  
		{
		    neighs->push_back(id + factor);	
		    //diagonal: hack for 2d grid
		    /*    if(i == 1 && (coords[0] - 1) >= 0)
			neighs->push_back(id + factor - 1);
		    if(i == 1 && (coords[0] + 1) < m_dims[0])
			neighs->push_back(id + factor + 1);	    	   
		    */
		}
		
		if(coords[i] - 1 >= 0)         
		{	    
		    neighs->push_back(id - factor);
		    //diagonal: hack for 2d grid
		    /*if(i == 1 && (coords[0] - 1) >= 0)
			neighs->push_back(id - factor - 1);
		    if(i == 1 && (coords[0] + 1) < m_dims[0])
			neighs->push_back(id - factor + 1);	  
		    */
		}
		
		factor *= m_dims[i];	
	    }    
	}
	else if(n == 3)
	{
	    for(int i = 0; i < 26; ++i)	    
		if((coords[0] + neigh3d[3 * i + 0]) < m_dims[0] &&
		   (coords[0] + neigh3d[3 * i + 0])  >= 0       &&
		   (coords[1] + neigh3d[3 * i + 1]) < m_dims[1] &&
		   (coords[1] + neigh3d[3 * i + 1])  >= 0       &&
		   (coords[2] + neigh3d[3 * i + 2]) < m_dims[2] &&
		   (coords[2] + neigh3d[3 * i + 2])  >= 0)
		    neighs->push_back(id + 
				      neigh3d[3 * i + 0] + 
				      neigh3d[3 * i + 1] * m_dims[0] +
				      neigh3d[3 * i + 2] * m_dims[0] * m_dims[1]);	
	}
    }
}
