#include "Utils/HeightField.hpp"
#include <cstdio>

namespace MP
{
    void HeightField::SetHeights(EvalFn eval)
    {
	const int xdim = m_grid.GetDims()[0];
	const int ydim = m_grid.GetDims()[1];
	int coords[2];
	
	m_heights.resize(xdim * ydim);
	for(coords[0] = 0; coords[0] < xdim; ++(coords[0]))
	    for(coords[1] = 0; coords[1] < ydim; ++(coords[1]))
	   	m_heights[m_grid.GetCellIdFromCoords(coords)] = eval(this, coords[0], coords[1]);
    }
    
}







