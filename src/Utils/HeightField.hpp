#ifndef MP_HEIGHT_FIELD_HPP_
#define MP_HEIGHT_FIELD_HPP_

#include "Utils/Grid.hpp"
#include <cstdlib>

namespace MP
{

    class HeightField
    {
    public:
	HeightField(void)
	{
	    m_evalFnData = NULL;
	}
	
	virtual ~HeightField(void)
	{
	}

	typedef double (*EvalFn)(HeightField * const hf, const int i, const int j);
	

	double GetHeightAtCell(const int id) const
	{
	    return m_heights[id];
	}
	
	double GetHeightAtCell(const int i, const int j) const
	{
	    const int coords[2] = {i, j};
	    return GetHeightAtCell(m_grid.GetCellIdFromCoords(coords));
	}
	
	double GetHeightAtPoint(const double x, const double y) const
	{
	    const double p[2] = {x, y};
	    return GetHeightAtCell(m_grid.GetCellId(p));
	    
	}

	void SetHeightAtCell(const int id, const double h)
	{
	    m_heights[id] = h;
	    
	}
	
	
	void SetHeights(EvalFn eval);

	
	Grid                m_grid;
	std::vector<double> m_heights;
	void               *m_evalFnData;
    };

   
}

#endif





    
    

