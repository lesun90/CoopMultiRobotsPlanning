#include "Utils/Maze.hpp"
#include "Utils/DisjointSet.hpp"
#include "Utils/Misc.hpp"

namespace MP
{
    void Maze::GenerateKruskal(const int dimsx, const int dimsy)
    {
	m_blocked.clear();
	m_empty.clear();
	
	DisjointSet                     dset;
	std::vector<DisjointSet::Elem*> cells;
	std::vector<Border>             walls;
	Border                          wall;
	int                             nrBlocked = 0;
	
	for(int x = 0; x < dimsx; ++x)
	    for(int y = 0; y < dimsy; ++y)
	    {
		if(x > 0)
		{
		    wall.m_cids[0] = y * dimsx + x;
		    wall.m_cids[1] = wall.m_cids[0] - 1;
		    walls.push_back(wall);
		}
		if(y > 0)
		{
		    wall.m_cids[0] = y * dimsx + x;
		    wall.m_cids[1] = wall.m_cids[0] - dimsx;
		    walls.push_back(wall);
		}
		cells.push_back(dset.Make());
	    }

	PermuteItems<Border>(&walls, walls.size());
	
	for(int i = 0; i < (int) (walls.size()); ++i)
	{
	    wall = walls[i];
	    if(dset.Same(cells[wall.m_cids[0]], cells[wall.m_cids[1]]) == false)
	    {
		m_empty.push_back(wall);
		dset.Join(cells[wall.m_cids[0]], cells[wall.m_cids[1]]);
	    }
	    else
		m_blocked.push_back(wall);
	}

	DeleteItems<DisjointSet::Elem*>(&cells);
    }

    void Maze::KeepBlocked(const double perc)
    {
	const int nuse = m_blocked.size() * perc;
	for(int i = m_blocked.size() - 1; i >= nuse; --i)
	{
	    m_empty.push_back(m_blocked.back());
	    m_blocked.pop_back();
	}
    }
    
    void Maze::GetWall(const Grid * const   grid, 
		       const Border         wall, 
		       const double         width,
		       double               min[2], 
		       double               max[2]) const
    {
	const double *gmin     = grid->GetMin();
	const int    *dims     = grid->GetDims();
	const double *units    = grid->GetUnits();
	const int     coords[] = {wall.m_cids[0] % dims[0], wall.m_cids[0] / dims[0]};
	const int     which    = (wall.m_cids[0] == (wall.m_cids[1] + 1)) ? 0 : 1;
	const double  lkeep    = 0.99;
	
	min[which]     = gmin[which]     + coords[which] * units[which] - 0.5 * width;
	min[1 - which] = gmin[1 - which] + coords[1 - which] * units[1 - which] + 0.5 * (1 - lkeep) * units[1 - which];
	max[which]     = min[which] + width;
	max[1 - which] = min[1 - which] + lkeep * units[1 - which];
    }

    void Maze::AddBlockedWalls(const Grid * const   grid, 
			       const double         width,
			       std::vector<Polygon2D*> * const polys) const
    {
	double min[2], max[2];
	Polygon2D *poly;
	
	for(int i = 0; i < m_blocked.size(); ++i)
	{
	    GetWall(grid, m_blocked[i], width, min, max);
	    poly = new Polygon2D();
	    poly->m_vertices.resize(8);
	    AABoxAsPolygon2D(min, max, &(poly->m_vertices[0]));
	    polys->push_back(poly);
	}
    }
    
}




