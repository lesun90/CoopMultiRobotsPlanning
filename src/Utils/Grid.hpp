#ifndef MP_GRID_HPP_
#define MP_GRID_HPP_

#include <vector>

namespace MP
{
    class Grid
    {
    public:
	Grid(void)
	{
	    m_ndims  = 0;
	    m_cvol   = 0;
	    m_ncells = 0;
	}

	virtual ~Grid(void)
	{
	}

	virtual void Setup2D(const int dimsX, const int dimsY,
			     const double minX, const double minY,
			     const double maxX, const double maxY)
	{
	    const int dims[2] =
		{
		    dimsX, dimsY
		};
	    const double min[2] =
		{
		    minX, minY
		};
	    const double max[2] =
		{
		    maxX, maxY
		};
	    Setup(2, dims, min, max);
	}

	virtual void Setup3D(const int dimsX, const int dimsY, const int dimsZ,
			     const double minX, const double minY, const double minZ,
			     const double maxX, const double maxY, const double maxZ)
	{
	    const int dims[] =
		{
		    dimsX, dimsY, dimsZ
		};
	    const double min[] =
		{
		    minX, minY, minZ
		};
	    const double max[] =
		{
		    maxX, maxY, maxZ
		};
	    Setup(3, dims, min, max);
	}

	virtual void Setup(const int     ndims,
			   const int     dims[],
			   const double  min[],
			   const double  max[]);




	int GetNrCells(void) const
	{
	    return m_ncells;
	}

	int GetNrDims(void) const
	{
	    return m_dims.size();
	}

	const int* GetDims(void) const
	{
	    return &(m_dims[0]);
	}

	const double* GetMin(void) const
	{
	    return &(m_min[0]);
	}

	const double* GetMax(void) const
	{
	    return &(m_max[0]);
	}

	const double* GetUnits(void) const
	{
	    return &(m_units[0]);
	}

	double GetCellVolume(void) const
	{
	    return m_cvol;
	}

	virtual int GetCellIdFromCoords(const int coords[]) const;

	virtual void GetCoordsFromCellId(const int id, int coords[]) const;

	virtual void GetCoords(const double p[], int coords[]) const;

	virtual int GetCellId(const double p[]) const;

	virtual void GetCellFromCoords(const int coords[],
				       double    min[],
				       double    max[]) const;

	virtual void GetCellFromCoords(const int coords[],
				       double    bbox[]) const
	{
	    GetCellFromCoords(coords, &bbox[0], &bbox[m_ndims]);
	}

	virtual void GetCellFromId(const int id, double min[], double max[]) const;
	virtual void GetCellFromId(const int id, double bbox[]) const
	{
	    GetCellFromId(id, &bbox[0], &bbox[m_ndims]);
	}


	virtual void GetCellCenterFromCoords(const int coords[], double c[]) const;
	virtual void GetCellCenterFromId(const int id, double c[]) const;


	virtual bool IsPointInside(const double p[]) const;

	virtual bool IsPointInsideCell(const int coords[], const double p[]) const;

	virtual void GetNeighs(const int id, const int coords[], std::vector<int> *neighs);

    protected:
	static int GetCoord(const double x,
			    const double min,
			    const double max,
			    const double unit,
			    const int    ndims)
	{
	    const int c =  ((int) ((x - min) / unit));

	    return c < 0 ? 0 : (c > (ndims - 1) ? (ndims - 1) : c);
	}

	int                 m_ndims;
	std::vector<int>    m_dims;
	std::vector<double> m_min;
	std::vector<double> m_max;
	std::vector<double> m_units;
	double              m_cvol;
	int                 m_ncells;
    };
}

#endif
