#ifndef MP_SHEWCHUK_TRIANGLE_HPP_
#define MP_SHEWCHUK_TRIANGLE_HPP_

#include <vector>
#include <cstdio>
#include <cmath>

namespace MP
{
    void TriangulatePolygonWithHoles2D(const bool   useConformingDelaunay,
				       const double angleConstraint,
				       const double areaConstraint,
				       const int    totalNrVertices,
				       const double verticesPolyAndHoles[],
				       const int    nrVerticesPerContour[],
				       const int    nrHoles,
				       const double ptsInsideHoles[],
				       std::vector<double> * const triangleVertices,
				       std::vector<int>    * const triangleIndices,
				       std::vector<int>    * const triangleNeighbors);


    static inline
    void TriangulatePolygonWithNoHoles2D(const bool   useConformingDelaunay,
					 const double angleConstraint,
					 const double areaConstraint,
					 const int    totalNrVertices,
					 const double verticesPoly[],
					 std::vector<double> * const triangleVertices,
					 std::vector<int>    * const triangleIndices,
					 std::vector<int>    * const triangleNeighbors)
    {
	int nrVerticesPerContour = totalNrVertices;

	TriangulatePolygonWithHoles2D(useConformingDelaunay,
				      angleConstraint,
				      areaConstraint,
				      totalNrVertices, verticesPoly,
				      &nrVerticesPerContour, 0, NULL,
				      triangleVertices,
				      triangleIndices,
				      triangleNeighbors);
    }
}

#endif
