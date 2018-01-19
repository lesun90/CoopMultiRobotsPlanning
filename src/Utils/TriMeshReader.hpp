#ifndef MP_TRIMESH_READER_HPP_
#define MP_TRIMESH_READER_HPP_

#include "Utils/TriMesh.hpp"
#include <cstdio>

namespace MP
{
    bool TriMeshReader(const char fname[], TriMesh * const tmesh);
    bool OFFTriMeshReader(const char fname[], TriMesh * const tmesh);
    bool PLYTriMeshReader(const char fname[], TriMesh * const tmesh);
    bool OBJTriMeshReader(const char fname[], TriMesh * const tmesh);
    bool STLAsciiTriMeshReader(const char fname[], TriMesh * const tmesh);
    bool STLBinaryTriMeshReader(const char fname[], TriMesh * const tmesh);
    bool HeightMapTriMeshReader(const char fname[], TriMesh * const tmesh);
    bool StandardTriMeshReader(FILE *out, TriMesh * const tmesh);


    bool STLAsciiTriMeshWriter(const char fname[], TriMesh * const tmesh);
    bool STLBinaryTriMeshWriter(const char fname[], TriMesh * const tmesh);
    bool StandardTriMeshWriter(FILE *in, TriMesh * const tmesh);

    void GetTrimeshOutline(TriMesh * const tmesh, TriMesh * outline);


}

#endif
