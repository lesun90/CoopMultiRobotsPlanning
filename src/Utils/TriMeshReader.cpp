#include "Utils/TriMeshReader.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <vector>
#include <algorithm>
#include "Utils/Algebra3D.hpp"
#include "Utils/PrintMsg.hpp"

namespace MP
{
  void GetTrimeshOutline(TriMesh * const tmesh, TriMesh * outline)
  {
    const int nf = tmesh->GetNrFaces();
    for(int i = 0; i < nf; ++i)
    {
      for(int t = 0; t < nf; ++t)
      {
        if (i == t)
        {
          continue;
        }

        const int   fnv1 = tmesh->GetFace(i)->m_vids.size();
        const int   fnv2 = tmesh->GetFace(t)->m_vids.size();

        double vector1[3];
        double vector2[3];
        double vector3[3];
        double vector4[3];
        double vector5[3];

        vector1[0] = tmesh->GetVertex(tmesh->GetFace(i)->m_vids[1])[0] - tmesh->GetVertex(tmesh->GetFace(i)->m_vids[0])[0];
        vector1[1] = tmesh->GetVertex(tmesh->GetFace(i)->m_vids[1])[1] - tmesh->GetVertex(tmesh->GetFace(i)->m_vids[0])[1];
        vector1[2] = tmesh->GetVertex(tmesh->GetFace(i)->m_vids[1])[2] - tmesh->GetVertex(tmesh->GetFace(i)->m_vids[0])[2];

        vector2[0] = tmesh->GetVertex(tmesh->GetFace(i)->m_vids[2])[0] - tmesh->GetVertex(tmesh->GetFace(i)->m_vids[0])[0];
        vector2[1] = tmesh->GetVertex(tmesh->GetFace(i)->m_vids[2])[1] - tmesh->GetVertex(tmesh->GetFace(i)->m_vids[0])[1];
        vector2[2] = tmesh->GetVertex(tmesh->GetFace(i)->m_vids[2])[2] - tmesh->GetVertex(tmesh->GetFace(i)->m_vids[0])[2];

        vector3[0] = tmesh->GetVertex(tmesh->GetFace(t)->m_vids[1])[0] - tmesh->GetVertex(tmesh->GetFace(t)->m_vids[0])[0];
        vector3[1] = tmesh->GetVertex(tmesh->GetFace(t)->m_vids[1])[1] - tmesh->GetVertex(tmesh->GetFace(t)->m_vids[0])[1];
        vector3[2] = tmesh->GetVertex(tmesh->GetFace(t)->m_vids[1])[2] - tmesh->GetVertex(tmesh->GetFace(t)->m_vids[0])[2];

        vector4[0] = tmesh->GetVertex(tmesh->GetFace(t)->m_vids[2])[0] - tmesh->GetVertex(tmesh->GetFace(t)->m_vids[0])[0];
        vector4[1] = tmesh->GetVertex(tmesh->GetFace(t)->m_vids[2])[1] - tmesh->GetVertex(tmesh->GetFace(t)->m_vids[0])[1];
        vector4[2] = tmesh->GetVertex(tmesh->GetFace(t)->m_vids[2])[2] - tmesh->GetVertex(tmesh->GetFace(t)->m_vids[0])[2];

        Algebra3D::VecCrossProduct(vector1, vector2, vector5);
        if ((Algebra3D::VecDotProduct(vector3, vector5) == 0)
        && (Algebra3D::VecDotProduct(vector4, vector5) == 0))
        {
          continue;
        }
        bool found = false;
        const double *vv1;
        const double *vv2;
        for(int j = 0; j < fnv1 && found == false; ++j)
        {
          const double *v1;
          const double *v2;
          if (j < fnv1-1)
          {
            v1 = tmesh->GetVertex(tmesh->GetFace(i)->m_vids[j]);
            v2 = tmesh->GetVertex(tmesh->GetFace(i)->m_vids[j+1]);
          }
          else
          {
            v1 = tmesh->GetVertex(tmesh->GetFace(i)->m_vids.back());
            v2 = tmesh->GetVertex(tmesh->GetFace(i)->m_vids[0]);
          }
          for(int k = 0; k < fnv2; ++k)
          {
            const double *v3;
            const double *v4;
            if (k < fnv2-1)
            {
              v3 = tmesh->GetVertex(tmesh->GetFace(t)->m_vids[k]);
              v4 = tmesh->GetVertex(tmesh->GetFace(t)->m_vids[k+1]);
            }
            else
            {
              v3 = tmesh->GetVertex(tmesh->GetFace(t)->m_vids.back());
              v4 = tmesh->GetVertex(tmesh->GetFace(t)->m_vids[0]);
            }
            if ((v1[0] == v3[0]) && (v1[1] == v3[1]) && (v1[2] == v3[2]) &&
            (v2[0] == v4[0]) && (v2[1] == v4[1]) && (v2[2] == v4[2]))
            {
              vv1 = v1;
              vv2 = v2;
              found = true;
              break;
            }
            if ((v1[0] == v4[0]) && (v1[1] == v4[1]) && (v1[2] == v4[2]) &&
            (v2[0] == v3[0]) && (v2[1] == v3[1]) && (v2[2] == v3[2]))
            {
              vv1 = v1;
              vv2 = v2;
              found = true;
              break;
            }
          }
        }
        if (found == true)
        {
          outline->AddQuad(vv1,vv1,vv2,vv2);
        }
      }
    }
  }

    bool TriMeshReader(const char fname[], TriMesh * const tmesh)
    {
	const int length = strlen(fname);

	if(length >= 4 &&
	   (strcmp(&fname[length - 4], ".ply") == 0 ||
	    strcmp(&fname[length - 4], ".PLY") == 0))
	    return PLYTriMeshReader(fname, tmesh);
	else if(length >= 4 &&
	   (strcmp(&fname[length - 4], ".off") == 0 ||
	    strcmp(&fname[length - 4], ".OFF") == 0))
	    return OFFTriMeshReader(fname, tmesh);
	else if(length >= 4 &&
	   (strcmp(&fname[length - 4], ".obj") == 0 ||
	    strcmp(&fname[length - 4], ".OBJ") == 0))
	    return OBJTriMeshReader(fname, tmesh);
	else if(length >= 4 &&
	   (strcmp(&fname[length - 4], ".stl") == 0))
	    return STLBinaryTriMeshReader(fname, tmesh);
	else if(length >= 6 &&
	   (strcmp(&fname[length - 6], ".tmesh") == 0))
	{
	    FILE *in = fopen(fname, "r");
	    if(in)
	    {
		const bool res = StandardTriMeshReader(in, tmesh);
		fclose(in);
		return res;
	    }
	    return false;
	}
	else if(length >= 3 &&
	   (strcmp(&fname[length - 3], ".HM") == 0 ||
	    strcmp(&fname[length - 3], ".hm") == 0))
	    return HeightMapTriMeshReader(fname, tmesh);
	else
	    fprintf(stderr, "don't know how to read <%s>\n", fname);
	return false;
    }

    bool StandardTriMeshWriter(FILE *out, TriMesh * const tmesh)
    {
	const int nv  = tmesh->GetNrVertices();
	const int nf  = tmesh->GetNrFaces();
	const int nt  = tmesh->GetNrTexCoords();


	fprintf(out, "%d %d\n", nv, nf);
	for(int i = 0; i < nv; ++i)
	{
	    const double *v = tmesh->GetVertex(i);
	    fprintf(out, "%f %f %f\n", v[0], v[1], v[2]);
	}
	for(int i = 0; i < nf; ++i)
	{
	    const TriMesh::Face* f = tmesh->GetFace(i);
	    fprintf(out, "%d ", (int) f->m_vids.size());
	    for(int j = 0; j < f->m_vids.size(); ++j)
		fprintf(out, "%d ", f->m_vids[j]);
	    fprintf(out, "\n");
	    fprintf(out, "%d %d %d\n", f->m_gid, f->m_mid, f->m_tid);
	}

	fprintf(out, "%d\n", nt);
	if(nt > 0)
	{
	    for(int i = 0; i < nt; ++i)
	    {
		TriMesh::TexCoord tx = tmesh->GetTexCoord(i);
		fprintf(out, "%f %f\n", tx.m_u, tx.m_v);
	    }
	    for(int i = 0; i < nf; ++i)
	    {
		const TriMesh::Face* f = tmesh->GetFace(i);
		fprintf(out, "%d ", (int) f->m_vtids.size());
		for(int j = 0; j < f->m_vtids.size(); ++j)
		    fprintf(out, "%d ", f->m_vtids[j]);
		fprintf(out, "\n");
	    }
	}

	if(tmesh->GetMainTexture())
	    fprintf(out, "MainTexture %s\n", tmesh->GetMainTexture()->GetFileName());

	if(tmesh->GetNrTextures() > 0)
	{
	    fprintf(out, "Textures %d\n", tmesh->GetNrTextures());
	    for(int i = 0; i < tmesh->GetNrTextures(); ++i)
		fprintf(out, "%s\n", tmesh->GetTexture(i)->GetFileName());
	}


	if(tmesh->GetMainMaterial())
	{
	    fprintf(out, "MainMaterial\n");
	    tmesh->GetMainMaterial()->Print(out);
	}

	if(tmesh->GetNrMaterials() > 0)
	{
	    fprintf(out, "Materials %d\n", tmesh->GetNrMaterials());
	    for(int i = 0; i < tmesh->GetNrMaterials(); ++i)
		tmesh->GetMaterial(i)->Print(out);
	}

	return true;
    }


    bool StandardTriMeshReader(FILE *in, TriMesh * const tmesh)
    {

	int nv, nf, nt, n, ival;
	const int nvOffset = tmesh->GetNrVertices();
	const int nfOffset = tmesh->GetNrFaces();
	const int ntOffset = tmesh->GetNrTexCoords();
	char keyword[300];


	double v[3];


	if(fscanf(in, "%d %d\n", &nv, &nf) != 2)
	    OnInputError(printf("expecting nv nf\n"));

	for(int i = 0; i < nv; ++i)
	{
	    if(fscanf(in, "%lf %lf %lf\n", &v[0], &v[1], &v[2]) != 3)
		OnInputError(printf("expecting vx vy vz for vertex %d/%d\n", i, nv));
	    tmesh->AddVertex(v);
	}
	for(int i = 0; i < nf; ++i)
	{
	    TriMesh::Face* f = new TriMesh::Face();
	    if(fscanf(in, "%d ", &n) != 1)
		OnInputError(printf("expecting nr vertices for face %d/%d\n", i, nf));
	    f->m_vids.resize(n);
	    f->m_needsUpdate = true;
	    for(int j = 0; j < n; ++j)
	    {
		if(fscanf(in, "%d", &(f->m_vids[j])) != 1)
		    OnInputError(printf("expecting %d-th value out of %d\n", j, n));
		f->m_vids[j] += nvOffset;
	    }

	    tmesh->AddFace(f);
	    if(fscanf(in, "%d %d %d", &(f->m_gid), &(f->m_mid), &(f->m_tid)) != 3)
		OnInputError(printf("expecting gid mid tid for face %d/%d\n", i, nf));
	}

	if(fscanf(in, "%d", &nt) == 1 && nt > 0)
	{
	    for(int i = 0; i < nt; ++i)
	    {
		if(fscanf(in, "%lf %lf", &v[0], &v[1]) != 2)
		    OnInputError(printf("expecting tvx tvy for texture %d/%d\n", i, nt));
		tmesh->AddTexCoord(v[0], v[1]);
	    }

	    for(int i = 0; i < nf; ++i)
	    {
		TriMesh::Face *f = const_cast<TriMesh::Face*>(tmesh->GetFace(nfOffset + i));
		if(fscanf(in, "%d", &n) != 1)
		    OnInputError(printf("expecting number of textures for face %d/%d\n", i, nf));
		for(int j = 0; j < n; ++j)
		{
		    if(fscanf(in, "%d", &ival) != 1)
			OnInputError(printf("expecting %d/%d value\n", j, n));
		    f->m_vtids.push_back(ival + ntOffset);
		}
	    }
	    tmesh->UseManualTexCoords(true);

	}

	GTexture *gtex;
	GMaterial *gmat;

	while(fscanf(in, "%s", keyword) == 1)
	{

	    if(strcmp(keyword, "MainTexture") == 0)
	    {
		if(fscanf(in, "%s", keyword) != 1)
		    OnInputError(printf("expecting filename after MainTexture\n"));
		gtex = new GTexture();
		gtex->SetFileName(keyword);
		tmesh->SetMainTexture(gtex);
	    }
	    else if(strcmp(keyword, "Textures") == 0)
	    {
		if(fscanf(in, "%d", &n) != 1)
		    OnInputError(printf("expecting number of textures after Textures\n"));
		for(int i = 0; i < n; ++i)
		{
		    if(fscanf(in, "%s", keyword) != 1)
			OnInputError(printf("expecting filename for texture %d/%d\n", i, n));
		    gtex = new GTexture();
		    gtex->SetFileName(keyword);
		    tmesh->AddTexture(gtex);
		}
	    }
	    else if(strcmp(keyword, "MainMaterial") == 0)
	    {
		gmat = new GMaterial();
		gmat->Read(in);
		tmesh->SetMainMaterial(gmat);
	    }
	    else if(strcmp(keyword, "Materials") == 0)
	    {
		if(fscanf(in, "%d", &n) != 1)
		    OnInputError(printf("expecting number of materials after Materials\n"));
		for(int i = 0; i < n; ++i)
		{
		    gmat = new GMaterial();
		    gmat->Read(in);
		    tmesh->AddMaterial(gmat);
		}
	    }

	}



	return true;
    }

    bool OFFTriMeshReader(const char fname[], TriMesh * const tmesh)
    {
	int i;

	// Open file
	FILE *fp = fopen(fname, "r");
	if (!fp)
	{
	    fprintf(stderr, "Unable to open file %s\n", fname);
	    return false;
	}

	const int nvold = tmesh->GetNrVertices();


	// Read file
	int nverts = 0;
	int nfaces = 0;
	int nfacesRead = 0;
	int nvFace     = 0;

	int nedges = 0;
	int line_count = 0;
	char buffer[1024];
	double v[3];

	while (fgets(buffer, 1023, fp))
	{
	    // Increment line counter
	    line_count++;

	    // Skip white space
	    char *bufferp = buffer;
	    while (isspace(*bufferp)) bufferp++;

	    // Skip blank lines and comments
	    if (*bufferp == '#') continue;
	    if (*bufferp == '\0') continue;

	    // Check section
	    if (nverts == 0)
	    {
		// Read header
		if (!strstr(bufferp, "OFF"))
		{
		    // Read mesh counts
		    if ((sscanf(bufferp, "%d%d%d", &nverts, &nfaces, &nedges) != 3) || (nverts == 0))
		    {
			fprintf(stderr, "Syntax error reading header on line %d in file %s\n",
				line_count, fname);
			fclose(fp);
			return false;
		    }
		}
	    }
	    else if (tmesh->GetNrVertices() < nverts)
	    {
		// Read vertex coordinates
		if (sscanf(bufferp, "%lf%lf%lf", &(v[0]), &(v[1]), &(v[2])) != 3)
		{
		    fprintf(stderr, "Syntax error with vertex coordinates on line %d in file %s\n",
			    line_count, fname);
		    fclose(fp);
		    return false;
		}
		tmesh->AddVertex(v);
	    }
	    else if (nfacesRead < nfaces)
	    {
		++nfacesRead;


		// Read number of vertices in face
		bufferp = strtok(bufferp, " \t");
		if (bufferp) nvFace = atoi(bufferp);
		else
		{
		    fprintf(stderr, "Syntax error with face on line %d in file %s\n", line_count, fname);
		    fclose(fp);
		    return false;
		}

		TriMesh::Face *face = new TriMesh::Face();
		face->m_vids.resize(nvFace);

		// Read vertex indices for face
		for (i = 0; i < nvFace; i++)
		{
		    bufferp = strtok(NULL, " \t");
		    if (bufferp)
			face->m_vids[i] = nvold + atoi(bufferp);
		    else
		    {
			fprintf(stderr, "Syntax error with face on line %d in file %s\n", line_count, fname);
			fclose(fp);
			delete face;
			return false;
		    }
		}
		std::reverse(face->m_vids.begin(), face->m_vids.end());

		tmesh->AddFace(face);
	    }
	    else
	    {
		// Should never get here
		fprintf(stderr, "Found extra text starting at line %d in file %s\n", line_count, fname);
		break;
	    }
	}

	// Check whether read all faces
	if (nfaces != nfacesRead)
	{
	    fprintf(stderr, "Expected %d faces, but read only %d faces in file %s\n",
		    nfaces, nfacesRead, fname);
	    return false;
	}

	// Close file
	fclose(fp);

	return true;
    }

    bool PLYTriMeshReader(const char fname[], TriMesh * const tmesh)
    {
	FILE *fp = fopen(fname, "r");
	if(!fp)
	{
	    fprintf(stderr, "Unable to open file %s\n", fname);
	    return false;
	}

	int  nread = 0;
	bool ascii = false;
	int  nvert = 0;
	int  nfaces= 0;

	char buffer[1024];
	char *ignore = NULL;

	while((nread = fscanf(fp, "%s", buffer)) == 1)
	{
	    if(strcmp(buffer, "comment") == 0)
		ignore = fgets(buffer, 1023, fp);

	    else if(strcmp(buffer, "format") == 0)
	    {
		if((nread = fscanf(fp, "%s", buffer)) != 1)
		{
		    fprintf(stderr, "expecting string after format\n");
		    return false;
		}
		if(strcmp(buffer, "ascii") == 0)
		    ascii = true;
		ignore = fgets(buffer, 1023, fp);
	    }
	    else if(strcmp(buffer, "element") == 0)
	    {
		if((nread = fscanf(fp, "%s", buffer)) != 1)
		{
		    fprintf(stderr, "expecting string after element\n");
		    return false;
		}
		if(strcmp(buffer, "vertex") == 0)
		{
		    if(fscanf(fp, "%d", &nvert) != 1)
		    {
			fprintf(stderr, "expecting number of vertices\n");
			return false;
		    }
		}
		else if(strcmp(buffer, "face") == 0)
		{
		    if(fscanf(fp, "%d", &nfaces) != 1)
		    {
			fprintf(stderr, "expecting number of faces\n");
			return false;
		    }
		}
		ignore = fgets(buffer, 1023, fp);
	    }
	    else if(strcmp(buffer, "end_header") == 0)
		break;
	    else
		ignore = fgets(buffer, 1023, fp);
	}


	printf("nread = %d ascii = %d nvert = %d nfaces = %d\n", nread, ascii, nvert, nfaces);
	    const int nvold = tmesh->GetNrVertices();

	if(ascii)
	{
	    double    v[3];

	    for(int i = 0; i < nvert; ++i)
	    {
		if(fscanf(fp, "%lf %lf %lf", &v[0], &v[1], &v[2]) != 3)
		{
		    fprintf(stderr, "error reading vertex %d\n", i);
		    return false;
		}
		ignore = fgets(buffer, 1023, fp);
		tmesh->AddVertex(v);
	    }

	    TriMesh::Face   *face   = NULL;
	    int              nvface = 0;
	    for(int i = 0; i < nfaces; ++i)
	    {
		if(fscanf(fp, "%d", &nvface) != 1)
		{
		    fprintf(stderr, "error reading nr. vertices for face %d\n", i);
		    return false;
		}

		face = new TriMesh::Face();
		face->m_vids.resize(nvface);
		for(int j = 0; j < nvface; ++j)
		    if(fscanf(fp, "%d", &(face->m_vids[j])) != 1)
		    {
			fprintf(stderr, "error reading vertex %d of face %d\n", j, i);
			delete face;
			return false;
		    }
		    else
			face->m_vids[j] += nvold;
		tmesh->AddFace(face);
	    }
	}
	else
	{
	    fprintf(stderr, "too tired to read in binary mode\n");
	    return false;
	}


	fclose(fp);
	return true;
    }

    void OBJTriMeshReaderCleanup(FILE *fp, std::vector<char *> *groups, std::vector<char *> *mats)
    {
	const int ng = groups->size();
	for(int i = 0; i < ng; ++i)
	    if((*groups)[i])
		free((*groups)[i]);
	const int nm = mats->size();
	for(int i = 0; i < nm; ++i)
	    if((*mats)[i])
		free((*mats)[i]);
	if(fp)
	    fclose(fp);
    }

    void OBJExtractNumbers(char s[], std::vector<int> * const vals)
    {
	int backslash = 0;
	int start     = 0;
	int count     = 0;
	int val;

	for(int i = 0; ; ++i)
	{
	    if(s[i] == '/' || s[i] == '\n' || s[i] == '\0')
	    {
		if(sscanf(&s[start], "%d", &val) == 1)
		    vals->push_back(val);
		else
		    vals->push_back(-1);
		start = i + 1;
	    }

	    if(s[i] == '\0')
		return;
	}
    }


    bool OBJTriMeshReader(const char fname[], TriMesh * const tmesh)
    {
	FILE *fp = fopen(fname, "r");
	if(!fp)
	{
	    fprintf(stderr, "Unable to open file %s\n", fname);
	    return false;
	}

	const int           nvold = tmesh->GetNrVertices();
	const int           nfold = tmesh->GetNrFaces();

	char                buffer[1024];
	double              v[3];
	int                 normCount = 0;
	std::vector<char *> groups;
	std::vector<char *> mats;
	GMaterial          *gmat = NULL;
	std::vector<int>    vals;
	char               *ignore = NULL;


	while(fscanf(fp, "%s", buffer) == 1)
	{
	    if(strcmp(buffer, "mtllib") == 0)
	    {
		if(fscanf(fp, "%s", buffer) != 1)
		{
		    fprintf(stderr, "expecting file name after mtllib\n");
		    OBJTriMeshReaderCleanup(fp, &groups, &mats);
		    return false;
		}
		FILE *fpmat = fopen(buffer, "r");
		if(!fpmat)
		{
		    fprintf(stderr, "could not read material file <%s>\n", buffer);
		    OBJTriMeshReaderCleanup(fp, &groups, &mats);
		    return false;
		}
		while(fscanf(fpmat, "%s", buffer) == 1)
		{
		    if(strcmp(buffer, "newmtl") == 0)
		    {
			if(fscanf(fpmat, "%s", buffer) != 1)
			{
			    fprintf(stderr, "could not read material name\n");
			    OBJTriMeshReaderCleanup(fp, &groups, &mats);
			    return false;
			}
			printf("material:<%s>\n", buffer);

			mats.push_back(strdup(buffer));
			gmat = new GMaterial();
			tmesh->AddMaterial(gmat);
			ignore = fgets(buffer, 1023, fpmat);
		    }
		    else if(strcmp(buffer, "Ka") == 0)
		    {
			if(fscanf(fpmat, "%lf %lf %lf", &v[0], &v[1], &v[2]) != 3)
			{
			    fprintf(stderr, "expecting three values after Ka\n");
			    OBJTriMeshReaderCleanup(fp, &groups, &mats);
			    return false;
			}
			gmat->SetAmbient(v[0], v[1], v[2]);
			ignore = fgets(buffer, 1023, fpmat);
		    }
		    else if(strcmp(buffer, "Kd") == 0)
		    {
			if(fscanf(fpmat, "%lf %lf %lf", &v[0], &v[1], &v[2]) != 3)
			{
			    fprintf(stderr, "expecting three values after Kd\n");
			    OBJTriMeshReaderCleanup(fp, &groups, &mats);
			    return false;
			}
			gmat->SetDiffuse(v[0], v[1], v[2]);
			ignore = fgets(buffer, 1023, fpmat);
		    }
		    else if(strcmp(buffer, "Ks") == 0)
		    {
			if(fscanf(fpmat, "%lf %lf %lf", &v[0], &v[1], &v[2]) != 3)
			{
			    fprintf(stderr, "expecting three values after Ks\n");
			    OBJTriMeshReaderCleanup(fp, &groups, &mats);
			    return false;
			}
			gmat->SetSpecular(v[0], v[1], v[2]);
			ignore = fgets(buffer, 1023, fpmat);
		    }
		    else if(strcmp(buffer, "Ns") == 0)
		    {
			if(fscanf(fpmat, "%lf", &v[0]) != 1)
			{
			    fprintf(stderr, "expecting one value after Ns\n");
			    OBJTriMeshReaderCleanup(fp, &groups, &mats);
			    return false;
			}
			gmat->SetShininess(v[0] * 128.0 / 1000);
			ignore = fgets(buffer, 1023, fpmat);
		    }
		    else
			ignore = fgets(buffer, 1023, fpmat);
		}
		fclose(fpmat);
	    }
	    else if(strcmp(buffer, "v") == 0)
	    {
		if(fscanf(fp, "%lf %lf %lf", &v[0], &v[1], &v[2]) != 3)
		{
		    fprintf(stderr, "expecting 3 vertex coordinates after v\n");
		    OBJTriMeshReaderCleanup(fp, &groups, &mats);
		    return false;
		}
		tmesh->AddVertex(v[0], v[1], v[2]);
		ignore = fgets(buffer, 1023, fp);
	    }
	    else if(strcmp(buffer, "vt") == 0)
	    {
		if(fscanf(fp, "%lf %lf", &v[0], &v[1]) != 2)
		{
		    fprintf(stderr, "expecting 2 texture coordinates after vt\n");
		    OBJTriMeshReaderCleanup(fp, &groups, &mats);
		    return false;
		}
		tmesh->AddTexCoord(v[0], v[1]);
		ignore = fgets(buffer, 1023, fp);
	    }
	    /*    else if(strcmp(buffer, "vn") == 0)
	    {
		if(fscanf(fp, "%lf %lf %lf", &v[0], &v[1], &v[2]) != 3)
		{
		    fprintf(stderr, "expecting 3 vertex coordinates after v\n");
		    OBJTriMeshReaderCleanup(fp, &groups, &mats);
		    return false;
		}
		Algebra3D::VecUnit(v, v);
		tmesh->SetVertexNormal(nvold + normCount, v[0], v[1], v[2]);
		++normCount;
		fgets(buffer, 1023, fp);
		}*/
	    else if(strcmp(buffer, "g") == 0)
	    {
		ignore = fgets(buffer, 1023, fp);
		int istart = 0;
		while(isspace(buffer[istart]))
		    ++istart;
		int iend = istart;
		while(buffer[iend] != '\0' && buffer[iend] != '\n')
		    ++iend;
		buffer[iend] = '\0';

//		printf("group name:<%s>\n", &buffer[istart]);

		int found = -1;
		if(buffer[istart] != '\0')
		{
		    for(int i = 0; i < (int) groups.size() && found < 0; ++i)
			if(strcmp(&buffer[istart], groups[i]) == 0)
			    found = i;
		}

		if(found >= 0)
		    tmesh->SetCurrentGroup(found);
		else
		{
		    //	    printf("adding group: <%s>\n", &buffer[istart]);

		    groups.push_back(strdup(&buffer[istart]));
		    tmesh->AddGroup();
		}
	    }
	    else if(strcmp(buffer, "group") == 0)
	    {
		ignore = fgets(buffer, 1023, fp);
		int istart = 0;
		while(isspace(buffer[istart]))
		    ++istart;
		int iend = istart;
		while(buffer[iend] != '\0' && buffer[iend] != '\n')
		    ++iend;
		buffer[iend] = '\0';

		int found = -1;
		if(buffer[istart] != '\0')
		{
		    for(int i = 0; i < (int) groups.size() && found < 0; ++i)
			if(strcmp(&buffer[istart], groups[i]) == 0)
			    found = i;
		}

		if(found >= 0)
		    tmesh->SetCurrentGroup(found);
	    }
	    else if(strcmp(buffer, "usemtl") == 0)
	    {
		if(fscanf(fp, "%s", buffer) != 1)
		    OnInputError(printf("expecting string after usemtl\n"));


		int found = -1;
		for(int i = 0; i < (int) mats.size() && found < 0; ++i)
		    if(strcmp(mats[i], buffer) == 0)
			found = i;
		if(found >= 0)
		    tmesh->SetCurrentMaterial(found);
		else
		{
		    fprintf(stderr, "material <%s> not found\n", buffer);
		    OBJTriMeshReaderCleanup(fp, &groups, &mats);
		    return false;
		}

		ignore = fgets(buffer, 1023, fp);
	    }
	    else if(strcmp(buffer, "f") == 0)
	    {
		ignore = fgets(buffer, 1023, fp);

		TriMesh::Face *face  = new TriMesh::Face();
		char          *token = strtok(buffer, " ");

		while(token && token[0] != '\n')
		{
		    vals.clear();
		    OBJExtractNumbers(token, &vals);
		    const int nvals = vals.size();

		    if(nvals > 0 && vals[0] >= 1)
			face->m_vids.push_back(vals[0] - 1);
		    if(nvals > 1 && vals[1] >= 1)
			face->m_vtids.push_back(vals[1] - 1);
		    token = strtok(NULL, " ");
		}
		tmesh->AddFace(face);

	    }
	    else
		ignore = fgets(buffer, 1023, fp);
	}

	return true;
    }

    bool HeightMapTriMeshReader(const char fname[], TriMesh * const tmesh)
    {
	FILE *in = fopen(fname, "r");
	if(!in)
	{
	    fprintf(stderr, "Unable to open file <%s>\n", fname);
	    return false;
	}

	int nrDimsX, nrDimsY;
	std::vector<double> vertices;
	if(fscanf(in, "%d %d", &nrDimsX, &nrDimsY) != 2)
	{
	    fprintf(stderr, "expecting nrDimsX nrDimsY\n");
	    return false;
	}

	vertices.resize(3 * nrDimsX * nrDimsY);
	for(int i = 0; i < 3 * nrDimsX * nrDimsY; ++i)
	    if(fscanf(in, "%lf", &vertices[i]) != 1)
	    {
		fprintf(stderr, "could not read element %d\n", i);
		return false;
	    }

	tmesh->AddHeightMap(nrDimsX, nrDimsY, &vertices[0]);

	return true;


    }

    bool STLAsciiTriMeshReader(const char fname[], TriMesh * const tmesh)
    {
	FILE          *in = fopen(fname, "r");
	char           keyword[300];
       	double         norm[3];
	double         v[3];
	TriMesh::Face *f;
	char          *ignore = NULL;

	if(!in)
	{
	    fprintf(stderr, "Unable to open file <%s>\n", fname);
	    return false;
	}

	keyword[79] = '\0';
	ignore = fgets(keyword, 80, in);
	if(strstr(keyword, "solid") == NULL)
	{
	    fprintf(stderr, "<%s> is not an ascii stl file: missing keyword solid\n", fname);
	    fclose(in);
	    fprintf(stderr, "opening it in binary mode\n");
	    return STLBinaryTriMeshReader(fname, tmesh);
	}


	while(fscanf(in, "%s", keyword) == 1)
	{
	    if(strcmp(keyword, "facet") == 0)
	    {
		f = new TriMesh::Face();
	     	if(fscanf(in, "%s %lf %lf %lf %s", keyword, &norm[0], &norm[1], &norm[2], keyword) != 5)
		    OnInputError(printf("expecting facet info\n"));

		while(fscanf(in, "%s", keyword) == 1)
		{
		    if(strcmp(keyword, "vertex") == 0)
		    {
			if(fscanf(in, "%lf %lf %lf", &v[0], &v[1], &v[2]) != 3)
			    OnInputError(printf("expecting vx vy vz\n"));
			f->m_vids.push_back(tmesh->GetNrVertices());
			tmesh->AddVertex(v);
		    }
		    else if(strcmp(keyword, "endfacet") == 0)
			break;
		}
		if(f->m_vids.size() == 3)
		{
		    tmesh->AddTriangle(&f->m_vids[0]);
		    delete f;
		}
		else
		    tmesh->AddFace(f);
	    }
	}

	fclose(in);

	return true;
    }


    bool STLAsciiTriMeshWriter(const char fname[], TriMesh * const tmesh)
    {
	FILE *out = fopen(fname, "w");


	if(!out)
	{
	    fprintf(stderr, "Unable to open file <%s>\n", fname);
	    return false;
	}

	fprintf(out, "solid name\n");


	const int nf = tmesh->GetNrFaces();
	for(int i = 0; i < nf; ++i)
	{
	    const TriMesh::Face *f = tmesh->GetFace(i);

	    fprintf(out, "facet normal %10.6f %10.6f %10.6f\n",
		    f->m_normal[0], f->m_normal[1], f->m_normal[2]);
	    fprintf(out, "   outer loop\n");
	    const int nv = f->m_vids.size();
	    for(int j = 0; j < nv; ++j)
	    {
		const double *v = tmesh->GetVertex(f->m_vids[j]);
		fprintf(out, "  vertex %10.6f %10.6f %10.6f\n", v[0], v[1], v[2]);
	    }
	    fprintf(out, "   endloop\n");
	    fprintf(out, "endfacet\n");
	}

	fprintf(out, "endsolid name\n");

	fclose(out);

	return true;
    }


    bool STLBinaryTriMeshReader(const char fname[], TriMesh * const tmesh)
    {
	char           header[300];
	unsigned int   n;
	float          data[12];
	double         ddata[12];
	unsigned short attr;


	FILE *in = fopen(fname, "rb");
	if(!in)
	{
	    fprintf(stderr, "Unable to open file <%s>\n", fname);
	    return false;
	}

	if(fread(header, sizeof(char), 80, in) != 80)
	{
	    fprintf(stderr, "unable to read header from stl file <%s>\n", fname);
	    fclose(in);
	    return false;
	}

	header[80] = '\0';
	if(strstr(header, "solid") != NULL)
	{
	    fprintf(stderr, "<%s> is not a binary stl file: it has the keyword solid\n", fname);
	    fclose(in);
	    fprintf(stderr, "opening it in ascii mode\n");
	    return STLAsciiTriMeshReader(fname, tmesh);
	}


	if(fread(&n, sizeof(unsigned int), 1, in) != 1)
	{
	    fprintf(stderr, "unable to read nr. triangles in stl file <%s>\n", fname);
	    fclose(in);
	    return false;
	}

	for(int i = 0; i < n; ++i)
	{
	    if(fread(data, sizeof(float), 12, in) != 12)
	    {
		fprintf(stderr, "unable to read triangle %d file <%s>\n", i, fname);
		fclose(in);
		return false;
	    }
	    for(int j = 0; j < 12; ++j)
		ddata[j] = (double) data[j];

	    tmesh->AddTriangle(&ddata[3]);

	    if(fread(&attr, sizeof(unsigned short), 1, in) != 1)
	    {
		fprintf(stderr, "unable to read attribute byte <%s>\n", fname);
		fclose(in);
		return false;
	    }
	}

	fclose(in);

	return true;
    }


    bool STLBinaryTriMeshWriter(const char fname[], TriMesh * const tmesh)
    {
	FILE *out = fopen(fname, "wb");


	if(!out)
	{
	    fprintf(stderr, "Unable to open file <%s>\n", fname);
	    return false;
	}

	char header[81];
	unsigned short attr = 0;

	for(int i = 0; i < 80; ++i)
	    header[i] = 'a';
	fwrite(header, sizeof(char), 80, out);

	const unsigned int nf = (unsigned int) tmesh->GetNrFaces();

	fwrite(&nf, sizeof(unsigned int), 1, out);

	for(int i = 0; i < nf; ++i)
	{
	    const TriMesh::Face *f = tmesh->GetFace(i);
	    const float float_n[3] =
		{
		    (float) f->m_normal[0], (float) f->m_normal[1], (float) f->m_normal[2]
		};

	    fwrite(float_n, sizeof(float), 3, out);
	    const int nv = f->m_vids.size();
	    for(int j = 0; j < nv; ++j)
	    {
		const double *v          = tmesh->GetVertex(f->m_vids[j]);
		const float   float_v[3] = { (float) v[0], (float) v[1], (float) v[2]};

		fwrite(float_v, sizeof(float), 3, out);
	    }

	    fwrite(&attr, sizeof(unsigned short), 1, out);
	}

	fclose(out);

	return true;
    }

}
