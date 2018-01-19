#include "Scenes/Scene.hpp"
#include "Utils/DisjointSet.hpp"
#include "Utils/HeightField.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/Geometry.hpp"

namespace MP
{
    void Scene::Extrude(const int    start,
			const int    end,
			const double zmin,
			const double zmax,
			const bool   useGround)
    {
	TriMesh *tm = useGround ? &m_ground : &m_tmesh;

	const int last = m_polys.size() - 1;
	const int usen = end < last ? end : last;

	for(int i = start; i <= usen; ++i)
	    tm->AddExtrudedPolygon(m_polys[i], 0, RandomUniformReal(zmin, zmax));
    }


    void AddToScenePolygonOnALine::Add(Scene * const scene)
    {
      const double  totalArea = scene->m_grid.GetCellVolume() * scene->m_grid.GetNrCells();
      double        obstArea  = 0.0;
      bool          tooClose  = false;
      bool          tooFar  = false;

      bool accept = false;
      double        x         = 0;
      double        y         = 0;
      double        r         = 0;
      int           i         = 0;
      int           nv        = 0;
      const double *min       = scene->m_grid.GetMin();
      const double *max       = scene->m_grid.GetMax();
      int           count     = 0;
      Polygon2D    *poly      = NULL;
      double        pmin1[2];
      double        pmin2[2];
      double        d;

      double dist = sqrt(((endx-startx)*(endx-startx)) + ((endy-starty)*(endy-starty)));
      double dx = (endx-startx)/d;
      double dy = (endy-starty)/d;


      for(int i = scene->m_polys.size() - 1; i >= 0; --i)
      obstArea += scene->m_polys[i]->GetArea();

      const int nstart = scene->m_polys.size();


      while(!((m_maxNrPolys > 0 && count >= m_maxNrPolys) || (m_maxNrPolys < 0 && obstArea >= m_perc * totalArea)))
      {
        if(i % 100 == 0)
        printf("generating polygon %d [curr coverage = %f]\n", i, obstArea / totalArea);
        ++i;

        poly = new Polygon2D();

        do
        {
          do
          {
            x = startx + dx * RandomUniformReal(0,dist);
            y = starty + dy * RandomUniformReal(0,dist);
            r = RandomUniformReal(m_rmin, m_rmax);
          }
          while((r + x) > max[0] || (x - r) < min[0] || (y - r) < min[1] || (y + r) > max[1]);


          nv = RandomUniformInteger(m_minNrVertices, m_maxNrVertices);

          const double coin = RandomUniformReal();
          if(coin < 0.5)
          {
            poly->m_vertices.resize(2 * nv);
            CircleAsPolygon2D(x, y, r, nv, &(poly->m_vertices[0]));
          }
          else
          {
            poly->m_vertices.clear();
            GenerateArcAsPolygon2D(x, y, r, RandomUniformReal(0, 2 * M_PI),
            RandomUniformReal(180/nv, 350/nv) * Constants::DEG2RAD, nv, r, &(poly->m_vertices));
          }

          poly->MakeCCW();

          accept = false;
          tooClose = false;
          tooFar = false;

          for(int j = 0; j < scene->m_polys.size() && !tooClose; ++j)
          {

            d = DistSquaredPolygons2D(scene->m_polys[j]->m_vertices.size() / 2,
            &(scene->m_polys[j]->m_vertices[0]),
            poly->m_vertices.size() / 2, &(poly->m_vertices[0]), pmin1, pmin2);

            tooClose =
            ((d < m_dsep * m_dsep) || (m_dsepCriterion2 > 0 && j >= m_startCriterion2 && d < m_dsepCriterion2 * m_dsepCriterion2)) ||
            IsPolygonInsidePolygon2D(scene->m_polys[j]->m_vertices.size() / 2,
            &(scene->m_polys[j]->m_vertices[0]),
            poly->m_vertices.size() / 2, &(poly->m_vertices[0])) ||
            IsPolygonInsidePolygon2D(poly->m_vertices.size() / 2, &(poly->m_vertices[0]),
            scene->m_polys[j]->m_vertices.size() / 2,
            &(scene->m_polys[j]->m_vertices[0]));
accept = tooClose;
          }

        }
        while(accept);

        ++count;
        scene->m_polys.push_back(poly);
        obstArea += poly->GetArea();

      }
}
    void AddToSceneRandomPolygons::Add(Scene * const scene)
    {
	const double  totalArea = scene->m_grid.GetCellVolume() * scene->m_grid.GetNrCells();
	double        obstArea  = 0.0;
	bool          tooClose  = false;
	double        x         = 0;
	double        y         = 0;
	double        r         = 0;
	int           i         = 0;
	int           nv        = 0;
	const double *min       = scene->m_grid.GetMin();
	const double *max       = scene->m_grid.GetMax();
	int           count     = 0;
	Polygon2D    *poly      = NULL;
	double        pmin1[2];
	double        pmin2[2];
	double        d;


	for(int i = scene->m_polys.size() - 1; i >= 0; --i)
	    obstArea += scene->m_polys[i]->GetArea();

	const int nstart = scene->m_polys.size();


	while(!((m_maxNrPolys > 0 && count >= m_maxNrPolys) || (m_maxNrPolys < 0 && obstArea >= m_perc * totalArea)))
	{
	    if(i % 100 == 0)
		printf("generating polygon %d [curr coverage = %f]\n", i, obstArea / totalArea);
	    ++i;

	    poly = new Polygon2D();

	    do
	    {
		do
		{
		    x = RandomUniformReal(min[0], max[0]);
		    y = RandomUniformReal(min[1], max[1]);
		    r = RandomUniformReal(m_rmin, m_rmax);
		}
		while((r + x) > max[0] || (x - r) < min[0] || (y - r) < min[1] || (y + r) > max[1]);


		nv = RandomUniformInteger(m_minNrVertices, m_maxNrVertices);

		const double coin = RandomUniformReal();
		if(coin < 0.5)
		{
		    poly->m_vertices.resize(2 * nv);
		    CircleAsPolygon2D(x, y, r, nv, &(poly->m_vertices[0]));
		}
		else
		{
		    poly->m_vertices.clear();
		    GenerateArcAsPolygon2D(x, y, r, RandomUniformReal(0, 2 * M_PI),
					   RandomUniformReal(180/nv, 350/nv) * Constants::DEG2RAD, nv, r, &(poly->m_vertices));
		}


		poly->MakeCCW();


		tooClose = false;
		for(int j = 0; j < scene->m_polys.size() && !tooClose; ++j)
		{

		    d = DistSquaredPolygons2D(scene->m_polys[j]->m_vertices.size() / 2,
					      &(scene->m_polys[j]->m_vertices[0]),
					      poly->m_vertices.size() / 2, &(poly->m_vertices[0]), pmin1, pmin2);


		    tooClose =
			((d < m_dsep * m_dsep) || (m_dsepCriterion2 > 0 && j >= m_startCriterion2 && d < m_dsepCriterion2 * m_dsepCriterion2)) ||
			IsPolygonInsidePolygon2D(scene->m_polys[j]->m_vertices.size() / 2,
						 &(scene->m_polys[j]->m_vertices[0]),
						 poly->m_vertices.size() / 2, &(poly->m_vertices[0])) ||
			IsPolygonInsidePolygon2D(poly->m_vertices.size() / 2, &(poly->m_vertices[0]),
						scene->m_polys[j]->m_vertices.size() / 2,
						 &(scene->m_polys[j]->m_vertices[0]));
		}

	    }
	    while(tooClose);

	    ++count;
	    scene->m_polys.push_back(poly);
	    obstArea += poly->GetArea();
	}

	printf("generated %d polygons [curr coverage = %f]\n", i, obstArea / totalArea);
    }

    void AddToSceneMaze::AddFencesWhenEmpty(Scene * const scene,
					    const double    prob,
					    const int       minNrDims,
					    const int       maxNrDims)
    {
	double gap, u1, u2, min[3], max[3];
	const int n = m_empty.size();
	int a, b;
	for(int i = 0; i < n; ++i)
	{
	    const int  dims1 = 2;//RandomUniformInteger(minNrDims, maxNrDims);
	    const int  dims2 = RandomUniformInteger(minNrDims, maxNrDims);
	    const int *cids  = m_empty[i].m_cids;

	    if(fabs(cids[0] - cids[1]) == 1) //fence YZ
	    {
		if(RandomUniformReal() < 0.1)
		    return;

		scene->m_grid.GetCellFromId(cids[1], min, max);

		scene->m_tmesh.SetCurrentMaterial(0);

		//add horizontal bars along y axis
		gap = m_zmax / dims1;
		for(int i = 0; i <= dims1; i += 1)
		    scene->m_tmesh.AddBox(max[0] - 0.5 * m_width, min[1], i * gap - 0.5 * m_width,
					  max[0] + 0.5 * m_width, max[1], i * gap + 0.5 * m_width);

		//add vertical bars along z axis
		gap = (max[1] - min[1]) / dims2;
		for(int i = 1; i <= dims2;  i += 1)
		  scene->m_tmesh.AddBox(max[0] - 0.5 * m_width, min[1] + i * gap - 0.5 * m_width, 0,
					max[0] + 0.5 * m_width, min[1] + i * gap + 0.5 * m_width, m_zmax);


		scene->m_tmesh.SetCurrentMaterial(1);

		u1 = m_zmax / dims1;
		u2 = (max[1] - min[1]) / dims2;
		for(a = 0; a < dims1; ++a)
		    for(b = 0; b < dims2; ++b)
			if(RandomUniformReal() < prob)
			    scene->m_tmesh.AddBox(max[0] - 0.5 * m_width,
						  min[1] + b * u2 + 0.5 * m_width,
						  a * u1 + 0.5 * m_width,
						  max[0] + 0.5 * m_width,
						  min[1] + (b + 1) * u2 - 0.5 * m_width,
						  (a + 1) * u1 - 0.5 * m_width);
	    }
	    else //fence XZ
	    {
		scene->m_grid.GetCellFromId(cids[1], min, max);

		scene->m_tmesh.SetCurrentMaterial(0);

		//add horizontal bars along x axis
		gap = m_zmax / dims1;
		for(int i = 0; i <= dims1; i += 1)
		    scene->m_tmesh.AddBox(min[0], max[1] - 0.5 * m_width, i * gap - 0.5 * m_width,
					  max[0], max[1] + 0.5 * m_width, i * gap + 0.5 * m_width);

		//add vertical bars along z axis
		gap = (max[0] - min[0]) / dims2;
		for(int i = 1; i < dims2;  i += 1)
		   scene->m_tmesh.AddBox(min[0] + i * gap - 0.5 * m_width, max[1] - 0.5 * m_width, 0,
					 min[0] + i * gap + 0.5 * m_width, max[1] + 0.5 * m_width, m_zmax);

		scene->m_tmesh.SetCurrentMaterial(1);


		u1 = m_zmax / dims1;
		u2 = (max[0] - min[0]) / dims2;
		for(a = 0; a < dims1; ++a)
		    for(b = 0; b < dims2; ++b)
			if(RandomUniformReal() < prob)
			    scene->m_tmesh.AddBox(min[0] + b * u2 + 0.5 * m_width,
						  max[1] - 0.5 * m_width,
						  a * u1 + 0.5 * m_width,
						  min[0] + (b + 1) * u2 - 0.5 * m_width,
						  max[1] + 0.5 * m_width,
						  (a + 1) * u1 - 0.5 * m_width);

	    }
	}
    }



    void AddToSceneMaze::AddSmallWallWhenEmpty(Scene * const scene,
					       const double    prob)
    {

//turn walls into polygons
	double r, min[2], max[2], c[2], TR[Algebra2D::TransRot_NR_ENTRIES];
	Polygon2D *poly;
	int k;


	const int n = m_empty.size();
	for(int i = 0; i < n; ++i)
	    if(RandomUniformReal() < prob)
	    {
		const int *cids = m_empty[i].m_cids;
		GetWall(scene, &m_empty[i], min, max);

		if(fabs(max[0] - min[0]) > fabs(max[1] - min[1])) //horizontal wall
		    k = 0;
		else
		    k = 1;

		{
		    r = RandomUniformReal(0.1, 0.8) * (max[k] - min[k]);
		    min[k] = 0.5 * (max[k] + min[k]) - 0.5 * r;
		    max[k] = min[k] + r;
		}
		c[0] = 0.5 * (min[0] + max[0]);
		c[1] = 0.5 * (min[1] + max[1]);
		Algebra2D::RotateAroundPointAsTransRot(RandomUniformReal(-M_PI, M_PI), c, TR);




		poly = new Polygon2D();
		poly->m_vertices.resize(8);
		AABoxAsPolygon2D(min, max, &(poly->m_vertices[0]));

		ApplyTransRotToPolygon2D(TR, 4, &(poly->m_vertices[0]), &(poly->m_vertices[0]));


		scene->m_polys.push_back(poly);
	    }
    }



    void AddToSceneMaze::Add(Scene * const scene)
    {
	DisjointSet                     dset;
	Wall                            wall;
	std::vector<DisjointSet::Elem*> cells;

	const int *dims = scene->m_grid.GetDims();

	for(int x = 0; x < dims[0]; ++x)
	    for(int y = 0; y < dims[1]; ++y)
	    {
		if(x > 0)
		{
		    wall.m_cids[0] = y * dims[0] + x;
		    wall.m_cids[1] = wall.m_cids[0] - 1;
		    m_walls.push_back(wall);
		}
		if(y > 0)
		{
		    wall.m_cids[0] = y * dims[0] + x;
		    wall.m_cids[1] = wall.m_cids[0] - dims[0];
		    m_walls.push_back(wall);
		}
		cells.push_back(dset.Make());
	    }
	PermuteItems<Wall>(&m_walls, m_walls.size());

	m_nkeep = 0;
	for(int i = 0; i < m_walls.size(); ++i)
	{
	    wall = m_walls[i];
	    if(dset.Same(cells[wall.m_cids[0]], cells[wall.m_cids[1]]) == false)
	    {
		m_empty.push_back(wall);
		dset.Join(cells[wall.m_cids[0]], cells[wall.m_cids[1]]);
	    }
	    else
		m_walls[m_nkeep++] = wall;
	}
	const int nuse = m_nkeep * m_keepPerc;
	for(int i = nuse; i < m_nkeep; ++i)
	    m_empty.push_back(m_walls[i]);

	m_nkeep = nuse;

	for(int i = 0; i < cells.size(); ++i)
	    delete cells[i];


//turn walls into polygons
	double min[2], max[2];
	Polygon2D *poly;

	for(int i = 0; i < m_nkeep; ++i)
	{
	    GetWall(scene, &m_walls[i], min, max);
	    poly = new Polygon2D();
	    poly->m_vertices.resize(8);
	    AABoxAsPolygon2D(min, max, &(poly->m_vertices[0]));
	    scene->m_polys.push_back(poly);
	}
    }

    void AddToSceneRadial::Add(Scene * const scene)
    {
	double x          = 0.5 * (scene->m_grid.GetMin()[0] + scene->m_grid.GetMax()[0]);
	double y          = 0.5 * (scene->m_grid.GetMin()[1] + scene->m_grid.GetMax()[1]);
	double thetaStart = 0;
	double thetaInc   = 0;
	double theta      = 0;
	double gap        = 0;
	double thetagap   = 0;
	double thetaprev  = MP::RandomUniformReal(-M_PI, M_PI);
	Polygon2D *poly;

	const double xrange = scene->m_grid.GetMax()[0] - scene->m_grid.GetMin()[0];
	const double yrange = scene->m_grid.GetMax()[1] - scene->m_grid.GetMin()[1];
	const double rmax   = 0.5 * (xrange > yrange ? yrange : xrange);


	for(double r = m_rmin; r <= rmax; r += m_rinc)
	{
	    gap         = MP::RandomUniformReal(m_minGap, m_maxGap);
	    thetagap    = gap / r;
	    theta       = (2 * M_PI - m_nrSegs * thetagap) / m_nrSegs;
	    thetaStart  = thetaprev + MP::RandomUniformReal(m_thetaMin, m_thetaMax);
	    thetaprev   = thetaStart;
	    thetaInc    = theta / (m_nrRays - 1);

	    for(int i = 0; i < m_nrSegs; ++i)
	    {
		poly       = new Polygon2D();
		GenerateArcAsPolygon2D(x, y, r, thetaStart, thetaInc, m_nrRays, m_thick, &(poly->m_vertices));
		scene->m_polys.push_back(poly);
		thetaStart += theta + thetagap;
	    }
	}
    }

    void AddArc::Add(Scene * const scene)
    {
      Polygon2D *poly;
      poly = new Polygon2D();
      double thetaInc = (m_thetaMax - m_thetaMin)/(m_nrRays - 1);
      poly       = new Polygon2D();
      GenerateArcAsPolygon2D(m_x, m_y, m_r, m_thetaMin, thetaInc, m_nrRays, m_thick, &(poly->m_vertices));
      scene->m_polys.push_back(poly);
    }


    void AddToSceneMaze::GetWall(Scene * const scene, Wall * const wall, double min[2], double max[2]) const
    {
	const double *gmin     = scene->m_grid.GetMin();
	const int    *dims     = scene->m_grid.GetDims();
	const double *units    = scene->m_grid.GetUnits();
	const int     coords[] = {wall->m_cids[0] % dims[0], wall->m_cids[0] / dims[0]};
	const int     which    = (wall->m_cids[0] == (wall->m_cids[1] + 1)) ? 0 : 1;
	const double  lkeep    = 0.9999;

	min[which]     = gmin[which]     + coords[which] * units[which] - 0.5 * m_width;
	min[1 - which] = gmin[1 - which] + coords[1 - which] * units[1 - which] + 0.5 * (1 - lkeep) * units[1 - which];
	max[which]     = min[which] + m_width;
	max[1 - which] = min[1 - which] + lkeep * units[1 - which];
    }

    void AddToSceneRamp::Add(Scene * const scene)
    {
	AddToScene::Add(scene);

	const bool asTri = true;
	const int  n     = scene->m_tmesh.GetNrVertices();

	scene->m_tmesh.AddVertex(-0.5 * m_top - m_left,  -0.5 * m_width, 0.0);
	scene->m_tmesh.AddVertex(-0.5 * m_top,           -0.5 * m_width, 0.0);
	scene->m_tmesh.AddVertex( 0.5 * m_top,           -0.5 * m_width, 0.0);
	scene->m_tmesh.AddVertex( 0.5 * m_top + m_right, -0.5 * m_width, 0.0);
	scene->m_tmesh.AddVertex( 0.5 * m_top,           -0.5 * m_width, m_height);
	scene->m_tmesh.AddVertex(-0.5 * m_top,           -0.5 * m_width, m_height);

	scene->m_tmesh.AddVertex(-0.5 * m_top - m_left,  0.5 * m_width, 0.0);
	scene->m_tmesh.AddVertex(-0.5 * m_top,           0.5 * m_width, 0.0);
	scene->m_tmesh.AddVertex( 0.5 * m_top,           0.5 * m_width, 0.0);
	scene->m_tmesh.AddVertex( 0.5 * m_top + m_right, 0.5 * m_width, 0.0);
	scene->m_tmesh.AddVertex( 0.5 * m_top,           0.5 * m_width, m_height);
	scene->m_tmesh.AddVertex(-0.5 * m_top,           0.5 * m_width, m_height);

	scene->m_tmesh.AddTriangle(n,     n + 1, n + 5);
	scene->m_tmesh.AddTriangle(n + 2, n + 3, n + 4);
	scene->m_tmesh.AddQuad(n + 1, n + 2, n + 4, n + 5, asTri);

	scene->m_tmesh.AddTriangle(n + 11, n + 7, n + 6);
	scene->m_tmesh.AddTriangle(n + 10, n + 9, n + 8);
	scene->m_tmesh.AddQuad(n + 11, n + 10, n + 8, n + 7, asTri);

	scene->m_tmesh.AddQuad(n + 0, n + 5, n + 11, n + 6, asTri);
	scene->m_tmesh.AddQuad(n + 5, n + 4, n + 10, n + 11, asTri);
	scene->m_tmesh.AddQuad(n + 4, n + 3, n + 9,  n + 10, asTri);

	scene->m_tmesh.AddQuad(n + 6, n + 9, n + 3, n + 0);

	const double min[2] = {-0.5 * m_top - m_left,  -0.5 * m_width};
	const double max[2] = { 0.5 * m_top + m_right,  0.5 * m_width};
	Polygon2D *poly = new Polygon2D();

	poly->m_vertices.resize(8);
	AABoxAsPolygon2D(min, max, &(poly->m_vertices[0]));
	scene->m_polys.push_back(poly);
    }

    void AddToSceneSpiral::Add(Scene * const scene)
    {
	AddToScene::Add(scene);

	double        t   = m_angleStart;
	double        x   = 0;
	double        y   = 0;
	const double *min = scene->m_grid.GetMin();
	const double *max = scene->m_grid.GetMax();


	std::vector<double> skel;
	Polygon2D          *poly = new Polygon2D();

	do
	{
	    x = m_coeffx * pow(t, m_open) * cos(t);
	    y = m_coeffy * pow(t, m_open) * sin(t);
	    skel.push_back(x);
	    skel.push_back(y);

	    t += m_angleInc;
	}
	while(x >= min[0] && x <= max[0] &&
	      y >= min[1] && y <= max[1]);

	FromSkeletonToPolygon2D(skel.size() / 2, &skel[0], m_width, &(poly->m_vertices));
	poly->MakeCCW();

	const int nv = poly->m_vertices.size() / 2;
	if(m_zinc > 0)
	{
	    poly->m_heights.resize(nv);
	    for(int i = 0; i < nv / 2; ++i)
		poly->m_heights[i] = poly->m_heights[nv - 1 - i] = m_zinc * (nv/2 - i - 1);
	}

	scene->m_tmesh.AddExtrudedPolygon(poly, 0, m_height);
	scene->m_polys.push_back(poly);
    }

    void AddToSceneBumpyTiles::Add(Scene * const scene)
    {
	AddToScene::Add(scene);

	const bool   asTri  = true;
	const int    n      = scene->m_ground.GetNrVertices();
	const double *min   = scene->m_grid.GetMin();
	const double *max   = scene->m_grid.GetMax();
	const int    *dims  = scene->m_grid.GetDims();
	const double *units = scene->m_grid.GetUnits();

	for(int j = 0; j <= dims[1]; ++j)
	    for(int i = 0; i <= dims[0]; ++i)
		scene->m_ground.AddVertex(min[0] + i * units[0], min[1] + j * units[1], 0.0);

	scene->m_ground.AddQuad(n + (dims[0] + 1) * dims[1],
			       n + (dims[0] + 1) * (dims[1] + 1) - 1, n + dims[0], n, asTri);

	for(int j = 0; j < dims[1]; ++j)
	    for(int i = 0; i < dims[0]; ++i)
	    {
		const double z = RandomUniformReal(m_zmin, m_zmax);
		const int    m = scene->m_ground.GetNrVertices();
		const int    c = n + j * (dims[0] + 1) + i;

		scene->m_ground.AddVertex(min[0] + i       * units[0], min[1] + j       * units[1], z);
		scene->m_ground.AddVertex(min[0] + (i + 1) * units[0], min[1] + j       * units[1], z);
		scene->m_ground.AddVertex(min[0] + (i + 1) * units[0], min[1] + (j + 1) * units[1], z);
		scene->m_ground.AddVertex(min[0] + i       * units[0], min[1] + (j + 1) * units[1], z);

		scene->m_ground.AddQuad(m,     m + 1,        m + 2, m + 3, asTri);
		scene->m_ground.AddQuad(c,     c + 1,        m + 1, m, asTri);
		scene->m_ground.AddQuad(c + 1, c + dims[0] + 2, m + 2, m + 1, asTri);
		scene->m_ground.AddQuad(c + dims[0] + 1, m + 3, m + 2, c + dims[0] + 2, asTri);
		scene->m_ground.AddQuad(c + dims[0] + 1, c, m, m + 3, asTri);
	    }
    }


    double AddToSceneHeightField::EvalBumpy(HeightField * const hf, const int i, const int j)
    {
	AddToSceneHeightField *add = static_cast<AddToSceneHeightField*>(hf->m_evalFnData);

	return RandomUniformReal(add->m_zmin, add->m_zmax);
    }

    double AddToSceneHeightField::EvalRampX(HeightField * const hf, const int i, const int j)
    {
	AddToSceneHeightField *add = static_cast<AddToSceneHeightField*>(hf->m_evalFnData);

	return add->m_zmin + (add->m_zmax - add->m_zmin)  * sin(((i + 0.0) / hf->m_grid.GetDims()[0]) * M_PI);
    }

    double AddToSceneHeightField::EvalRampY(HeightField * const hf, const int i, const int j)
    {
	AddToSceneHeightField *add = static_cast<AddToSceneHeightField*>(hf->m_evalFnData);

	return add->m_zmin + (add->m_zmax - add->m_zmin) * sin(((j + 0.0) / (hf->m_grid.GetDims()[1] - 1)) * M_PI);
    }


    double AddToSceneHeightField::EvalRollingHills(HeightField * const hf, const int i, const int j)
    {
	AddToSceneHeightField *add = static_cast<AddToSceneHeightField*>(hf->m_evalFnData);

//i/nrDimsX ==> 0...1
//0   -> magmax
//1   -> magmin

//ax + b
//b = magmax
//a = -magmax + magmin
//0.25a + 0.5b + c = magmax =>0.25a + 0.5b = magmax - magmin
//b = 4 (magmax - magmin)
//a = -4(magmax - magmin)

	const double magmax = add->m_zmax;
	const double magmin = add->m_zmin;
	const double a      = -magmax + magmin;
	const double b      = magmax;
	double p[4];

	const int   coords[2] =
	    {
		i, j
	    };
	hf->m_grid.GetCellFromCoords(coords, p);


	const double xi     = p[0];
	const double yi     = p[1];//(j + 0.0) / (nrDimsZ - 1);
	const double duse   = sqrt(xi * xi + yi * yi) / sqrt(2);
	const double mag    = a * duse + b;

//	if(i == 0 || j == 0 || i == (nrDimsX - 1) || (j == nrDimsZ - 1))
//	    printf("%d %d %f %f %f %f\n", i, j, xi, yi, duse, mag);
//	else
//	if(duse <= 0.04)
//	    printf("%d %d %f %f %f %f\n", i, j, xi, yi, duse, mag);



	double wl = 0.5f;

	return add->m_zmax + add->m_zmax * sin(i * wl) * cos(j * wl);
    }



    void AddToSceneHeightField::Add(Scene * const scene)
    {
	AddToScene::Add(scene);

	if(m_fnEval != NULL)
	{
	    HeightField hf;

	    hf.m_grid.Setup2D(scene->m_grid.GetDims()[0] + 1,
			      scene->m_grid.GetDims()[1] + 1,
			      scene->m_grid.GetMin()[0],
			      scene->m_grid.GetMin()[1],
			      scene->m_grid.GetMax()[0] + scene->m_grid.GetUnits()[0],
			      scene->m_grid.GetMax()[1] + scene->m_grid.GetUnits()[1]);
	    hf.m_evalFnData = this;
	    hf.SetHeights(m_fnEval);
	    scene->m_ground.AddHeightField(&hf);
	}
    }


    double AddToSceneBumpyTerrain::EvalBumpy(HeightField * const hf, const int i, const int j)
    {
	AddToSceneBumpyTerrain *add = static_cast<AddToSceneBumpyTerrain*>(hf->m_evalFnData);

	return RandomUniformReal(add->m_zmin, add->m_zmax);
    }

    void AddToSceneBumpyTerrain::Add(Scene * const scene)
    {
	AddToScene::Add(scene);

	HeightField hf;

	hf.m_grid.Setup(scene->m_grid.GetNrDims(),
			scene->m_grid.GetDims(),
			scene->m_grid.GetMin(),
			scene->m_grid.GetMax());
	hf.m_evalFnData = this;
	hf.SetHeights(EvalBumpy);
	scene->m_ground.AddHeightField(&hf);

/*
	Polygon2D *poly = new Polygon2D();
	poly->m_vertices.resize(8);
	AABoxAsPolygon2D(scene->m_grid.GetMin(), scene->m_grid.GetMax(), &(poly->m_vertices[0]));
	scene->m_polys.push_back(poly);
*/
    }


    void AddToScenePotentialTerrain::Add(Scene * const scene)
    {
	double d, dnear, c[100];

	AddToScene::Add(scene);

	HeightField hf;
	hf.m_grid.Setup(scene->m_grid.GetNrDims(),
			scene->m_grid.GetDims(),
			scene->m_grid.GetMin(),
			scene->m_grid.GetMax());

	m_pts.resize(2 * m_nrPts);
	m_peaks.resize(m_nrPts);
	for(int k = 0; k < m_nrPts; ++k)
	{
	    m_pts[2 * k]     = RandomUniformReal(hf.m_grid.GetMin()[0], hf.m_grid.GetMax()[0]);
	    m_pts[2 * k + 1] = RandomUniformReal(hf.m_grid.GetMin()[1], hf.m_grid.GetMax()[1]);
	    m_peaks[k]       = RandomUniformReal(m_zmin, m_zmax);

	}

	printf("zmin = %f zmax = %f\n", m_zmin, m_zmax);

	m_dmin = HUGE_VAL;
	m_dmax = -HUGE_VAL;

	for(int i = 0; i < hf.m_grid.GetNrCells(); ++i)
	{
	    hf.m_grid.GetCellFromId(i, c);
	    dnear = HUGE_VAL;
	    for(int k = 0; k < m_nrPts; ++k)
	    {
		d = sqrt((m_pts[2 * k] - c[0]) * (m_pts[2 * k] - c[0]) +
			 (m_pts[2 * k + 1] - c[1]) * (m_pts[2 * k + 1] - c[1]));
		if(d < dnear)
		    dnear = d;
	    }
	    if(dnear > m_dmax)
		m_dmax = dnear;
	    if(dnear < m_dmin)
		m_dmin = dnear;
	}


	hf.m_evalFnData = this;
	m_hmin = HUGE_VAL;
	hf.SetHeights(EvalPotential);
	for(int i = 0; i < hf.m_grid.GetNrCells(); ++i)
	    hf.SetHeightAtCell(i, hf.GetHeightAtCell(i) - m_hmin);


	scene->m_ground.AddHeightField(&hf);

	printf("potential = %f %f\n", m_dmin, m_dmax);

/*
	Polygon2D *poly = new Polygon2D();
	poly->m_vertices.resize(8);
	AABoxAsPolygon2D(scene->m_grid.GetMin(), scene->m_grid.GetMax(), &(poly->m_vertices[0]));
	scene->m_polys.push_back(poly);
*/
    }


    double AddToScenePotentialTerrain::EvalPotential(HeightField *hf, const int i, const int j)
    {
	AddToScenePotentialTerrain *add = static_cast<AddToScenePotentialTerrain*>(hf->m_evalFnData);


	const double x    = hf->m_grid.GetMax()[0] - hf->m_grid.GetMin()[0];
	const double y    = hf->m_grid.GetMax()[1] - hf->m_grid.GetMin()[1];
	const double dmax = 5; //0.25 * sqrt(x * x + y * y);

	double c[10], d, dnear, h;
	int    inear = -1;

	const int coords[] = {i, j};

	hf->m_grid.GetCellFromCoords(coords, c);

	dnear = HUGE_VAL;
	h     = 0;

	for(int k = 0; k < add->m_nrPts; ++k)
	{
	    d = sqrt((add->m_pts[2 * k] - c[0]) * (add->m_pts[2 * k] - c[0]) +
		     (add->m_pts[2 * k + 1] - c[1]) * (add->m_pts[2 * k + 1] - c[1]));
	    if(d < dmax)
		h += add->m_peaks[k] * (1 - d/dmax);


/*
	    if(d < dnear)
	    {
		inear = k;
	  	dnear = d;
	    }
*/

	}

/*	if(dnear > add->m_dmax)
	    dnear = add->m_dmax;
	else if(dnear < add->m_dmin)
	    dnear = add->m_dmin;
*/
	//dnear = (dnear - add->m_dmin) / (add->m_dmax - add->m_dmin);
	dnear = dnear / sqrt(x * x + y * y);

//	if(h < 0.1)
//	    h = 0.1;

	if(h < add->m_hmin)
	    add->m_hmin = h;


	return h;//add->m_peaks[inear] * (1 - dnear); //;add->m_zmin + dnear * (add->m_zmax - add->m_zmin);
    }
}
