#include "MP/MPTriAbstraction.hpp"
#include "External/ShewchukTriangle.hpp"
#include "Utils/Colormap.hpp"

namespace MP
{
  MPTriAbstraction::MPTriAbstraction(void): MPAbstraction()
  {
   m_useConformingDelaunay = true;
   m_angleConstraint = 20;
   m_areaConstraint = 520;
   m_minAreaToAdd = Constants::DECOMPOSITION_MIN_AREA_TO_ADD;
  }
  void MPTriAbstraction::CompleteSetup(void)
  {
    m_grid.Setup(2, m_scene->m_grid.GetDims(), m_scene->m_grid.GetMin(), m_scene->m_grid.GetMax());
    Triangulate();
    ConsTructWeight();
    LocatorCompleteSetup();
    MPAbstraction::CompleteSetup();
  }

  void MPTriAbstraction::HCosts(void)
  {
    MPAbstraction::HCosts();
  }

  void MPTriAbstraction::SamplePointInsideRegion(const int rid, double p[], const double dtol) const
  {
    const Polygon2D *shape = &(dynamic_cast<const TriRegion*>(m_regions[rid])->m_shape);
    const_cast<Polygon2D*>(shape)->SampleRandomPointInside(p);
  }

  void MPTriAbstraction::DrawRegion(const int rid)
  {
    if (m_regions[rid]->m_valid == false)
    return;
    GDrawWireframe(true);
    const double c = m_regions[rid]->m_colors[0];
    GDrawColor(Colormap::m_singleton->GetRed(c),
    Colormap::m_singleton->GetGreen(c),
    Colormap::m_singleton->GetBlue(c));
    GDrawLineWidth(1.0);
    dynamic_cast<TriRegion*>(m_regions[rid])->m_shape.Draw();
    GDrawWireframe(false);
  }

  void MPTriAbstraction::DrawRegions(void)
  {
    glDisable(GL_LIGHTING);
    GDrawPushTransformation();
    GDrawMultTrans(0, 0, m_scene->m_maxGroundHeight+0.4);
    for (int i = 0; i < m_regions.size(); i++)
    {
      DrawRegion(i);
    }
    GDrawPopTransformation();
    glEnable(GL_LIGHTING);

  }

  void MPTriAbstraction::DrawTriangle(const int rid)
  {
    if(m_regions[rid]->m_valid == false)
    {
      return;
    }

    GDrawWireframe(true);
    GDrawLineWidth(1.0);
    GDrawIndexColor(3);
    dynamic_cast<TriRegion*>(m_regions[rid])->m_shape.Draw();
    GDrawWireframe(false);
  }

  void MPTriAbstraction::DrawTriangles(void)
  {
    glDisable(GL_LIGHTING);
    GDrawPushTransformation();
    GDrawMultTrans(0, 0, m_scene->m_maxGroundHeight+0.3);
    for (int i = 0; i < m_regions.size(); i++)
    {
      DrawTriangle(i);
    }
    GDrawPopTransformation();
    glEnable(GL_LIGHTING);

  }

  int MPTriAbstraction::LocateRegion(const double p[])
  {
    const int cid = m_grid.GetCellId(p);

    if(cid < 0 || cid >= m_grid.GetNrCells())
    return -1;

    if(m_cellsToRegions[cid]->m_insideRegion >= 0)
    return m_cellsToRegions[cid]->m_insideRegion;

    for(int i = 0; i < m_cellsToRegions[cid]->m_intersectRegions.size(); ++i)
    {
      const std::vector<double> *poly =
      &(dynamic_cast<TriRegion*>(m_regions[m_cellsToRegions[cid]->m_intersectRegions[i]])->m_shape.m_vertices);

      if(IsPointInsidePolygon2D(p, poly->size() / 2, &((*poly)[0])))
      return m_cellsToRegions[cid]->m_intersectRegions[i];
    }
    return -1;
  }

  bool MPTriAbstraction::SplitRegion(const int rid, std::vector<int> *new_rids)
  {
    TriRegion   *r  = dynamic_cast<TriRegion *>(m_regions[rid]);

    new_rids->clear();

    if(r->m_shape.m_vertices.size() != 6 ||
    r->m_valid == false ||
    r->m_shape.GetArea() < 0.25)
    return false;

    new_rids->push_back(rid);
    new_rids->push_back(m_regions.size());
    new_rids->push_back(m_regions.size() + 1);


    TriRegion   *rnew[] = {new TriRegion(), new TriRegion(), new TriRegion()};
    const double cx     = r->m_cfg[0];
    const double cy     = r->m_cfg[1];

    for(int k = 0; k < 3; ++k)
    {
      rnew[k]->m_shape.m_vertices.resize(6);
      //k = 0 : 0,2  : 1, 3
      //k = 1 : 2,4  : 3, 5
      //k = 2 : 4,0  : 5, 1
      // 2 * k, (2 * k + 2) mod 6 : 2 * k + 1, (2 * k + 3) mod 6
      rnew[k]->m_shape.m_vertices[0] = cx;
      rnew[k]->m_shape.m_vertices[2] = r->m_shape.m_vertices[2 * k];
      rnew[k]->m_shape.m_vertices[4] = r->m_shape.m_vertices[(2 * k + 2) % 6];

      rnew[k]->m_shape.m_vertices[1] = cy;
      rnew[k]->m_shape.m_vertices[3] = r->m_shape.m_vertices[2 * k + 1];
      rnew[k]->m_shape.m_vertices[5] = r->m_shape.m_vertices[(2 * k + 3) % 6];

      MakePolygonCCW2D(3, &rnew[k]->m_shape.m_vertices[0]);
      // rnew[k]->m_cfg   = new double[2];
      rnew[k]->m_cfg[0] = (rnew[k]->m_shape.m_vertices[0] +
        rnew[k]->m_shape.m_vertices[2] +
        rnew[k]->m_shape.m_vertices[4]) / 3;
      rnew[k]->m_cfg[1] = (rnew[k]->m_shape.m_vertices[1] +
        rnew[k]->m_shape.m_vertices[3] +
        rnew[k]->m_shape.m_vertices[5]) / 3;

      rnew[k]->m_vol   = rnew[k]->m_shape.GetArea();
      rnew[k]->m_valid = true;
      rnew[k]->m_label = 1;
    }

    const double dtol = Constants::EPSILON;
    double       w;

    for(int i = r->m_neighs.size() - 1; i >= 0; --i)
    {
      TriRegion *rneigh = dynamic_cast<TriRegion*>(m_regions[r->m_neighs[i]]);
      const int  pos    = FindItem<int>(&(rneigh->m_neighs), rid);

      rneigh->m_neighs.erase(rneigh->m_neighs.begin() + pos);
      rneigh->m_dists.erase(rneigh->m_dists.begin() + pos);

      for(int k = 0; k < 3; ++k)
      if(HaveCommonEdgePolygons2D(rneigh->m_shape.m_vertices.size() / 2,
      &rneigh->m_shape.m_vertices[0],
      rnew[k]->m_shape.m_vertices.size() / 2,
      &(rnew[k]->m_shape.m_vertices[0]), dtol))
      {
        w = Algebra2D::PointDist(rneigh->m_cfg, rnew[k]->m_cfg);
        rnew[k]->m_neighs.push_back(r->m_neighs[i]);
        rnew[k]->m_dists.push_back(w);
        rneigh->m_neighs.push_back((*new_rids)[k]);
        rneigh->m_dists.push_back(w);
      }
    }

    for(int i = 0; i < 3; ++i)
    for(int k = i + 1; k < 3; ++k)
    {
      w = Algebra2D::PointDist(rnew[i]->m_cfg, rnew[k]->m_cfg);
      rnew[k]->m_neighs.push_back((*new_rids)[i]);
      rnew[k]->m_dists.push_back(w);
      rnew[i]->m_neighs.push_back((*new_rids)[k]);
      rnew[i]->m_dists.push_back(w);

    }

    LocatorRemoveCells(rid);

    m_regions[rid] = rnew[0];
    m_regions.push_back(rnew[1]);
    m_regions.push_back(rnew[2]);
    for(int k = 0; k < 3; ++k)
    LocatorUpdateCells((*new_rids)[k]);
    delete r;


    return true;
  }

  void MPTriAbstraction::LocatorUpdateCells(const int rid)
  {
    TriRegion *r = dynamic_cast<TriRegion *>(m_regions[rid]);

    r->m_shape.OccupiedGridCells(&m_grid, &(r->m_cellsInside), &(r->m_cellsIntersect));
    for(int j = 0; j < r->m_cellsInside.size(); ++j)
    m_cellsToRegions[r->m_cellsInside[j]]->m_insideRegion = rid;
    for(int j = 0; j < r->m_cellsIntersect.size(); ++j)
    m_cellsToRegions[r->m_cellsIntersect[j]]->m_intersectRegions.push_back(rid);
  }

  void MPTriAbstraction::LocatorCompleteSetup(void)
  {
    //construct cells to region map
    const int nrGridCells = m_grid.GetNrCells();
    m_cellsToRegions.resize(nrGridCells);
    for(int i = 0; i < nrGridCells; ++i)
    {
      m_cellsToRegions[i] = new InsideIntersectRegions();
      m_cellsToRegions[i]->m_insideRegion = -1;
    }

    const int nr = m_regions.size();
    for(int i = 0; i < nr; ++i)
    LocatorUpdateCells(i);
  }


  void MPTriAbstraction::LocatorRemoveCells(const int rid)
  {
    TriRegion *r = dynamic_cast<TriRegion *>(m_regions[rid]);

    for(int j = 0; j < r->m_cellsInside.size(); ++j)
    m_cellsToRegions[r->m_cellsInside[j]]->m_insideRegion = Constants::ID_UNDEFINED;

    for(int j = 0; j < r->m_cellsIntersect.size(); ++j)
    {
      const int pos = FindItem<int>(&m_cellsToRegions[r->m_cellsIntersect[j]]->m_intersectRegions, rid);
      if(pos >= 0)
      m_cellsToRegions[r->m_cellsIntersect[j]]->m_intersectRegions.erase(
        m_cellsToRegions[r->m_cellsIntersect[j]]->m_intersectRegions.begin() + pos);

      }
    }

    void MPTriAbstraction::Triangulate(void)
    {
      std::vector<double> vertices;
      std::vector<int>    nrVerticesPerContour;
      std::vector<double> ptsInsideHoles;
      const int           nrObsts = m_scene->m_obstacles.m_polys.size();
      int                 nrInvalid = 0;

      const double *pmin = m_grid.GetMin();
      const double *pmax = m_grid.GetMax();

      //grid boundaries
      vertices.push_back(pmin[0]); vertices.push_back(pmin[1]);
      vertices.push_back(pmax[0]); vertices.push_back(pmin[1]);
      vertices.push_back(pmax[0]); vertices.push_back(pmax[1]);
      vertices.push_back(pmin[0]); vertices.push_back(pmax[1]);
      nrVerticesPerContour.push_back(4);

      //obstacles and goals as holes
      ptsInsideHoles.resize(2 * (nrObsts));

      // //props
      // for(int i = 0; i < nrProps; ++i)
      // {
      //   m_goals[i]->UpdateProjPolygon();
      //   nrVerticesPerContour.push_back(m_goals[i]->m_poly.m_vertices.size() / 2);
      //   m_goals[i]->GetSomePointInside2D(&ptsInsideHoles[2 * i]);
      //   vertices.insert(vertices.end(),
      //   m_goals[i]->m_poly.m_vertices.begin(),
      //   m_goals[i]->m_poly.m_vertices.end());
      // }

      //obstacles
      for(int i = 0; i < nrObsts; ++i)
      {
        nrVerticesPerContour.push_back(m_scene->m_obstacles.m_polys[i]->m_vertices.size() / 2);
        m_scene->m_obstacles.m_polys[i]->GetSomePointInside(&ptsInsideHoles[2 * i]);
        vertices.insert(vertices.end(),
        m_scene->m_obstacles.m_polys[i]->m_vertices.begin(),
        m_scene->m_obstacles.m_polys[i]->m_vertices.end());
      }

      std::vector<double> triVertices;
      std::vector<int>    triIndices;
      std::vector<int>    triNeighs;

      TriangulatePolygonWithHoles2D(m_useConformingDelaunay,
        m_angleConstraint,
        m_areaConstraint,
        vertices.size() / 2,
        &vertices[0],
        &nrVerticesPerContour[0],
        ptsInsideHoles.size() / 2,
        &ptsInsideHoles[0],
        &triVertices, &triIndices, &triNeighs);

        //construct decomposition
      TriRegion   *region  = NULL;
      const int    nrTris3 = triIndices.size();
      double       pmin1[2];
      double       pmin2[2];

      //add triangle regions
      for(int i = 0; i < nrTris3; i += 3)
      {
        region = new TriRegion();
        region->m_label = 0;
        region->m_shape.m_vertices.push_back(triVertices[    2 * triIndices[i]]);
        region->m_shape.m_vertices.push_back(triVertices[1 + 2 * triIndices[i]]);
        region->m_shape.m_vertices.push_back(triVertices[    2 * triIndices[i + 1]]);
        region->m_shape.m_vertices.push_back(triVertices[1 + 2 * triIndices[i + 1]]);
        region->m_shape.m_vertices.push_back(triVertices[    2 * triIndices[i + 2]]);
        region->m_shape.m_vertices.push_back(triVertices[1 + 2 * triIndices[i + 2]]);
        MakePolygonCCW2D(3, &region->m_shape.m_vertices[0]);
        region->m_shape.GetSomePointInside(region->m_cfg);

        // PQPTriMesh tmesh;
        // const double *c = region->m_cfg;
        // tmesh.Clear();
        // tmesh.AddQuad(c,c,c,c);
        // double d = tmesh.Distance(NULL, NULL, &m_scene->m_obstacles.m_tmesh, NULL, NULL);

        region->m_valid = (region->m_shape.GetArea() > 2.5);
        // region->m_cfg   = new double[2];
        region->m_label = m_scene->m_grid.GetCellId(region->m_cfg);

        if(region->m_valid == false)
        {
          ++nrInvalid;
          // continue;
        }
        m_regions.push_back(region);

      }

      //no edge if a triangle's area is small or if it happens to be in collision
      for(int i = 0; i < nrTris3; i += 3)
    	{
        region = dynamic_cast<TriRegion*>(m_regions[i/3]);

        if(region->m_valid == false)
        continue;

        if(triNeighs[i] >= 0 && m_regions[triNeighs[i]]->m_valid == true)
        region->m_neighs.push_back(triNeighs[i]);
        if(triNeighs[i + 1] >= 0  && m_regions[triNeighs[i + 1]]->m_valid == true)
        region->m_neighs.push_back(triNeighs[i + 1]);
        if(triNeighs[i + 2] >= 0  && m_regions[triNeighs[i + 2]]->m_valid == true)
        region->m_neighs.push_back(triNeighs[i + 2]);
      }
      printf("nr Triangle: %d nrInvalid: %d \n",m_regions.size(), nrInvalid );
      //UpdateNeighs();
      //UpdateClearance();
      // printf("nrInvalid %d\n",nrInvalid );
    }

  }
