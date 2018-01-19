#include "Scenes/CreateScene.hpp"
#include "Utils/TriMeshReader.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/Algebra2D.hpp"
#include <math.h>       /* asin */

using namespace MP;
void CreateCase1_add(Scene *scene)
{
  Polygon2D *poly;
  std::vector<double> skel;
  double thick = 1;

  skel.clear();
  skel.push_back(-12);
  skel.push_back(3);
  skel.push_back(-3.5);
  skel.push_back(3);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(-12);
  skel.push_back(-3);
  skel.push_back(-3.5);
  skel.push_back(-3);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(12);
  skel.push_back(3);
  skel.push_back(3.5);
  skel.push_back(3);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(12);
  skel.push_back(-3);
  skel.push_back(3.5);
  skel.push_back(-3);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);


}
extern "C" void CreateCase1(int argc, char **argv)
{
  const char  *prefix = argv[1];
  Scene        scene;
  double x,y;
  x = 60;
  y = 30;
  scene.m_grid.Setup2D(128, 128, -x/2, -y/2, x/2, y/2);

  AddArc m_arc;
  m_arc.m_x = -12;
  m_arc.m_y = -15;
  m_arc.m_r = 18;
  m_arc.m_nrRays = 10;
  m_arc.m_thetaMin = 90 * Constants::DEG2RAD;
  m_arc.m_thetaMax =  180 * Constants::DEG2RAD;
  m_arc.Add(&scene);

  m_arc.m_x = -12;
  m_arc.m_y = -15;
  m_arc.m_r = 12;
  m_arc.m_nrRays = 10;
  m_arc.m_thetaMin = 90 * Constants::DEG2RAD;
  m_arc.m_thetaMax =  180 * Constants::DEG2RAD;
  m_arc.Add(&scene);

  m_arc.m_x = 12;
  m_arc.m_y = 15;
  m_arc.m_r = 18;
  m_arc.m_nrRays = 10;
  m_arc.m_thetaMin = -90 * Constants::DEG2RAD;
  m_arc.m_thetaMax =  0 * Constants::DEG2RAD;
  m_arc.Add(&scene);

  m_arc.m_x = 12;
  m_arc.m_y = 15;
  m_arc.m_r = 12;
  m_arc.m_nrRays = 10;
  m_arc.m_thetaMin = -90 * Constants::DEG2RAD;
  m_arc.m_thetaMax =  0 * Constants::DEG2RAD;
  m_arc.Add(&scene);

  CreateCase1_add(&scene);

  AddToSceneRandomPolygons addRandom;
  scene.m_grid.Setup2D(32, 32, -30, 3, -6, 15);
  addRandom.m_zmax = 1.5;
  addRandom.m_perc = 0.2;
  addRandom.m_dsep = 2.5;
  addRandom.m_rmin = 1.0;
  addRandom.m_rmax = 1.6;
  addRandom.m_minNrVertices = 3;
  addRandom.m_maxNrVertices = 8;
  addRandom.m_maxNrPolys = 8;
  addRandom.Add(&scene);

  scene.m_grid.Setup2D(32, 32, -15, -15, -6, -3);
  addRandom.m_zmax = 1.5;
  addRandom.m_perc = 0.2;
  addRandom.m_dsep = 2.5;
  addRandom.m_rmin = 1.0;
  addRandom.m_rmax = 1.6;
  addRandom.m_minNrVertices = 3;
  addRandom.m_maxNrVertices = 8;
  addRandom.m_maxNrPolys = 3;
  addRandom.Add(&scene);

  scene.m_grid.Setup2D(32, 32, 6, -15, 30, -3);
  addRandom.m_zmax = 1.5;
  addRandom.m_perc = 0.2;
  addRandom.m_dsep = 2.5;
  addRandom.m_rmin = 1.0;
  addRandom.m_rmax = 1.6;
  addRandom.m_minNrVertices = 3;
  addRandom.m_maxNrVertices = 8;
  addRandom.m_maxNrPolys = 8;
  addRandom.Add(&scene);

  scene.m_grid.Setup2D(32, 32, 6, 6, 15, 15);
  addRandom.m_zmax = 1.5;
  addRandom.m_perc = 0.2;
  addRandom.m_dsep = 2.5;
  addRandom.m_rmin = 1.0;
  addRandom.m_rmax = 1.6;
  addRandom.m_minNrVertices = 3;
  addRandom.m_maxNrVertices = 8;
  addRandom.m_maxNrPolys = 3;
  addRandom.Add(&scene);

  scene.m_grid.Setup2D(32, 32, -6, 10, 6, 15);
  addRandom.m_zmax = 1.5;
  addRandom.m_perc = 0.2;
  addRandom.m_dsep = 2.5;
  addRandom.m_rmin = 1.0;
  addRandom.m_rmax = 1.6;
  addRandom.m_minNrVertices = 3;
  addRandom.m_maxNrVertices = 8;
  addRandom.m_maxNrPolys = 1;
  addRandom.Add(&scene);

  scene.m_grid.Setup2D(32, 32, -6, -15, 6, -10);
  addRandom.m_zmax = 1.5;
  addRandom.m_perc = 0.2;
  addRandom.m_dsep = 2.5;
  addRandom.m_rmin = 1.0;
  addRandom.m_rmax = 1.6;
  addRandom.m_minNrVertices = 3;
  addRandom.m_maxNrVertices = 8;
  addRandom.m_maxNrPolys = 1;
  addRandom.Add(&scene);

  GTexture *gtex = new GTexture();
  gtex->SetFileName("textures/terrain4.ppm");
  scene.m_ground.AddTexture(gtex);
  scene.m_ground.SetCurrentTexture(0);
  scene.m_ground.AddBox2D(scene.m_grid.GetMin()[0],
        scene.m_grid.GetMin()[1],
        scene.m_grid.GetMax()[0],
        scene.m_grid.GetMax()[1]);
  TriMesh::TexCoord tc;

  tc.m_u = 0; tc.m_v = 0;  scene.m_ground.AddTexCoord(tc);
  tc.m_u = 0.1; tc.m_v = 0;  scene.m_ground.AddTexCoord(tc);
  tc.m_u = 0.1; tc.m_v = 0.1;  scene.m_ground.AddTexCoord(tc);
  tc.m_u = 0; tc.m_v = 0.1;  scene.m_ground.AddTexCoord(tc);
  AddToSceneBumpyTerrain addBumpy;
  addBumpy.m_zmin  = 0.0;
  addBumpy.m_zmax  = 0.75;
  scene.m_grid.Setup2D(32, 32, -x/2, -y/2, x/2+2, y/2+2);
  addBumpy.Add(&scene);
  scene.m_grid.Setup2D(128, 128, -x/2, -y/2, x/2, y/2);
  CreateScene::Print(&scene, prefix);



}
