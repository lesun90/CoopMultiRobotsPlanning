#include "Scenes/CreateScene.hpp"
#include "Utils/TriMeshReader.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/Algebra2D.hpp"
#include <math.h>       /* asin */

using namespace MP;
void CreateCase2_add(Scene *scene)
{
  const double thick = 1.0;
  std::vector<double> skel;
  Polygon2D *poly;
  skel.clear();
  skel.push_back(21.660883);
  skel.push_back(2.303064);
  skel.push_back(18.957377);
  skel.push_back(6.302739);
  skel.push_back(21.354339);
  skel.push_back(10.590304);
  skel.push_back(20.038880);
  skel.push_back(14.775692);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(22.753328);
  skel.push_back(-4.111109);
  skel.push_back(19.364852);
  skel.push_back(-9.254966);
  skel.push_back(22.798733);
  skel.push_back(-14.751784);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(-20.213665);
  skel.push_back(2.043749);
  skel.push_back(-18.881966);
  skel.push_back(5.511090);
  skel.push_back(-21.374619);
  skel.push_back(10.297543);
  skel.push_back(-19.726490);
  skel.push_back(14.586938);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(-21.413289);
  skel.push_back(-4.374240);
  skel.push_back(-19.269424);
  skel.push_back(-9.477705);
  skel.push_back(-21.381104);
  skel.push_back(-14.850441);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);



}
extern "C" void CreateCase2(int argc, char **argv)
{
  const char  *prefix = argv[1];
  Scene        scene;
  double x,y;
  x = 60;
  y = 30;
  scene.m_grid.Setup2D(128, 128, -x/2, -y/2, x/2, y/2);
  CreateCase2_add(&scene);

  const double thick = 1.0;
  std::vector<double> skel;
  Polygon2D *poly;
  skel.clear();
  for(double i = x/2-8.5; i >= -x/2+8.5; i -= 2)
  {
    skel.push_back(i);
    skel.push_back(3 + cos(0.2 * i) * 2);
  }
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2, &skel[0], thick, &(poly->m_vertices));
  scene.m_polys.push_back(poly);

  skel.clear();
  for(double i = x/2-7.5; i >= -x/2+7.5; i -= 2)
  {
    skel.push_back(i);
    skel.push_back(-3 + cos(0.2 * i) * 2);
  }
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2, &skel[0], thick, &(poly->m_vertices));
  scene.m_polys.push_back(poly);

  scene.m_grid.Setup2D(32, 32, -20, 2, 20, 15);
  AddToSceneRandomPolygons addRandom;
  addRandom.m_zmax = 1.5;
  addRandom.m_perc = 0.2;
  addRandom.m_dsep = 1.5;
  addRandom.m_rmin = 1.0;
  addRandom.m_rmax = 1.6;
  addRandom.m_minNrVertices = 3;
  addRandom.m_maxNrVertices = 8;
  addRandom.m_maxNrPolys = 12;
  addRandom.Add(&scene);

  scene.m_grid.Setup2D(32, 32, -20, -15, 20, -5);
  addRandom.m_zmax = 1.5;
  addRandom.m_perc = 0.2;
  addRandom.m_dsep = 1.5;
  addRandom.m_rmin = 1.0;
  addRandom.m_rmax = 1.6;
  addRandom.m_minNrVertices = 3;
  addRandom.m_maxNrVertices = 8;
  addRandom.m_maxNrPolys = 12;
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
