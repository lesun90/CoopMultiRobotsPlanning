#include "Scenes/CreateScene.hpp"
#include "Utils/TriMeshReader.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/Algebra2D.hpp"
#include <math.h>       /* asin */

using namespace MP;

void CreateCase3_add(Scene *scene)
{
  const double thick = 0.75;
  std::vector<double> skel;
  Polygon2D *poly;
  skel.clear();
  skel.push_back(-29.997357);
  skel.push_back(14.881612);
  skel.push_back(-16.932469);
  skel.push_back(3.208970);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(8.215329);
  skel.push_back(7.984182);
  skel.push_back(15.028172);
  skel.push_back(14.789553);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(17.949792);
  skel.push_back(2.247452);
  skel.push_back(29.835465);
  skel.push_back(2.853864 );
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(-23.371473);
  skel.push_back(-5.929869);
  skel.push_back(-29.870390);
  skel.push_back(-6.479246);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

}

extern "C" void CreateCase3(int argc, char **argv)
{
  const char  *prefix = argv[1];
  Scene        scene;
  double x,y;
  x = 60;
  y = 30;
  scene.m_grid.Setup2D(128, 128, -x/2, -y/2, x/2, y/2);

  AddArc m_arc;
  m_arc.m_x = 0;
  m_arc.m_y = -15;
  m_arc.m_r = 10;
  m_arc.m_nrRays = 5;
  m_arc.m_thetaMin = 0;
  m_arc.m_thetaMax =  37 * Constants::DEG2RAD;
  m_arc.Add(&scene);
  m_arc.m_x = 0;
  m_arc.m_y = -15;
  m_arc.m_r = 10;
  m_arc.m_nrRays = 5;
  m_arc.m_thetaMin = 70* Constants::DEG2RAD;
  m_arc.m_thetaMax =  180 * Constants::DEG2RAD;
  m_arc.Add(&scene);

  m_arc.m_x = 0;
  m_arc.m_y = -15;
  m_arc.m_r = 17;
  m_arc.m_nrRays = 8;
  m_arc.m_thetaMin = 0* Constants::DEG2RAD;
  m_arc.m_thetaMax =  110 * Constants::DEG2RAD;
  m_arc.Add(&scene);

  m_arc.m_x = 0;
  m_arc.m_y = -15;
  m_arc.m_r = 17;
  m_arc.m_nrRays = 5;
  m_arc.m_thetaMin = 128* Constants::DEG2RAD;
  m_arc.m_thetaMax =  180 * Constants::DEG2RAD;
  m_arc.Add(&scene);

  m_arc.m_x = 0;
  m_arc.m_y = -15;
  m_arc.m_r = 24;
  m_arc.m_nrRays = 5;
  m_arc.m_thetaMin = 0* Constants::DEG2RAD;
  m_arc.m_thetaMax =  45 * Constants::DEG2RAD;
  m_arc.Add(&scene);

  m_arc.m_x = 0;
  m_arc.m_y = -15;
  m_arc.m_r = 24;
  m_arc.m_nrRays = 8;
  m_arc.m_thetaMin = 58* Constants::DEG2RAD;
  m_arc.m_thetaMax =  145 * Constants::DEG2RAD;
  m_arc.Add(&scene);

  m_arc.m_x = 0;
  m_arc.m_y = -15;
  m_arc.m_r = 24;
  m_arc.m_nrRays = 5;
  m_arc.m_thetaMin = 158* Constants::DEG2RAD;
  m_arc.m_thetaMax =  180 * Constants::DEG2RAD;
  m_arc.Add(&scene);

  CreateCase3_add(&scene);

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
