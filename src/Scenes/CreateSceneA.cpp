#include "Scenes/CreateScene.hpp"
#include "Utils/TriMeshReader.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/Algebra2D.hpp"
#include <math.h>       /* asin */

using namespace MP;

void CreateSceneA_add(Scene *scene)
{
  Polygon2D *poly;
  std::vector<double> skel;
  double thick = 2;

  skel.clear();
  skel.push_back(-52.675583);
  skel.push_back(-45.144852);
  skel.push_back(-46.349107);
  skel.push_back(-47.884116);
  skel.push_back(-39.071976);
  skel.push_back(-49.835428);
  skel.push_back(-32.403499);
  skel.push_back(-49.687895);
  skel.push_back(-27.692030);
  skel.push_back(-49.088683);
  skel.push_back(-20.691587);
  skel.push_back(-46.521105);
  skel.push_back(-17.355947);
  skel.push_back(-44.114382);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(-11.436778);
  skel.push_back(-56.717766);
  skel.push_back(-6.411778);
  skel.push_back(-52.210061);
  skel.push_back(2.848988);
  skel.push_back(-49.974651);
  skel.push_back(12.244627);
  skel.push_back(-49.928530);
  skel.push_back(18.028811);
  skel.push_back(-52.124736);
  skel.push_back(26.199833);
  skel.push_back(-55.872012);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(-35.072970);
  skel.push_back(-36.027917);
  skel.push_back(-36.225021);
  skel.push_back(-27.781311);
  skel.push_back(-39.131848);
  skel.push_back(-20.565247);
  skel.push_back(-45.342305);
  skel.push_back(-13.607069);
  skel.push_back(-50.559254);
  skel.push_back(-6.014360);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);
//
  skel.clear();
  skel.push_back(-29.014254);
  skel.push_back(-18.096506);
  skel.push_back(-22.358878);
  skel.push_back(-13.607069);
  skel.push_back(-9.658682);
  skel.push_back(-11.966729);
  skel.push_back(-0.745281);
  skel.push_back(-15.772848);
  skel.push_back(9.895529);
  skel.push_back(-17.384783);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);
//
  skel.clear();
  skel.push_back(-3.603958);
  skel.push_back(-28.455077);
  skel.push_back(5.805463);
  skel.push_back(-33.591789);
  skel.push_back(11.318668);
  skel.push_back(-35.381247);
  skel.push_back(21.087793);
  skel.push_back(-34.099960);
  skel.push_back(30.843313);
  skel.push_back(-32.087050);
  skel.push_back(34.619595);
  skel.push_back(-25.366973);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(22.646627);
  skel.push_back(-17.205592);
  skel.push_back(23.101128);
  skel.push_back(-7.331812);
  skel.push_back(25.964910);
  skel.push_back(1.969387);
  skel.push_back(30.672455);
  skel.push_back(10.044127);
  skel.push_back(34.271921);
  skel.push_back(14.239399);
  skel.push_back(38.832995);
  skel.push_back(18.009086);//
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(44.488440);
  skel.push_back(-1.045116);
  skel.push_back(51.155048);
  skel.push_back(-12.147327);
  skel.push_back(51.498494);
  skel.push_back(-20.738014);
  skel.push_back(48.468106);
  skel.push_back(-30.792718);
  skel.push_back(43.595920);
  skel.push_back(-42.076338);
  skel.push_back(38.336329);
  skel.push_back(-44.727686);//
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(-53.571559);
  skel.push_back(8.267655);
  skel.push_back(-47.547037);
  skel.push_back(14.438795);
  skel.push_back(-46.577622);
  skel.push_back(25.296367);
  skel.push_back(-47.968308);
  skel.push_back(36.685481);
  skel.push_back(-53.058900);
  skel.push_back(46.863371);//
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(-29.329975);
  skel.push_back(58.552726);
  skel.push_back(-33.001747);
  skel.push_back(52.007694);
  skel.push_back(-30.931416);
  skel.push_back(40.075595);
  skel.push_back(-26.558528);
  skel.push_back(33.105629);
  skel.push_back(-22.378560);
  skel.push_back(29.925633);
  skel.push_back(-17.449055);
  skel.push_back(28.343630);//
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(-12.107174);
  skel.push_back(46.127156);
  skel.push_back(-4.808302);
  skel.push_back(51.016016);
  skel.push_back(4.514399);
  skel.push_back(50.767442);
  skel.push_back(12.709961);
  skel.push_back(46.679668);
  skel.push_back(17.102346);
  skel.push_back(38.083485);
  skel.push_back(16.882483);
  skel.push_back(27.839514);//
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(0.139464);
  skel.push_back(31.617976);
  skel.push_back(3.967760);
  skel.push_back(23.399838);
  skel.push_back(3.790135);
  skel.push_back(18.871591);
  skel.push_back(0.131534);
  skel.push_back(6.922732);
  skel.push_back(-10.511825);
  skel.push_back(1.502565);//
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(32.381842);
  skel.push_back(51.508638);
  skel.push_back(34.189278);
  skel.push_back(42.766278);
  skel.push_back(40.123957);
  skel.push_back(35.728791);
  skel.push_back(48.419935);
  skel.push_back(25.778371);
  skel.push_back(55.535969);
  skel.push_back(20.879917);//
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(-35.611646);
  skel.push_back(28.562807);
  skel.push_back(-36.138321);
  skel.push_back(15.559963);
  skel.push_back(-29.053295);
  skel.push_back(4.346521);
  skel.push_back(-27.762595);
  skel.push_back(-2.303729);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

  skel.clear();
  skel.push_back(13.985497);
  skel.push_back(16.117642);
  skel.push_back(11.651898);
  skel.push_back(9.691575);
  skel.push_back(12.613106);
  skel.push_back(-1.814803);
  poly = new Polygon2D();
  FromSkeletonToPolygon2D(skel.size() / 2,
  &(skel)[0], thick,
  &(poly->m_vertices));
  scene->m_polys.push_back(poly);

}

extern "C" void CreateSceneA(int argc, char **argv)
{
  const char  *prefix = argv[1];
  Scene        scene;
  int size = 64;
  scene.m_grid.Setup2D(128, 128, -size, -size, size, size);
  CreateSceneA_add(&scene);
  AddToSceneRandomPolygons addRandom;
  scene.m_grid.Setup2D(128, 128, -size, -size, size, size);
  addRandom.m_zmax = 1.5;
  addRandom.m_perc = 0.2;
  addRandom.m_dsep = 4.5;
  addRandom.m_rmin = 1.5;
  addRandom.m_rmax = 2.5;
  addRandom.m_minNrVertices = 3;
  addRandom.m_maxNrVertices = 8;
  addRandom.m_maxNrPolys = 30;
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
  addBumpy.m_zmax  = 1.75;
  scene.m_grid.Setup2D(32, 32, -size, -size, size+4, size+4);
  addBumpy.Add(&scene);
  scene.m_grid.Setup2D(128, 128, -size, -size, size, size);
  CreateScene::Print(&scene, prefix);



}
