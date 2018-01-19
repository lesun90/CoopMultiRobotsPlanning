#include "Scenes/CreateScene.hpp"
#include "Utils/TriMeshReader.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/Algebra2D.hpp"
#include <math.h>       /* asin */

using namespace MP;

extern "C" void CreateSceneB(int argc, char **argv)
{
  const char  *prefix = argv[1];
  Scene        scene;
  int size = 45;
  scene.m_grid.Setup2D(128, 128, -size, -size, size, size);
  AddToSceneRadial      OutRadial;
  scene.m_grid.Setup2D(128, 128, -size+2, -size+2, size-2, size-2);
  OutRadial.m_zmax      = 1.5;
  OutRadial.m_minGap    = 6;
  OutRadial.m_maxGap    = 6.5;
  OutRadial.m_rmin      = 7;
  OutRadial.m_rinc      = 8;
  OutRadial.m_nrRays    = 10;
  OutRadial.m_nrSegs    = 7;
  OutRadial.m_thick     = 1.5;
  OutRadial.Add(&scene);

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
  scene.m_grid.Setup2D(32, 32, -size, -size, size+3, size+3);
  addBumpy.Add(&scene);
  scene.m_grid.Setup2D(128, 128, -size, -size, size, size);
  CreateScene::Print(&scene, prefix);



}
