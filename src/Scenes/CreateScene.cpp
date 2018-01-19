#include "Scenes/CreateScene.hpp"
#include "Utils/TriMeshReader.hpp"


namespace MP
{
  void CreateScene::Print(Scene * const scene, const char prefix[])
  {
    FILE     *scenefile = NULL;
    char      cmd[300];

    sprintf(cmd, "%s.txt", prefix);
    scenefile = fopen(cmd, "w");

    //print scene size
    fprintf(scenefile, "Grid2 %d %d %d %d 128 128\n",
    (int)scene->m_grid.GetMin()[0],(int)scene->m_grid.GetMin()[1],
    (int)scene->m_grid.GetMax()[0],(int)scene->m_grid.GetMax()[1]);

    //print robottype and searchType
    // fprintf(scenefile, "RobotType 0\n");
    // fprintf(scenefile, "SearchType 0\n");
    // fprintf(scenefile, "DepthSearch 2\n");
    //print scene view
    fprintf(scenefile, "SceneView\n");
    fprintf(scenefile, "eye 0.000000 -51.159329 105.039005\n");
    fprintf(scenefile, "center 0.000000 0.000000 0.000000\n");
    fprintf(scenefile, "right 1.000000 -0.000000 0.000000\n");
    fprintf(scenefile, "forward 0.000000 0.438371 -0.898794\n");
    //print scene ground file
    if(scene->m_ground.GetNrVertices() > 0)
    {
      FILE     *groundfile = NULL;
      sprintf(cmd, "ground.tmesh");
      printf("..writing ground file <%s>\n", cmd);
      fprintf(scenefile, "GroundFile %s\n",cmd);
      groundfile = fopen(cmd, "w");
      StandardTriMeshWriter(groundfile, &(scene->m_ground));
      fclose(groundfile);
    }
    else
    {
      fprintf(scenefile, "GroundFile none\n");
    }

    //print scene Obstacles file
    // if(scene->m_tmesh.GetNrVertices() > 0)
    // {
    //   FILE     *obsfile = NULL;
    //   sprintf(cmd, "%s_Obstacles.tmesh", prefix);
    //   printf("..writing Obstacles file <%s>\n", cmd);
    //   fprintf(scenefile, "ObstaclesFile %s\n",cmd);
    //   obsfile = fopen(cmd, "w");
    //   StandardTriMeshWriter(obsfile, &(scene->m_tmesh));
    //   fclose(obsfile);
    // }
    // else
    // {
      // fprintf(scenefile, "ObstaclesFile none\n");
    // }
    fprintf(scenefile, "ObstaclesFile none\n");

    //custom obsdata
    const int n   = scene->m_polys.size();
    fprintf(scenefile, "Obstacles %d \n", n);
    for(int i = 0; i < n; ++i)
    {
      int n = scene->m_polys[i]->m_vertices.size() / 2;
      fprintf(scenefile, "%d \n", n);
      for(int j = 0; j < 2 * n; ++j)
      fprintf(scenefile, "%f ", scene->m_polys[i]->m_vertices[j]);
      fprintf(scenefile, "\n0\n");
    }
    fclose(scenefile);


  }
}
