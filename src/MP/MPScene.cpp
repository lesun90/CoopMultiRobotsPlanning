
#include "MP/MPScene.hpp"
#include "MP/Constants.hpp"
#include "Utils/Misc.hpp"
#include "Utils/TriMeshReader.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/GDraw.hpp"

namespace MP
{
  MPScene::MPScene(void)
  {
    m_nrRobot = -1;
    m_ground = NULL;
    m_maxGroundHeight = 0.4;
  }

  MPScene::~MPScene(void)
  {
    DeleteItems<Polygon2D*>(&(m_obstacles.m_polys));
  }

  void MPScene::ExtrudeObstacles(void)
  {
    double obstacleHeight = m_maxGroundHeight + 2;
    // m_obstacles.m_tmesh.Clear();
    double thickness = 0.5;
    m_obstacles.m_tmesh.AddBoundaries(m_grid.GetMin()[0]-thickness/2, m_grid.GetMin()[1]-thickness/2, -obstacleHeight,
    m_grid.GetMax()[0]+thickness/2, m_grid.GetMax()[1]+thickness/2, obstacleHeight,thickness);

    const int nrObs = m_obstacles.m_polys.size();
    for(int i = 0; i < nrObs; ++i)
    {
      m_obstacles.m_tmesh.AddExtrudedPolygon(m_obstacles.m_polys[i], 0.0, obstacleHeight);
    }
  }

  void MPScene::SetupFromFile(FILE * const in)
  {
    SetupScene(in);
  }

  void MPScene::SetupFromFile(FILE * const in,FILE * const query)
  {
    SetupScene(in);
    SetupQuery(query);
  }

  void MPScene::SetupScene(FILE * const in)
  {
    char keyword[100];
    char filename[100];
    FILE         *fp      = NULL;

    //read grid
    double minx, miny, maxx, maxy;
    int    dimsx, dimsy;
    if(fscanf(in, "%s %lf %lf %lf %lf %d %d",
    keyword, &minx, &miny, &maxx, &maxy, &dimsx, &dimsy) != 7)
    OnInputError(printf("Grid2 must have 6 params\n"));
    m_grid.Setup2D(dimsx, dimsy, minx, miny, maxx, maxy);

    //read view
    if(fscanf(in, "%s", keyword) != 1)
    OnInputError(printf(".. missing View data\n"));

    if(fscanf(in, "%s %lf %lf %lf",keyword, &eye[0], &eye[1], &eye[2]) != 4)
    OnInputError(printf(".. missing View data\n"));
    if(fscanf(in, "%s %lf %lf %lf",keyword, &center[0], &center[1], &center[2]) != 4)
    OnInputError(printf(".. missing View data\n"));
    if(fscanf(in, "%s %lf %lf %lf",keyword, &right[0], &right[1], &right[2]) != 4)
    OnInputError(printf(".. missing View data\n"));
    if(fscanf(in, "%s %lf %lf %lf",keyword, &forward[0], &forward[1], &forward[2]) != 4)
    OnInputError(printf(".. missing View data\n"));

    //read ground file
    if(fscanf(in, "%s %s", keyword, filename) != 2)
    OnInputError(printf(".. missing ground data\n"));
    printf("..setting up ground mesh from file <%s>\n", filename);
    if(strcmp(filename, "none") != 0)
    {
      fp = fopen(filename, "r");
      m_ground = new PQPTriMesh();
      // StandardTriMeshReader(fp, m_ground);
      TriMeshReader(filename, m_ground);
      fclose(fp);
    }
    if(!m_ground)
    {
      m_ground = new PQPTriMesh();
      m_ground->AddBox(m_grid.GetMin()[0], m_grid.GetMin()[1], -0.5, m_grid.GetMax()[0], m_grid.GetMax()[1], 0);
    }
    m_maxGroundHeight = m_ground->GetBoundingBoxMax()[2];

    //read obstacles file
    if(fscanf(in, "%s %s", keyword, filename) != 2)
    OnInputError(printf(".. missing obstacles data\n"));
    printf("..setting up obstacles mesh from file <%s>\n", filename);
    if(strcmp(filename, "none") != 0)
    {
      fp = fopen(filename, "r");
      m_obstacles.m_tmesh.Clear();
      TriMeshReader(filename, &m_obstacles.m_tmesh);
      fclose(fp);
    }
    else
    {
      m_obstacles.m_tmesh.Clear();
      //read custom obstacles
      int nrObs;
      if(fscanf(in, "%s %d", keyword, &nrObs) != 2)
      OnInputError(printf("..expecting Obstacles nrObs\n"));
      m_obstacles.m_polys.resize(nrObs);
      for(int i = 0; i < nrObs; ++i)
      {
        m_obstacles.m_polys[i] = new Polygon2D();
        m_obstacles.m_polys[i]->Read(in);
      }
      ExtrudeObstacles();
    }

  }

  void MPScene::SetupQuery(FILE * const in)
  {
    char keyword[100];
    char filename[100];
    FILE         *fp      = NULL;

    //read robots
    int nrRobot;
    std::vector<robotCfg*> robotInit;
    std::vector<Object*>   goals;
    if(fscanf(in, "%s %d", keyword, &nrRobot) != 2)
    OnInputError(printf("..expecting querysize\n"));
    robotInit.resize(nrRobot);
    goals.resize(nrRobot);

    for(int i = 0; i < nrRobot; ++i)
    {
      robotInit[i] = new robotCfg();
      if(fscanf(in, "%lf %lf %lf",
      &robotInit[i]->m_cfg[0],
      &robotInit[i]->m_cfg[1],
      &robotInit[i]->m_cfg[2]) != 3)
      OnInputError(printf("..expecting TR2 for Robots %d\n", i));

      goals[i] = new Object();
      if(fscanf(in, "%lf %lf %lf",
      &goals[i]->m_cfg[0],
      &goals[i]->m_cfg[1],
      &goals[i]->m_cfg[2]) != 3)
      OnInputError(printf("..expecting TR2 of goal %d\n", i));
    }

    std::vector<int> robotorder;
    robotorder.clear();
    for (int r = 0; r < nrRobot; r++)
    {
      robotorder.push_back(r);
    }
    //PermuteItems<int>(&robotorder,nrRobot);
    if (m_nrRobot  == -1)
    {
      m_nrRobot = nrRobot;
    }
    Stats::GetSingleton()->AddValue("NrRobots", m_nrRobot);

    m_robotInit.resize(m_nrRobot);
    m_goals.resize(m_nrRobot);
    for (int i = 0; i < m_nrRobot;i++)
    {
      int r = robotorder[i];
      m_robotInit[i] = new robotCfg();
      m_robotInit[i]->m_cfg[0] = robotInit[r]->m_cfg[0];
      m_robotInit[i]->m_cfg[1] = robotInit[r]->m_cfg[1];
      m_robotInit[i]->m_cfg[2] = robotInit[r]->m_cfg[2];

      m_goals[i] = new Object();
      m_goals[i]->m_cfg[0] = goals[r]->m_cfg[0];
      m_goals[i]->m_cfg[1] = goals[r]->m_cfg[1];
      m_goals[i]->m_cfg[2] = goals[r]->m_cfg[2];
    }
  }


  void MPScene::Draw(void)
  {
    DrawObstacles();
    //DrawGoals();
    DrawGround();
  }

  void MPScene::DrawGoals(void)
  {
    for(int i = (int) m_goals.size() - 1; i >= 0; --i)
    {
      char      msg[100];
      GMaterial gmat;
      GDrawMaterial(&gmat);
      sprintf(msg, "G%d", i);
      gmat.SetObsidian();
      GDrawMaterial(&gmat);
      GDrawString3D(msg, m_goals[0]->m_cfg[0], m_goals[0]->m_cfg[1], 0.04 , false, 2.0);
      GDrawPopTransformation();
    }
  }

  void MPScene::DrawScene(void)
  {
    glEnable(GL_LIGHTING);
    GDraw2D();
    GDrawPushTransformation();
    GDrawColor(0.455, 0.51, 0.561);
    GDrawMultTrans(0, 0, Constants::SCENE_DRAWZ_ROOM_BOX);
    GDrawAABox2D(m_grid.GetMin(), m_grid.GetMax());
    GDrawPopTransformation();
    glDisable(GL_LIGHTING);
  }

  void MPScene::DrawGround(void)
  {
    glEnable(GL_LIGHTING);
    GDrawPushTransformation();
    GDraw3D();
    if (m_ground== NULL)
    return;

    m_ground->DrawWithEdges();
    GDrawPopTransformation();
    glDisable(GL_LIGHTING);
  }

  void MPScene::DrawObstacles(void)
  {
    glEnable(GL_LIGHTING);
    GDrawPushTransformation();

    GDraw3D();
    GMaterial gmat;
    gmat.SetRuby();
    gmat.SetDiffuse(0.455, 0.51, 0.561);
    GDrawMaterial(&gmat);
    m_obstacles.m_tmesh.Draw();
    GDrawPopTransformation();

    GDrawPushTransformation();
    GDraw2D();
    GDrawColor(0.455, 0.51, 0.561);
    GDrawMultTrans(0, 0, Constants::SCENE_DRAWZ_ROOM_BOX);
    GDrawPopTransformation();
    glDisable(GL_LIGHTING);

  }

}
