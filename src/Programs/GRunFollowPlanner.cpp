#include "Utils/GManager.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Flags.hpp"
#include "MP/MPFollowPlanner.hpp"
#include "MP/MPScene.hpp"
#include "MP/MPRobotCar.hpp"


namespace MP
{
  class GRunFollowPlanner : public GManager
  {
  public:
    enum
    {
      FLAG_DRAW_TREE = 1,
      FLAG_PAUSE = 2,
      FLAG_STEER = 4,
      FLAG_RUN  = 8
    };

    GRunFollowPlanner(void) : GManager()
    {
      printf("__GNUC__%d\n",__GNUC__ );

      m_timer   = 50;
      m_flags   = FLAG_DRAW_TREE;
      GlobalCamera();

      m_scene = new MPScene();

    }


    virtual ~GRunFollowPlanner(void)
    {

    }

    virtual void CompleteSetup()
    {
      m_gCamera.SetEyeCenterRightForward(m_scene->eye,
        m_scene->center,
        m_scene->right,
        m_scene->forward);

      m_planner.m_sim = new MPRobotCar();
      m_planner.m_sim->m_id = 0;
      m_planner.m_sim->SetScene(m_scene);
      m_planner.m_sim->SetStateFromCfg(m_scene->m_robotInit[0]->m_cfg);
      m_planner.m_sim->SetGoal(m_scene->m_goals[0]->m_cfg);
      m_planner.m_sim->CompleteSetup();
    }

    virtual void HandleEventOnDisplay(void)
    {
      GManager::HandleEventOnDisplay();
      m_scene->Draw();
      m_planner.Draw();
    }

    virtual bool HandleEventOnNormalKeyPress(const int key)
    {
      if(key == 'r')
      {
        m_planner.Run(100);
      }
      return GManager::HandleEventOnNormalKeyPress(key);
    }

    virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y)
    {
      double *m_target;
      m_target = new double[3];
      MousePosFromScreenToWorld(x, y, &m_target[0], &m_target[1], &m_target[2]);
      printf("%f %f \n",m_target[0],m_target[1]);
      m_planner.m_waypts.push_back(m_target);
      return true;
    }

    virtual void HandleEventOnTimer(void)
    {
      GManager::HandleEventOnTimer();
    }

    void GlobalCamera(void)
    {
      const double eye[] = {0.000000, -294.782392, 278.219162};
      const double center[] = {0.000000, 0.000000, 0.000000};
      const double right[] = {1.000000, -0.000000, 0.000000};
      const double forward[] = {0.000000, 0.731354, -0.681998};
      m_gCamera.SetEyeCenterRightForward(eye, center, right, forward);
    }

    Flags                 m_flags;
    MPFollowPlanner       m_planner;
    MPScene              *m_scene;

  };
};


extern "C" int GRunFollowPlanner(int argc, char **argv)
{
  MP::GRunFollowPlanner gManager;

  FILE        *in           = argc > 1 ? fopen(argv[1], "r") : NULL;
  FILE        *query        = argc > 2 ? fopen(argv[2], "r") : NULL;

  if(in&&query)
  {
    gManager.m_scene->SetupFromFile(in,query);
    gManager.CompleteSetup();
    fclose(in);
    fclose(query);
  }

  gManager.MainLoop("GRunPlanner", 1280, 720);

  return 0;
}
