#include "Utils/GManager.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Flags.hpp"
#include "MP/MPFollowPlanner.hpp"
#include "MP/MPScene.hpp"
#include "MP/MPRobotCar.hpp"
#include "MP/MPAbstraction.hpp"
#include "MP/MPTriAbstraction.hpp"
#include "MP/MPPrmAbstraction.hpp"

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

        m_abstract = new MPTriAbstraction();

        m_abstract->SetScene(m_scene);
        m_abstract->CompleteSetup();

      m_planner.m_sim = new MPRobotCar();
      m_planner.m_sim->m_id = 0;
      m_planner.m_sim->SetScene(m_scene);
      m_planner.m_sim->SetStateFromCfg(m_scene->m_robotInit[0]->m_cfg);
      m_planner.m_sim->SetGoal(m_scene->m_goals[0]->m_cfg);
      m_planner.m_sim->CompleteSetup();

      MPFollowPlanner::WayPoint wp;
      wp.m_radius = 10;
      wp.m_tolReach = wp.m_radius;
      wp.m_cfg = m_planner.m_sim->GetCfg();

      m_planner.m_waypts.push_back(wp);
      std::vector<int> path2goal;
      path2goal = m_abstract->m_regions[m_abstract->LocateRegion(m_planner.m_sim->GetCfg())]->m_pathsToGoal[0];
      for (int i = 1 ; i < path2goal.size(); i++)
      {
        wp.m_cfg = m_abstract->m_regions[path2goal[i]]->m_cfg;
        m_planner.m_waypts.push_back(wp);
      }

      m_planner.m_reserveTable.resize(1);
      m_planner.m_reserveTable[0].push_back(m_abstract->m_regions[path2goal[4]]->m_cfg);

    }

    virtual void HandleEventOnDisplay(void)
    {
      GManager::HandleEventOnDisplay();
      m_scene->Draw();
      m_planner.Draw();
      m_abstract->Draw();

      GDrawCircle2D(m_planner.m_reserveTable[0][0], 1);

    }

    virtual bool HandleEventOnNormalKeyPress(const int key)
    {
      if(key == 'r')
      {
        m_planner.Run();
      }
      if(m_planner.IsSolved() == true)
      {
        m_pos = 0;
      }
      return GManager::HandleEventOnNormalKeyPress(key);
    }

    virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y)
    {
      double *m_target;
      m_target = new double[3];
      MousePosFromScreenToWorld(x, y, &m_target[0], &m_target[1], &m_target[2]);
      printf("%f %f \n",m_target[0],m_target[1]);
      MPFollowPlanner::WayPoint wp;
      wp.m_radius = m_planner.m_radius;
      wp.m_tolReach = m_planner.m_tolReach;
      wp.m_cfg = m_target;

      // m_planner.m_waypts.push_back(wp);
      return true;
    }

    virtual void HandleEventOnTimer(void)
    {
      if(m_planner.IsSolved()==true)
      {
        m_planner.m_sim->SetState(m_planner.m_statePath[m_pos]);
        m_pos++;
        if (m_pos == m_planner.m_statePath.size() )
        {
          m_pos=0;
        }
      }
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
    int m_pos;
    MPAbstraction       *m_abstract;


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
