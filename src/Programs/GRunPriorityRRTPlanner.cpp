#include "Utils/GManager.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Flags.hpp"
#include "MP/MPPriorityPlannerRRT.hpp"

namespace MP
{
  class GRunPriorityRRTPlanner : public GManager
  {
  public:
    enum
    {
      FLAG_DRAW_TREE = 1,
      FLAG_PAUSE = 2,
      FLAG_STEER = 4,
      FLAG_RUN  = 8
    };

    GRunPriorityRRTPlanner(void) : GManager()
    {
      m_timer   = 50;
      m_flags   = FLAG_DRAW_TREE;
      GlobalCamera();
    }
    virtual void CompleteSetup()
    {
      m_gCamera.SetEyeCenterRightForward(m_planner.m_scene.eye,
        m_planner.m_scene.center,
        m_planner.m_scene.right,
        m_planner.m_scene.forward);
    }

    virtual ~GRunPriorityRRTPlanner(void)
    {

    }

    virtual void HandleEventOnDisplay(void)
    {
      GManager::HandleEventOnDisplay();
      m_planner.Draw();
    }

    virtual bool HandleEventOnNormalKeyPress(const int key)
    {
      if(key == 'p')
      m_flags = FlipFlag(m_flags, FLAG_PAUSE);
      else if(key == 'r')
      {
        Timer::Clock clk;
        Timer::Start(&clk);
        m_planner.Run(100);
        m_runtime += Timer::Elapsed(&clk);

        printf("[total time = %f %d]\n",m_runtime, m_planner.IsSolved() );
      }
      if(m_planner.IsSolved()>0)
      {
        m_pos = 0;
        for (int i = 0 ; i < m_planner.m_robots.size(); i++)
        {
          m_rpos[i] = 0;
        }
      }
      return GManager::HandleEventOnNormalKeyPress(key);
    }

    virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y)
    {
      double m_target[3];
      MousePosFromScreenToWorld(x, y, &m_target[0], &m_target[1], &m_target[2]);
      printf("%f %f \n",m_target[0],m_target[1]);

      return true;
    }

    virtual void HandleEventOnTimer(void)
    {
      if(m_planner.IsSolved()>0)
      {
        for (int r = 0; r < m_planner.m_robots.size(); ++r)
        {
          if (m_rpos[r]>=m_planner.m_robots[r]->m_path.size())
          {
            m_rpos[r] = m_planner.m_robots[r]->m_path.size() - 1;
          }
          m_planner.m_robots[r]->SetRobotStateAtPos(m_rpos[r]);
          m_rpos[r]++;
        }
      }

      GManager::HandleEventOnTimer();
    }

    void GlobalCamera(void)
    {
      const double eye[] = {0.000000, -129.326207, 117.180340};
	    const double center[] = {0.000000, 0.000000, 0.000000};
	    const double right[] = {1.000000, -0.000000, 0.000000};
	    const double forward[] = {0.000000, 0.731354, -0.681998};
      m_gCamera.SetEyeCenterRightForward(eye, center, right, forward);
    }

    Flags                 m_flags;
    MPPriorityPlannerRRT       m_planner;
    int                   m_pos;
    std::vector<int>      m_rpos;
    double                m_runtime;

  };
};


extern "C" int GRunPriorityRRTPlanner(int argc, char **argv)
{
  MP::GRunPriorityRRTPlanner gManager;

  FILE        *in           = argc > 1 ? fopen(argv[1], "r") : NULL;
  FILE        *query        = argc > 2 ? fopen(argv[2], "r") : NULL;
  int          nrRobot      = argc > 3 ? atoi(argv[3]) : 1;
  int          robotType    = argc > 4 ? atoi(argv[4]) : 0;

  if(in&&query)
  {
    gManager.m_planner.m_scene.m_nrRobot = nrRobot;
    gManager.m_planner.m_scene.SetupFromFile(in,query);
    gManager.m_planner.m_robotType = robotType;
    gManager.m_planner.CompleteSetup();
    gManager.m_rpos.resize(gManager.m_planner.m_robots.size());
    for (int i = 0 ; i < gManager.m_rpos.size(); i++)
    {
      gManager.m_rpos[i] = 0;
    }
    fclose(in);
    fclose(query);
    gManager.CompleteSetup();

  }

  gManager.MainLoop("GRunPlanner", 1280, 720);

  return 0;
}
