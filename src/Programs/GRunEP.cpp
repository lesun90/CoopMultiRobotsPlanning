#include "Utils/GManager.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Flags.hpp"
#include "MP/MPEPlanner.hpp"
#include "MP/RobotPlanner.hpp"

namespace MP
{
  class GRunEP : public GManager
  {
  public:
    enum
    {
      FLAG_DRAW_TREE = 1,
      FLAG_PAUSE = 2,
      FLAG_STEER = 4,
      FLAG_RUN  = 8
    };

    GRunEP(void) : GManager()
    {
      m_timer   = 50;
      m_flags   = FLAG_DRAW_TREE;
      GlobalCamera();
    }

    virtual ~GRunEP(void)
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
        if (m_planner.GetSolved()<0)
        {
          m_planner.Run(100);
          m_pathPos = 0;
        }
      }
      if (m_planner.GetSolved()>0)
      {
        m_planner.GetReversePath(m_planner.m_vidSolved, &m_path);
        m_pathPos = 0;
      }
      return GManager::HandleEventOnNormalKeyPress(key);
    }

    virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y)
    {
      double m_target[3];
      MousePosFromScreenToWorld(x, y, &m_target[0], &m_target[1], &m_target[2]);
      printf("click xy: %f %f \n",m_target[0],m_target[1]);
      return true;
    }

    virtual void HandleEventOnTimer(void)
    {
      if(HasFlag(m_flags, FLAG_PAUSE))
      return;

      if (m_planner.GetSolved() > 0)
      {
        m_planner.m_sim->SetState(m_planner.GetState(m_path[m_pathPos]));
        m_pathPos++;
        if (m_pathPos >= m_path.size())
        {
          m_pathPos = m_path.size() -1;
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

    Flags           m_flags;
    MPEPlanner      m_planner;
    std::vector<int> m_path;
    int             m_pathPos;

  };
};


extern "C" int GRunEP(int argc, char **argv)
{
  MP::GRunEP gManager;

  FILE *in = argc > 1 ? fopen(argv[1], "r") : NULL;
  FILE *query = argc > 2 ? fopen(argv[2], "r") : NULL;
  if(in&&query)
  {
    gManager.m_planner.m_scene.SetupFromFile(in,query);
    gManager.m_planner.m_scene.CompleteSetup();

    gManager.m_planner.CompleteSetup();
    fclose(in);
  }
  gManager.MainLoop("GRunPlanner", 1280, 720);

  return 0;
}
