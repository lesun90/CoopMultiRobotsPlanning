#include "Utils/GManager.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Flags.hpp"
#include "Utils/Misc.hpp"
#include "MP/MPRobotCar.hpp"
#include "MP/MPBVehicleSimulator.hpp"
#include "MP/MPScene.hpp"

namespace MP
{
  class GRunSimulator : public GManager
  {
  public:
    enum
    {
      FLAG_STEER = 1,
      FLAG_MOVE = 2
    };

    GRunSimulator(void) : GManager()
    {
      m_timer   = 30;
      m_flags   = 0;
      m_target[0] = m_target[1] = m_target[2] = 0;

      GlobalCamera();
    }

    virtual ~GRunSimulator(void)
    {
    }

    virtual void CompleteSetup()
    {
      m_gCamera.SetEyeCenterRightForward(m_scene.eye,
        m_scene.center,
        m_scene.right,
        m_scene.forward);
        m_robot.StartSteerToPosition();

    }

    virtual void HandleEventOnDisplay(void)
    {
      GManager::HandleEventOnDisplay();
      m_scene.Draw();
      double s[3];
      s[0] = s[1] = s[2] = 0;
      //m_robot.DrawCfgShape(s);
      m_robot.Draw();

      if(HasFlag(m_flags, FLAG_STEER))
      {
        GDrawColor(1.0, 0.0, 0.0);
        GDrawSphere3D(m_target, 1.0);
      }

    }

    virtual bool HandleEventOnNormalKeyPress(const int key)
    {
      if(key == 's')
      {
        m_robot.StartSteerToPosition();
        m_flags = FlipFlag(m_flags, FLAG_STEER);

      }
      return GManager::HandleEventOnNormalKeyPress(key);
    }

    virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y)
    {
      MousePosFromScreenToWorld(x, y, &m_target[0], &m_target[1], &m_target[2]);
      if(HasFlag(m_flags, FLAG_STEER))
      {
        m_robot.StartSteerToPosition();
      }

      return true;
    }

    virtual void HandleEventOnTimer(void)
    {
      if(HasFlag(m_flags, FLAG_STEER))
      {
        m_robot.SteerToPosition(m_target);
        m_robot.SimulateOneStep();
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

    MPScene              m_scene;
    MPBVehicleSimulator      m_robot;
    // MPRobotCar      m_robot;

    Flags     m_flags;
    double    m_target[3];
    MPBVehicleState s;


  };
};


extern "C" int GRunSimulator(int argc, char **argv)
{
  MP::GRunSimulator gManager;

  FILE *in = argc > 1 ? fopen(argv[1], "r") : NULL;
  double               x     = 0;
	double               y     = 0;
	double               z     = 0;

  if(in)
  {
    gManager.m_scene.SetupFromFile(in);
    gManager.m_scene.CompleteSetup();
    gManager.m_robot.SetScene(&gManager.m_scene);
    fclose(in);
    double s[3];
    s[0] = x;
    s[1] = y;
    s[2] = z;
    gManager.m_robot.SetStateFromCfg(s);
    gManager.m_robot.CompleteSetup();
    gManager.CompleteSetup();
  }

  gManager.MainLoop("GRunProblem", 1280, 720);

  return 0;
}
