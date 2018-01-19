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
#include "MP/MPAbstraction.hpp"
#include "MP/MPPrmAbstraction.hpp"
#include "MP/MPTriAbstraction.hpp"

namespace MP
{
  class GRunScene : public GManager
  {
  public:
    enum
    {
      FLAG_STEER = 1,
      FLAG_MOVE = 2
    };

    GRunScene(void) : GManager()
    {
      m_timer   = 50;
      m_flags   = 0;
      m_target[0] = m_target[1] = m_target[2] = 0;

      GlobalCamera();
    }

    virtual ~GRunScene(void)
    {
    }

    virtual void CompleteSetup()
    {
      m_gCamera.SetEyeCenterRightForward(m_scene.eye,
        m_scene.center,
        m_scene.right,
        m_scene.forward);
      // m_abstract = new MPTriAbstraction();
      m_abstract = new MPPrmAbstraction();
      m_abstract->SetScene(&m_scene);
      // m_abstract->CompleteSetup();
      // printf("nr regions %d\n",(int) m_abstract->m_regions.size() );

    }

    virtual void HandleEventOnDisplay(void)
    {
      GManager::HandleEventOnDisplay();
      m_scene.Draw();
      // m_abstract->DrawRegions();
      // m_abstract->DrawEdges();
      m_robot.Draw();


      const bool is2D = GDrawIs2D();
      GDrawPushTransformation();
      GDrawMultTrans(0, 0, 0.3);
      GDraw2D();
      GDrawIndexColor(5);
      for (int i = 0; i < points.size(); i ++)
      {
        GDrawCircle2D(points[i], 0.5);
        if (i%2==1)
        {
          GDrawSegment2D(points[i], points[i-1]);
        }
      }
      GDrawPopTransformation();

      if(!is2D)
      GDraw3D();

      if(HasFlag(m_flags, FLAG_STEER))
      {
        GDrawColor(1.0, 0.0, 0.0);
        GDrawSphere3D(m_target, 1.0);
      }

    }

    virtual bool HandleEventOnNormalKeyPress(const int key)
    {
      if(key == 's')
      m_flags = FlipFlag(m_flags, FLAG_STEER);
      return GManager::HandleEventOnNormalKeyPress(key);
    }

    virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y)
    {
      MousePosFromScreenToWorld(x, y, &m_target[0], &m_target[1], &m_target[2]);
      if(HasFlag(m_flags, FLAG_STEER))
      {
        m_robot.StartSteerToPosition();
      }
      printf("%f %f \n",m_target[0],m_target[1]);
      // 
      // if(points.size() == 4)
      // {
      //   points.clear();
      // }
      // double *a = new double[2];
      // a[0] = m_target[0];
      // a[1] = m_target[1];
      //
      // points.push_back(a);
      // if(points.size() == 4)
      // {
      //   double cd = DistSegments(points[0],points[1],points[2],points[3]);
      //   printf("dist2seg: %f\n", cd);
      // }
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
      const double eye[] = {0.000000, -234.708158, 209.209480};
      const double center[] = {0.000000, 0.000000, 0.000000};
      const double right[] = {1.000000, -0.000000, 0.000000};
      const double forward[] = {0.000000, 0.731354, -0.681998};
      m_gCamera.SetEyeCenterRightForward(eye, center, right, forward);
    }

    std::vector<double*> points;
    MPScene              m_scene;
    MPRobotCar      m_robot;
    // MPBVehicleSimulator  m_robot;
    MPAbstraction  *m_abstract;
    //MPTriAbstraction  *m_abstract;

    int i;
    Flags     m_flags;
    double    m_target[3];

  };
};


extern "C" int GRunScene(int argc, char **argv)
{
  MP::GRunScene gManager;

  FILE *in = argc > 1 ? fopen(argv[1], "r") : NULL;
  double               x     = -0.0;
	double               y     = -0.0;
	double               t     = 0.0;
  double s[3] ;
  s[0] = x;
  s[1] = y;
  s[2] = t;
  if(in)
  {
    gManager.m_scene.SetupFromFile(in);
    gManager.m_scene.CompleteSetup();
    gManager.m_robot.SetScene(&gManager.m_scene);
    gManager.CompleteSetup();
    fclose(in);
    gManager.m_robot.CompleteSetup();
    gManager.m_robot.SetStateFromCfg(s);
  }

  gManager.MainLoop("GRunProblem", 1280, 720);

  return 0;
}
