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
  class GRunEditor : public GManager
  {
  public:
    enum
    {
      FLAG_STEER = 1,
      FLAG_MOVE = 2
    };

    GRunEditor(void) : GManager()
    {
      m_timer   = 50;
      m_flags   = 0;
      m_target[0] = m_target[1] = m_target[2] = 0;

      GlobalCamera();
    }

    virtual ~GRunEditor(void)
    {
    }

    virtual void CompleteSetup()
    {
      // m_abstract = new MPTriAbstraction();
      m_abstract = new MPPrmAbstraction();
      m_abstract->SetScene(&m_scene);
      // m_abstract->CompleteSetup();
      // printf("nr regions %d\n",(int) m_abstract->m_regions.size() );
      m_gCamera.SetEyeCenterRightForward(m_scene.eye,
        m_scene.center,
        m_scene.right,
        m_scene.forward);
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
      if (points.size()>1)
      {
        for (int i = 0; i < points.size()-1; i ++)
        {
          GDrawSegment2D(points[i], points[i+1]);
        }
      }

      GDrawPopTransformation();

      if(!is2D)
      GDraw3D();

    }

    virtual bool HandleEventOnNormalKeyPress(const int key)
    {
      if(key == 'c')
      {
        printf("\n");
        points.clear();
        printf("\n");

      }
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
      printf("skel.push_back(%f);\n",m_target[0]);
      printf("skel.push_back(%f);\n",m_target[1]);
      // printf("%f %f ",m_target[0], m_target[1]);

      double *a = new double[2];
      a[0] = m_target[0];
      a[1] = m_target[1];

      points.push_back(a);
      return true;
    }

    virtual void HandleEventOnTimer(void)
    {
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
    // MPRobotCar      m_robot;
    MPBVehicleSimulator  m_robot;
    MPAbstraction  *m_abstract;
    //MPTriAbstraction  *m_abstract;

    int i;
    Flags     m_flags;
    double    m_target[3];

  };
};


extern "C" int GRunEditor(int argc, char **argv)
{
  MP::GRunEditor gManager;

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
