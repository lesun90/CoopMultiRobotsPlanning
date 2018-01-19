#include "Utils/GManager.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Flags.hpp"
#include "MP/MPCoopPlanner.hpp"
#include "MP/MPDiscretePlanner.hpp"


namespace MP
{
  class GRunCoopPlanner : public GManager
  {
  public:
    enum
    {
      FLAG_DRAW_TREE = 1,
      FLAG_PAUSE = 2,
      FLAG_STEER = 4,
      FLAG_RUN  = 8
    };

    GRunCoopPlanner(void) : GManager()
    {
      printf("__GNUC__%d\n",__GNUC__ );

      m_timer   = 50;
      m_flags   = FLAG_DRAW_TREE;
      GlobalCamera();
    }


    virtual ~GRunCoopPlanner(void)
    {

    }

    virtual void CompleteSetup()
    {
      m_gCamera.SetEyeCenterRightForward(m_planner.m_scene.eye,
        m_planner.m_scene.center,
        m_planner.m_scene.right,
        m_planner.m_scene.forward);
    }

    virtual void HandleEventOnDisplay(void)
    {
      GManager::HandleEventOnDisplay();
      m_planner.Draw();

      if(m_planner.IsSolved() == true)
      {
        // ExportFrameAsImage();
      }

      // const bool is2D = GDrawIs2D();
      // GDrawPushTransformation();
      // GDrawMultTrans(0, 0, 0.3);
      // GDraw2D();
      // GDrawIndexColor(5);
      // for (int i = 0; i < points.size(); i ++)
      // {
      //   GDrawCircle2D(points[i], 0.5);
      //   if (i%2==1)
      //   {
      //     GDrawSegment2D(points[i], points[i-1]);
      //   }
      // }
      // GDrawPopTransformation();

      // if(!is2D)
      // GDraw3D();

    }

    virtual bool HandleEventOnNormalKeyPress(const int key)
    {
      // ExportFrameAsImage();
      if(key == 'r')
      {
        Timer::Clock clk;
        Timer::Start(&clk);
        m_planner.Run(100);
        m_runtime += Timer::Elapsed(&clk);
        printf("[total time = %f %d]\n",m_runtime, m_planner.IsSolved() );

      }
      if(m_planner.IsSolved()>0)
      {
        for (int i = 0 ; i < m_planner.m_robots.size(); i++)
        {
          m_rpos[i] = 0;
        }
      }
      printf("done ... solved       = %d\n", m_planner.IsSolved() > 0);
      m_planner.ShowStat();
      return GManager::HandleEventOnNormalKeyPress(key);
    }

    virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y)
    {
      double m_target[3];
      MousePosFromScreenToWorld(x, y, &m_target[0], &m_target[1], &m_target[2]);
      printf("%f %f \n",m_target[0],m_target[1]);
      printf("region: %d\n",m_planner.m_abstract->LocateRegion(m_target) );

      // printf("skel.push_back(%f);\n",m_target[0]);
      // printf("skel.push_back(%f);\n",m_target[1]);
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
      const double eye[] = {0.000000, -294.782392, 278.219162};
      const double center[] = {0.000000, 0.000000, 0.000000};
      const double right[] = {1.000000, -0.000000, 0.000000};
      const double forward[] = {0.000000, 0.731354, -0.681998};
      m_gCamera.SetEyeCenterRightForward(eye, center, right, forward);
    }

    Flags                 m_flags;
    MPCoopPlanner         m_planner;
    std::vector<double*>  points;
    std::vector<int>      m_rpos;
    double                m_runtime;

  };
};


extern "C" int GRunCoopPlanner(int argc, char **argv)
{
  MP::GRunCoopPlanner gManager;

  FILE        *in           = argc > 1 ? fopen(argv[1], "r") : NULL;
  FILE        *query        = argc > 2 ? fopen(argv[2], "r") : NULL;
  int          nrRobot      = argc > 3 ? atoi(argv[3]) : 1;
  int          robotType    = argc > 4 ? atoi(argv[4]) : 0;
  int          depthSearch  = argc > 5 ? atoi(argv[5]) : 5;
  int          searchType   = argc > 6 ? atoi(argv[6]) : 0;

  if(in&&query)
  {
    gManager.m_planner.m_scene.m_nrRobot = nrRobot;
    gManager.m_planner.m_scene.SetupFromFile(in,query);
    gManager.m_planner.m_robotType = robotType;
    gManager.m_planner.m_depthSearch = depthSearch;
    gManager.m_planner.m_searchType = searchType;
    gManager.m_planner.CompleteSetup();
    gManager.m_rpos.resize(gManager.m_planner.m_robots.size());
    for (int i = 0 ; i < gManager.m_rpos.size(); i++)
    {
      gManager.m_rpos[i] = 0;
    }
    gManager.CompleteSetup();
    fclose(in);
    fclose(query);
  }

  gManager.MainLoop("GRunPlanner", 1280, 720);

  return 0;
}
