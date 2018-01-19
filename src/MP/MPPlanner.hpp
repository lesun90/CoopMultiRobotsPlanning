#ifndef MP_PLANNER_HPP_
#define MP_PLANNER_HPP_

#include "Utils/Flags.hpp"
#include "Utils/GDraw.hpp"
#include "MP/MPSimulator.hpp"
#include "MP/MPRobotCar.hpp"
#include "MP/MPBVehicleSimulator.hpp"
#include "MP/MPAbstraction.hpp"
#include "MP/MPTriAbstraction.hpp"
#include "MP/MPPrmAbstraction.hpp"
#include "MP/MPScene.hpp"

namespace MP
{
  class MPPlanner
  {
  public:

    enum Type
    {
      PRM     = 0,
      TRI     = 1,
      RCAR    = 0,
      RBULL   = 1,
      ASTAR   = 0,
      PAS     = 1
    };

    MPPlanner(void);

    virtual ~MPPlanner(void)
    {

    }

    virtual void     CompleteSetup();
    virtual void     SetupAbstraction();
    virtual void     Run(const int nrIters) = 0;
    virtual int      GetSolved(void) const = 0;
    virtual bool     IsSolved(void) = 0;
    virtual void     GetReversePath(const int vid, std::vector<int> * const rpath) const = 0;
    virtual double   PathCost(std::vector<int> * const path) const = 0;
    virtual void     DrawVertices() const = 0;
    virtual void     DrawGoal();
    virtual void     GetCfgPath(void) = 0;
    virtual void     Draw(void)
    {
      m_scene.Draw();
      m_abstract->DrawRegions();
      //m_abstract->DrawEdges();

      DrawVertices();
      DrawGoal();
      m_sim->Draw();
    }

    virtual void SetAbstraction(MPAbstraction *abstract)
    {
      m_abstract = abstract;
    }
    virtual void SetSimulator(MPSimulator *sim)
    {
      m_sim = sim;
    }

    virtual void PrintStat(FILE * const out)
    {
      fprintf(out, "%d %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
      IsSolved() > 0,
      Stats::GetSingleton()->GetValue("TotalRunTime")+Stats::GetSingleton()->GetValue("TimePreprocess"),
      Stats::GetSingleton()->GetValue("NrRobots"),
      Stats::GetSingleton()->GetValue("TimePreprocess"),
      Stats::GetSingleton()->GetValue("SimulateTime"),
      Stats::GetSingleton()->GetValue("ValidateCheckTime"),
      Stats::GetSingleton()->GetValue("FindPathTime"),
      Stats::GetSingleton()->GetValue("NrFindPath"),
      Stats::GetSingleton()->GetValue("PathDepth"),
      Stats::GetSingleton()->GetValue("CollisionWithRobots"),
      Stats::GetSingleton()->GetValue("CollisionWithObstacle"),
      Stats::GetSingleton()->GetValue("SearchType"),
      Stats::GetSingleton()->GetValue("RobotType"),
      getAvgPathCosts()
      );
    }

    virtual void ShowStat(void)
    {
      printf("done ... solved       = %d\n", IsSolved() > 0);
      printf("RunTime               = %f\n", Stats::GetSingleton()->GetValue("TotalRunTime"));
      printf("NrRobots              = %f\n", Stats::GetSingleton()->GetValue("NrRobots"));
      printf("TimePreprocess        = %f\n", Stats::GetSingleton()->GetValue("TimePreprocess"));
      printf("SimulateTime          = %f\n", Stats::GetSingleton()->GetValue("SimulateTime"));
      printf("ValidateCheckTime     = %f\n", Stats::GetSingleton()->GetValue("ValidateCheckTime"));
      printf("FindPathTime          = %f\n", Stats::GetSingleton()->GetValue("FindPathTime"));
      printf("NrFindPath            = %f\n", Stats::GetSingleton()->GetValue("NrFindPath"));
      printf("PathDepth             = %f\n", Stats::GetSingleton()->GetValue("PathDepth"));
      printf("CollisionWithRobots   = %f\n", Stats::GetSingleton()->GetValue("CollisionWithRobots"));
      printf("CollisionWithObstacle = %f\n", Stats::GetSingleton()->GetValue("CollisionWithObstacle"));
      printf("RobotType             = %f\n", Stats::GetSingleton()->GetValue("RobotType"));
      printf("SearchType            = %f\n", Stats::GetSingleton()->GetValue("SearchType"));
      printf("AvgPathCosts          = %f\n", getAvgPathCosts());
    }


    virtual double getAvgPathCosts(void)
    {
      return -1;
    }

    MPSimulator         *m_sim;
    MPAbstraction       *m_abstract;
    MPScene              m_scene;
    std::vector<double*> m_cfgPath;
    int                  m_nrSimSteps;
    int                  m_abstractionType;
    int                  m_robotType;

  };
}

#endif
