#include "Utils/GManager.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Flags.hpp"
#include "MP/MPPriorityPlanner.hpp"
#include "Utils/Timer.hpp"

using namespace MP;
extern "C" int RunPriorityPlanner(int argc, char **argv)
{
  MPPriorityPlanner  m_planner;
  double       tcurr      = 0;
  FILE        *in         = argc > 1 ? fopen(argv[1], "r") : NULL;
  FILE        *query      = argc > 2 ? fopen(argv[2], "r") : NULL;
  int          nrRobots   = argc > 3 ? atoi(argv[3]) : 1;
  int          robotType  = argc > 4 ? atoi(argv[4]) : 0;
  FILE        *out        = argc > 5 ? fopen(argv[5], "a+") : NULL;
  const double tmax       = argc > 6 ? atof(argv[6]) : 45;

  if(in&&query)
  {
    m_planner.m_scene.m_nrRobot = nrRobots;
    m_planner.m_scene.SetupFromFile(in,query);
    m_planner.m_robotType = robotType;
    m_planner.CompleteSetup();
    fclose(in);
    fclose(query);
  }

  if (Stats::GetSingleton()->GetValue("TimePreprocess") > 20)
  {
    fclose (out); // must close after opening
    return 0;
  }

  Timer::Clock clk;
  Timer::Start(&clk);
  while((tcurr = Timer::Elapsed(&clk)) < tmax && m_planner.IsSolved() <= 0)
  {
    m_planner.Run(100);
    // printf("time...%f\n", Timer::Elapsed(&clk));
  }
  tcurr = Timer::Elapsed(&clk);
  Stats::GetSingleton()->AddValue("TotalRunTime", tcurr);

  printf("done ... solved       = %d\n", m_planner.IsSolved() > 0);
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
  printf("AvgPathCosts          = %f\n", m_planner.getAvgPathCosts());
  m_planner.PrintStat(out);
  m_planner.ShowStat();

  fclose (out); // must close after opening
  return 0;
}
