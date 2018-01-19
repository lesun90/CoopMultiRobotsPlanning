#include "Utils/GManager.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Flags.hpp"
#include "MP/MPCoopPlanner.hpp"
#include "Utils/Timer.hpp"

using namespace MP;
extern "C" int RunCoopPlanner(int argc, char **argv)
{
  MPCoopPlanner  m_planner;
  double       tcurr      = 0;
  FILE        *in         = argc > 1 ? fopen(argv[1], "r") : NULL;
  FILE        *query      = argc > 2 ? fopen(argv[2], "r") : NULL;
  int          nrRobots   = argc > 3 ? atoi(argv[3]) : 1;
  int          robotType  = argc > 4 ? atoi(argv[4]) : 0;
  int          depthSearch= argc > 5 ? atoi(argv[5]) : 5;
  int          searchType = argc > 6 ? atoi(argv[6]) : 0;
  FILE        *out        = argc > 7 ? fopen(argv[7], "a+") : NULL;
  const double tmax       = argc > 8 ? atof(argv[8]) : 75;


// printf("robotType%d\n",robotType );
  if(in&&query)
  {
    m_planner.m_scene.m_nrRobot = nrRobots;
    m_planner.m_scene.SetupFromFile(in,query);
    m_planner.m_robotType = robotType;
    m_planner.m_depthSearch = depthSearch;
    m_planner.m_searchType = searchType;
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
  
  m_planner.ShowStat();
  m_planner.PrintStat(out);

  fclose (out); // must close after opening

  return 0;
}
