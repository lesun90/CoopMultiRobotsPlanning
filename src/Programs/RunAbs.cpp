#include "Utils/GManager.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Flags.hpp"
#include "MP/MPCoopPlanner.hpp"
#include "Utils/Timer.hpp"

using namespace MP;
extern "C" int RunAbs(int argc, char **argv)
{
  MPCoopPlanner  m_planner;
  double       tcurr      = 0;
  FILE        *in         = argc > 1 ? fopen(argv[1], "r") : NULL;
  FILE        *query      = argc > 2 ? fopen(argv[2], "r") : NULL;
  int          nrRobots   = argc > 3 ? atoi(argv[3]) : 1;
  FILE        *out        = argc > 4 ? fopen(argv[4], "a+") : NULL;
  const double tmax       = argc > 5 ? atof(argv[5]) : 90;

  if(in&&query)
  {
    m_planner.m_scene.m_nrRobot = nrRobots;
    m_planner.m_scene.SetupFromFile(in,query);
    m_planner.CompleteSetup();
    fclose(in);
    fclose(query);
  }

  printf("%f %f %f\n",
  Stats::GetSingleton()->GetValue("NrRobots"),
  Stats::GetSingleton()->GetValue("TimePreprocess"),
  Stats::GetSingleton()->GetValue("TimeFix"));

  fprintf(out, "%f %f \n",
  Stats::GetSingleton()->GetValue("NrRobots"),
  Stats::GetSingleton()->GetValue("TimePreprocess"));
  fclose (out); // must close after opening

  return 0;
}
