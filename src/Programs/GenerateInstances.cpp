#include "MP/MPScene.hpp"
#include "MP/MPRobotCar.hpp"
#include "Utils/Geometry.hpp"
#include <cstdio>

using namespace MP;

std::vector<double> mindits;

bool IsValidStates(MPRobotCar*  m_robot, double clearance)
{
  if (m_robot->IsStateValid() == false)
  {
    return false;
  }
  if (m_robot->GetCfg()[0] < m_robot->m_scene->m_grid.GetMin()[0])
  {
    return false;
  }
  if (m_robot->GetCfg()[0] > m_robot->m_scene->m_grid.GetMax()[0])
  {
    return false;
  }
  if (m_robot->GetCfg()[1] < m_robot->m_scene->m_grid.GetMin()[1])
  {
    return false;
  }
  if (m_robot->GetCfg()[1] > m_robot->m_scene->m_grid.GetMax()[1])
  {
    return false;
  }
  if (m_robot->MinDistFromCfgToObstacles(m_robot->GetCfg()) < clearance)
  {
    return false;
  }
  return true;
}


extern "C" int GenerateInstances(int argc, char **argv)
{
  MPScene                   m_scene;
  std::vector<MPRobotCar*>  m_robots;
  char      cmd[300];
  FILE        *out       = NULL;
  FILE *in = argc > 2 ? fopen(argv[1], "r") : NULL;
  FILE *query = argc > 2 ? fopen(argv[2], "r") : NULL;
  const char  *prefix = argv[3];
  double nrQueries = argc > 4 ? atoi(argv[4]) : 60;

  if(in&&query)
  {
    m_scene.SetupFromFile(in,query);
    m_scene.CompleteSetup();
    fclose(in);
    fclose(query);
  }
  m_robots.resize(m_scene.m_robotInit.size());

  for (int r = 0 ; r < m_robots.size(); r++)
  {
    m_robots[r] = new MPRobotCar();
    m_robots[r]->m_id = r;
    m_robots[r]->SetScene(&m_scene);
    m_robots[r]->SetStateFromCfg(m_scene.m_robotInit[m_robots[r]->m_id]->m_cfg);
    m_robots[r]->SetGoal(m_scene.m_goals[m_robots[r]->m_id]->m_cfg);
  }

  double radius = 1;
  double goalsize = 2.5;
  double range = 0.2;
  std::vector<double> orientation;
  orientation.resize(m_robots.size());
  mindits.resize(m_robots.size());
  for (int r = 0 ; r < m_robots.size(); r++)
  {
    orientation[r] = m_robots[r]->GetCfg()[2];
    mindits[r] = m_robots[r]->MinDistFromCfgToObstacles(m_robots[r]->GetCfg());
  }


  for(int j = 1; j < nrQueries; ++j)
  {
    printf("generate intances %d\n",j );
    sprintf(cmd, "%squery%d.txt",prefix,j);
    out = fopen(cmd, "w");
    fprintf(out, "Robots %d\n", (int)m_robots.size());
    for (int r = 0 ; r < m_robots.size(); r++)
    {
      //generate robot position
      do {
        double state[3];
        SampleRandomPointInsideCircle2D(m_scene.m_robotInit[r]->m_cfg,radius,state);
        state[2] = RandomUniformReal(orientation[r]-range,orientation[r]+range);
        m_robots[r]->SetStateFromCfg(state);
      } while(IsValidStates(m_robots[r],mindits[r]) == false);
      fprintf(out, "%f %f %f\n", m_robots[r]->GetCfg()[0], m_robots[r]->GetCfg()[1],m_robots[r]->GetCfg()[2]);

      //generate goals position
      do {
        double state[3];
        SampleRandomPointInsideCircle2D(m_scene.m_goals[r]->m_cfg,radius,state);
        state[2] = RandomUniformReal(-M_PI,M_PI);
        m_robots[r]->SetStateFromCfg(state);
      } while(IsValidStates(m_robots[r],1) == false);
      fprintf(out, "%f %f %f\n", m_robots[r]->GetCfg()[0], m_robots[r]->GetCfg()[1],goalsize);
    }
  }


}
