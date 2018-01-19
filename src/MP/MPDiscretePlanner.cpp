#include "MP/MPDiscretePlanner.hpp"
#include "Utils/Timer.hpp"

namespace MP
{
  void MPDiscretePlanner::CompleteSetup()
  {
    m_scene.CompleteSetup();
    m_abstract = new MPPrmAbstraction();
    // m_abstract = new MPTriAbstraction();
    printf("%d\n",m_abstract->m_regions.size() );
    m_abstract->SetScene(&m_scene);
    m_abstract->CompleteSetup();
    m_robots.resize(m_scene.m_robotInit.size());

    // m_multiSearch = new MPPushAndSwap();
    m_multiSearch = new MPCoopAStar();
    m_multiSearch->SetAbstraction(m_abstract);
    m_multiSearch->CompleteSetup();

    for (int i = 0; i< m_robots.size(); i++)
    {
      m_robots[i] = new RobotPlanner(m_abstract,i,m_robotType);
      m_multiSearch->m_inits[i] = m_abstract->LocateRegion(m_scene.m_robotInit[i]->m_cfg);
      m_multiSearch->m_goals[i] = m_abstract->LocateRegion(m_scene.m_goals[i]->m_cfg);
    }

  }
  void MPDiscretePlanner::MoveOneStep()
  {
    m_pos++;
    for (int r = 0 ; r < m_robots.size(); r++)
    {
      if (m_multiSearch->m_pathsToGoal[r].size() == 0)
      {
        continue;
      }
      if (m_pos >= m_multiSearch->m_pathsToGoal[r].size())
      {
        m_robots[r]->SetCurrentRegion(m_multiSearch->m_pathsToGoal[r].back());
      }
      else
      {
        m_robots[r]->SetCurrentRegion(m_multiSearch->m_pathsToGoal[r][m_pos]);
      }
    }
  }


  void MPDiscretePlanner::Run(const int nrIters)
  {
    m_pos = 0;
    for (int i = 0; i< m_robots.size(); i++)
    {
      m_multiSearch->m_inits[i] = m_robots[i]->GetCurrentRegion();
      m_multiSearch->m_pathsToGoal[i].clear();
    }

    if (m_multiSearch->isSolved() == false)
    {
      m_multiSearch->RunSearch(m_multiSearch->m_inits);
    }
    else
    {
      // printf("solved\n" );
    }

    for (int i = 0; i< m_robots.size(); i++)
    {
      // printf("%d size %d \n",i,m_multiSearch->m_pathsToGoal[i].size() );

    }
  }

  void MPDiscretePlanner::Draw()
  {
    m_scene.Draw();
    m_abstract->Draw();
    for (int i = 0; i< m_robots.size(); i++)
    {
      m_robots[i]->Draw();
    }
    m_multiSearch->DrawPaths();
  }

}
