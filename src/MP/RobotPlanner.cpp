#include "MP/RobotPlanner.hpp"
#include "Utils/Timer.hpp"

namespace MP
{
  RobotPlanner::RobotPlanner(MPAbstraction *abstract, int id, int type)
  {
    m_shouldStop = false;
    SetAbstraction(abstract);
    m_robotType = type;
    if (m_robotType == RCAR)
    {
      printf("Use RobotCar\n");
      m_sim = new MPRobotCar();
    }
    else if (m_robotType == RBULL)
    {
      printf("Use RobotBullet\n");
      m_sim = new MPBVehicleSimulator();
    }

    m_sim->m_id = m_id = id;
    m_sim->SetScene(m_abstract->m_scene);
    m_sim->SetStateFromCfg(abstract->m_scene->m_robotInit[m_sim->m_id]->m_cfg);
    m_sim->SetGoal(abstract->m_scene->m_goals[m_sim->m_id]->m_cfg);

    m_sim->CompleteSetup();
    m_target = m_sim->NewCfg();

    //TODO: read from setup file
    m_tolReach = 2;
    m_tolReachregion = 2;
    m_sampleradius = 4;

    m_goalRid = m_abstract->LocateRegion(m_sim->m_goal);
    m_currentRegion = m_abstract->LocateRegion(m_sim->GetCfg());
    m_path.clear();
    m_prepareToStop = false;
    m_shouldStop = false;

  }

  void RobotPlanner::Draw(void)
  {
    DrawRobot();
    DrawGoal();
    // DrawNextTarget();
    //if(m_pathToGoal.size()>0)
    //DrawDiscretePathToGoal();
    //DrawAbstract();
  }

  void RobotPlanner::DrawRobot(void)
  {
    m_sim->Draw();
  }

  void RobotPlanner::DrawAbstract(void)
  {
    m_abstract->DrawRegions();
    //m_abstract->DrawEdges();
  }

  void RobotPlanner::DrawGoal(void)
  {
    const bool is2D = GDrawIs2D();
    GDrawPushTransformation();
    GDrawMultTrans(0, 0, m_sim->m_scene->m_maxGroundHeight+0.3);
    GDraw2D();
    GDrawIndexColor(0);
    GDrawCircle2D(m_sim->m_goal[0], m_sim->m_goal[1],m_sim->m_goal[2]);
    char      msg[100];
    sprintf(msg, "G%d", m_sim->m_id);
    GDrawColor(1, 0.875, 0);
    GDrawString3D(msg, m_sim->m_goal[0], m_sim->m_goal[1], 0.35 , false, 2.5);
    GDrawPopTransformation();

    if(!is2D)
    GDraw3D();
  }

  void RobotPlanner::DrawCfgPath(void)
  {
    const bool is2D = GDrawIs2D();

    GDrawPushTransformation();
    GDrawMultTrans(0, 0, 0.3);
    GDraw2D();
    GDrawIndexColor(m_sim->m_id);

    // printf("m_cfgPathsize %d robotid %d\n",m_cfgPath.size(),m_id );
    for(int i = m_cfgPath.size() - 1; i >= 0; --i)
    {
      GDrawCircle2D(m_cfgPath[i][0],m_cfgPath[i][1], 0.3);
    }

    GDrawPopTransformation();

    if(!is2D)
    GDraw3D();
  }

  void RobotPlanner::DrawDiscretePathToGoal(void)
  {
    const bool is2D = GDrawIs2D();

    GDrawPushTransformation();
    GDrawMultTrans(0, 0, 1.3);
    GDraw2D();
    GDrawIndexColor(m_sim->m_id);

    const int n = m_pathToGoal.size();
    	for(int i = 0; i < n - 1; ++i)
    	{
    	    const double *p1   = m_abstract->m_regions[m_pathToGoal[i]]->m_cfg;
    	    const double *p2   = m_abstract->m_regions[m_pathToGoal[i+1]]->m_cfg;
    	    const double  d    = p1[2];
    	    const double  vx   = p2[0] - p1[0];
    	    const double  vy   = p2[1] - p1[1];
    	    const double  norm = sqrt(vx * vx + vy * vy);
    	    const double  ux   = -vy * d / norm;
    	    const double  uy   =  vx * d/ norm;


    	    GDrawSegment2D(p1, p2);

    	}

    	GDrawWireframe(true);
    	for(int i = 0; i < n; ++i)
    	    GDrawCircle2D(m_abstract->m_regions[m_pathToGoal[i]]->m_cfg, m_tolReach);
    	GDrawWireframe(false);
      //
    	// for(int i = 0; i < n; ++i)
    	//     GDrawCircle2D(m_abstract->m_regions[m_pathToGoal[i]]->m_cfg, m_tolReach);

    	// char msg[100];
      //
    	// GDrawColor(0, 0, 0);
    	// for(int i = 0; i < n; ++i)
    	// {
    	//     sprintf(msg, "%d", i);
    	//     GDrawString2D(msg, GetWaypt(i)[0], GetWaypt(i)[1]);
    	// }
      //
      //

    GDrawPopTransformation();

    if(!is2D)
    GDraw3D();
  }

  void RobotPlanner::GenerateRandomTarget(void)
  {
    SampleRandomTarget();
    if(RandomUniformReal() < 0.1)
    {
      m_prepareToStop = true;
      SampleNextTargetAtGoal();
    }
  }

  void RobotPlanner::GenerateTarget(void)
  {

    int step       = RandomUniformInteger(1,1);
    // m_fromRegion   = m_currPathToGoal[m_currPathStep];
    m_fromRegion   = GetCurrentRegion();
    m_currPathStep = std::min<int>(m_currPathStep+step, m_currPathToGoal.size() - 1);
    m_toRegion     = m_currPathToGoal[m_currPathStep];

    SampleTargetInRegion(m_toRegion);
    m_nextToRegion = m_currPathToGoal[std::min<int>(m_currPathStep+1, m_currPathToGoal.size() - 1)];
    if (m_fromRegion == m_toRegion)
    {
      m_shouldStop = true;
    }
    else if (m_nextToRegion == m_toRegion)
    {
      m_prepareToStop = true;
    }
    if ((m_prepareToStop == true) && (m_toRegion == GetGoalRegion()))
    {
      SampleNextTargetAtGoal();
    }
    else if ((m_shouldStop == true) && (m_toRegion == GetGoalRegion()))
    {
      SampleNextTargetAtGoal();
    }
    if ((m_shouldStop == false) && (RandomUniformReal() < 0.1))
    {
      SampleRandomTarget();
      return;
    }
  }

  void RobotPlanner::StartSteerToTarget(void)
  {
    m_sim->StartSteerToPosition();
  }

  void RobotPlanner::SteerToTarget(void)
  {
    if(m_prepareToStop == true)
    {
      m_sim->SteerToStop(m_target);
    }
    else if ((m_shouldStop == true) && (HasReachNextRegion() != true))
    {
      m_sim->SteerToStop(m_abstract->m_regions[m_toRegion]->m_cfg);
    }
    else
    {
      m_sim->SteerToPosition(m_target);
    }

    Timer::Clock sclk;
    Timer::Start(&sclk);
    m_sim->SimulateOneStep();
    Stats::GetSingleton()->AddValue("SimulateTime", Timer::Elapsed(&sclk));
  }

  bool RobotPlanner::HasReachTarget(void)
  {
    return Algebra2D::PointDist(m_sim->GetCfg(), m_target) < m_tolReach;
  }

  bool RobotPlanner::HasReachNextRegion(void)
  {
    return HasReachRegion(m_toRegion,m_tolReach);
  }

  void RobotPlanner::GetCurrentPath(const MPState * const s,std::vector<int> path)
  {

  }


}
