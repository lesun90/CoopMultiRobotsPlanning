#include "MP/MPPlanner.hpp"

namespace MP
{
  MPPlanner::MPPlanner(void)
  {
    m_nrSimSteps = 0;
    m_abstractionType = PRM;
    m_robotType = RCAR;
  }

  void MPPlanner::CompleteSetup()
  {
    m_scene.CompleteSetup();
    SetupAbstraction();

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
    
    m_sim->m_id = 0;
    m_sim->SetScene(&m_scene);
    m_sim->CompleteSetup();
    m_sim->SetStateFromCfg(m_scene.m_robotInit[m_sim->m_id]->m_cfg);
    m_sim->SetGoal(m_scene.m_goals[m_sim->m_id]->m_cfg);
  }

  void MPPlanner::SetupAbstraction()
  {
    if (m_abstractionType == TRI)
    {
      m_abstract = new MPTriAbstraction();
    }
    else if (m_abstractionType == PRM)
    {
      m_abstract = new MPPrmAbstraction();
    }
    m_abstract->SetScene(&m_scene);
    m_abstract->CompleteSetup();
  }

  void MPPlanner::DrawGoal(void)
  {
    GDrawPushTransformation();
    char      msg[100];
    GMaterial gmat;
    GDrawMaterial(&gmat);
    sprintf(msg, "G%d", m_sim->m_id);
    gmat.SetObsidian();
    GDrawMaterial(&gmat);
    GDrawString3D(msg, m_sim->m_goal[0], m_sim->m_goal[1], m_scene.m_maxGroundHeight+ 0.04 , false, 2.0);

    GDrawPopTransformation();
  }

}
