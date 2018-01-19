#include "MP/MPSimulator.hpp"
#include "Utils/Constants.hpp"

namespace MP
{
  MPSimulator::MPSimulator(void)
  {
    m_dt             = Constants::SIMULATOR_TIME_STEP;
    m_minDistOneStep = Constants::SIMULATOR_MIN_DISTANCE_ONE_STEP;
    m_maxDistOneStep = Constants::SIMULATOR_MAX_DISTANCE_ONE_STEP;
    m_scene          = NULL;
    m_id             = 0;
    m_length         = Constants::ROBOT_LENGTH;
    m_width          = Constants::ROBOT_WIDTH;
    m_height         = Constants::ROBOT_HEIGHT;
    m_cfgUseRot      = false;

  }

  MPSimulator::~MPSimulator(void)
  {
  }

  void MPSimulator::DrawCfgShape(const double cfg[])
  {
    double R[Algebra3D::Rot_NR_ENTRIES];
    double T3[] = {cfg[0], cfg[1], m_scene->m_maxGroundHeight+0.3};
    Algebra3D::ZAxisAngleAsRot(cfg[2], R);
    GDrawPushTransformation();
    GDrawMultTransRot(T3, R);
    m_projShape.Draw();
    GDrawPopTransformation();
  }

  bool MPSimulator::IsCfgValid(double cfg[])
  {
    return MinDistFromCfgToObstacles(cfg) > m_bodyLength/4;
  }

  bool MPSimulator::IsCfgValid(void)
  {
    return MinDistFromCfgToObstacles(GetCfg()) > 0.1;
  }


}
