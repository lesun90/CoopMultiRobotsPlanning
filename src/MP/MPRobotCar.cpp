#include "MP/MPRobotCar.hpp"
#include "Utils/Constants.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/GMaterial.hpp"
#include <algorithm>    // std::reverse


namespace MP
{

  MPRobotCar::MPRobotCar(void)
  {
    m_bodyLength        = Constants::ROBOT_LENGTH;
  	m_bodyWidth         = Constants::ROBOT_WIDTH;
    m_bodyHeight        = Constants::ROBOT_HEIGHT - 0.5;
    m_wheelCylRadius    = 0.5;
    m_wheelThickness    = 0.2;

  	m_minSteerAngle     = Constants::ROBOT_MIN_STEER_ANGLE;
  	m_maxSteerAngle     = Constants::ROBOT_MAX_STEER_ANGLE;
  	m_minVel            = Constants::ROBOT_MIN_VELOCITY;
  	m_maxVel            = Constants::ROBOT_MAX_VELOCITY;
  	m_minAcc            = Constants::ROBOT_MIN_ACCELERATION;
  	m_maxAcc            = Constants::ROBOT_MAX_ACCELERATION;
  	m_minSteerVel       = Constants::ROBOT_MIN_STEER_VELOCITY;
  	m_maxSteerVel       = Constants::ROBOT_MAX_STEER_VELOCITY;

    //bodies
    GMaterial *gmat;

    //collision mesh
    m_tmeshCollision.AddBox(-0.5 * m_bodyLength,
    			                  -0.5 * m_bodyWidth - m_wheelThickness,
    			                  0,
    			                  0.5 * m_bodyLength,
    			                  0.5 * m_bodyWidth + m_wheelThickness,
    			                  m_wheelCylRadius + m_bodyHeight);

    //drawing meshes
    gmat = new GMaterial();
    gmat->SetPearl();

    m_tmeshDrawBody.SetMainMaterial(gmat);
    m_tmeshDrawBody.AddBox(-0.5 * m_bodyLength,
    			                 -0.5 * m_bodyWidth,
    			                  m_wheelCylRadius,
    			                  0.5 * m_bodyLength,
    			                  0.5 * m_bodyWidth,
    			                  m_wheelCylRadius + m_bodyHeight);

    m_projShape.AddBox(-0.5 * m_bodyLength,
    			             -0.5 * m_bodyWidth - m_wheelThickness,
    			              0,
    			              0.5 * m_bodyLength,
    			              0.5 * m_bodyWidth + m_wheelThickness,
    			              0);

    	gmat = new GMaterial();
    	gmat->SetRuby();
    	m_tmeshDrawWheel.SetMainMaterial(gmat);
    	m_tmeshDrawWheel.AddCylinderY(m_wheelCylRadius,
    				      -0.5 * m_wheelThickness,
    				       0.5 * m_wheelThickness);
    	m_tmeshDrawWheel.ApplyTrans(0, m_tmeshDrawWheel.GetNrVertices() - 1,
    				    0, 0, m_wheelCylRadius);

      m_currControl     = NULL;
      m_currControl     = NewControl();

      m_currState       = NULL;
      m_currState       = dynamic_cast<MPCarState*>(NewState());
  }

  void MPRobotCar::CompleteSetup(void)
  {
    // m_currControl     = NULL;
    // m_currControl     = NewControl();
    //
    // m_currState       = NULL;
    // m_currState       = dynamic_cast<MPCarState*>(NewState());
  }

  void MPRobotCar::DrawState(MPState * const s)
  {
    glEnable(GL_LIGHTING);

    MPCarState *vs = dynamic_cast<MPCarState*>(s);

    //id
    GDrawPushTransformation();
    GDrawMultTrans(0, 0, 0.8);
    char      msg[100];
    GMaterial gmat;
    gmat.SetRuby();
    GDrawMaterial(&gmat);
    sprintf(msg, "%d", m_id);
    GDrawString3D(msg, vs->m_state[0], vs->m_state[1], m_bodyHeight+0.04 , false, 1.5);
    GDrawPopTransformation();

    double T[Algebra3D::Trans_NR_ENTRIES];
    double R[Algebra3D::Rot_NR_ENTRIES];
    double Rs[Algebra3D::Rot_NR_ENTRIES];

    T[0] = vs->m_state[0];
    T[1] = vs->m_state[1];
    T[2] = 0;
    Algebra3D::ZAxisAngleAsRot(vs->m_state[2], R);
    Algebra3D::ZAxisAngleAsRot(vs->m_state[4], Rs);

    GDrawPushTransformation();
    GDrawMultTransRot(T, R);
    m_tmeshDrawBody.Draw();
    //arrow
    GDrawPushTransformation();
    GDraw2D();
    GDrawColor(0, 0, 1);
    GDrawMultTrans(0, 0, m_bodyHeight+ m_wheelCylRadius+0.02);
    GDrawLineWidth(4.0);
    GDrawArrow2D(0,0,0.5 * m_bodyLength,0);
    GDraw3D();
    GDrawPopTransformation();
    //draw wireframe for collision box
    // GDrawWireframe(1);
    // GDrawLineWidth(1.0);
    // m_tmeshCollision.Draw();
    // GDrawWireframe(0);

    //back left
    GDrawPushTransformation();
    GDrawMultTrans(-0.5 * m_bodyLength + m_wheelCylRadius, -0.5 * m_bodyWidth - 0.5 * m_wheelThickness, 0);
    m_tmeshDrawWheel.Draw();
    GDrawPopTransformation();

    //back right
    GDrawPushTransformation();
    GDrawMultTrans(-0.5 * m_bodyLength + m_wheelCylRadius, 0.5 * m_bodyWidth + 0.5 * m_wheelThickness, 0);
    m_tmeshDrawWheel.Draw();
    GDrawPopTransformation();

    //front left
    GDrawPushTransformation();
    GDrawMultTrans(0.5 * m_bodyLength - m_wheelCylRadius, -0.5 * m_bodyWidth - 0.5 * m_wheelThickness, 0);
    GDrawMultRot(Rs);
    m_tmeshDrawWheel.Draw();
    GDrawPopTransformation();

    //front right
    GDrawPushTransformation();
    GDrawMultTrans(0.5 * m_bodyLength - m_wheelCylRadius, 0.5 * m_bodyWidth + 0.5 * m_wheelThickness, 0);
    GDrawMultRot(Rs);
    m_tmeshDrawWheel.Draw();
    GDrawPopTransformation();

    GDrawPopTransformation();

    GDrawPopTransformation();
    glDisable(GL_LIGHTING);

  }

  double MPRobotCar::TimeStepConstantVelocity(const double v, const double d) const
  {
    const double absv = fabs(v);
    return absv < Constants::EPSILON ? (d / Constants::EPSILON) : (d / absv);
  }

  double MPRobotCar::TimeStepConstantAcceleration(const double v,const double a,const double d) const
  {
    if(fabs(a) < Constants::EPSILON)
    return TimeStepConstantVelocity(v, d);

    if(a >= 0.0 && v >= 0.0)
    return fabs((-v + sqrt(v * v + 2 * a * d)) / a);
    else if(a <= 0.0 && v <= 0.0)
    return fabs((v + sqrt(fabs(v * v - 2 * a * d))) / (-a));
    else if(a <= 0.0 && v >= 0.0)
    {
      const double b2_4ac = v * v + 2 * a * d;
      if(b2_4ac >= 0.0)
      return fabs((-v + sqrt(b2_4ac)) / a);
      const double dist_remaining = d - v * v / (2*a);
      //traveled from speed zero in reverse
      return sqrt(2 * dist_remaining / fabs(a));
    }
    else if(a >= 0 && v <= 0.0)
    {
      const double b2_4ac = v * v - 2 * a * d;
      if(b2_4ac >= 0.0)
      return fabs((-v - sqrt(b2_4ac)) / a);
      const double dist_remaining = d + v * v / (2 * a);
      return sqrt(2 * dist_remaining / a);
    }

    return Constants::EPSILON;
  }

  void MPRobotCar::MotionEqs(const double s[], const double t, const double u[], double ds[])
  {
    const double steer    = s[STATE_STEER_ANGLE];
  	const double cpsi     = cos(steer);
  	ds[STATE_X]           = s[STATE_VEL] * cos(s[STATE_THETA]) * cpsi;
  	ds[STATE_Y]           = s[STATE_VEL] * sin(s[STATE_THETA]) * cpsi;
  	ds[STATE_THETA]       = s[STATE_VEL] * sin(steer) / (m_bodyLength - 2*m_wheelCylRadius);
  	ds[STATE_VEL]         = u[CONTROL_ACC];
  	ds[STATE_STEER_ANGLE] = u[CONTROL_STEER_VEL];
  }

  void MPRobotCar::ClampState(double s[]) const
  {
    if(s[STATE_VEL] < m_minVel)
    s[STATE_VEL] = m_minVel;
    else if(s[STATE_VEL] > m_maxVel)
    s[STATE_VEL] = m_maxVel;

    const double sa = Algebra2D::AngleNormalize(s[STATE_STEER_ANGLE], -M_PI);

    if(sa < m_minSteerAngle)
    s[STATE_STEER_ANGLE] = m_minSteerAngle;
    else if(sa > m_maxSteerAngle)
    s[STATE_STEER_ANGLE] = m_maxSteerAngle;
  }

  void MPRobotCar::SimulateOneStep(void)
  {
    double dt = fabs(TimeStepConstantAcceleration(GetVelocity(),GetAcceleration(),RandomUniformReal(m_minDistOneStep, m_maxDistOneStep)));
    if(dt > m_dt)
    dt = m_dt;

    const double *s        = m_currState->m_state;
    const double  hhalf    = 0.5 * dt;
    const double  hthird   = dt / 3.0;
    const double  hsixth   = dt / 6.0;
    const int     dimState = GetDimState();

    std::vector<double> waux;
    waux.resize(3 * dimState);

    double *wa   = &waux[0];
    double *wb   = &waux[dimState];
    double *snew = &waux[2 * dimState];

    MotionEqs(s, 0, m_currControl, wa);
    for(int i = 0; i < dimState; ++i)
    {
      snew[i] = s[i] + hsixth * wa[i];
      wa[i]   = s[i] + hhalf  * wa[i];
    }
    MotionEqs(wa, hhalf, m_currControl, wb);
    for(int i = 0; i < dimState; ++i)
    {
      snew[i] += hthird * wb[i];
      wb[i]    = s[i] + hhalf * wb[i];
    }
    MotionEqs(wb, hhalf, m_currControl, wa);
    for(int i = 0; i < dimState; ++i)
    {
      snew[i] += hthird * wa[i];
      wa[i]    = s[i] + dt * wa[i];
    }
    MotionEqs(wa, dt, m_currControl, wb);
    for(int i = 0; i < dimState; ++i)
    snew[i] += hsixth * wb[i];

    ClampState(snew);
    CopyArray<double>(m_currState->m_state, dimState, snew);
    SetState(m_currState);

  }

  void MPRobotCar::StartSteerToPosition()
  {
    m_pidSteer.Reset();
    m_pidSteer.SetDesiredValue(0);
    m_pidVel.Reset();
  }

  void MPRobotCar::StartSteerToStop(void)
  {
    m_pidVel.Reset();
    m_pidVel.SetDesiredValue(0);

    m_pidSteer.Reset();
    m_pidSteer.SetDesiredValue(0);
  }

  void MPRobotCar::SteerToStop(const double target[])
  {
    const double u[2] = {target[0] - m_currState->m_state[0], target[1] - m_currState->m_state[1]};
    double dist = sqrt(u[0]*u[0]+u[1]*u[1]);

    m_pidLoc.Kp = 0.01 * m_maxVel;
    m_pidLoc.Ki = 0.01 * m_maxVel;
    m_pidLoc.Kd = 0.005 * m_maxVel;
    m_pidLoc.Reset();
    m_pidLoc.SetDesiredValue(1);
    double DesiredVel = -m_pidLoc.Update(dist, m_dt);
    // printf("didst: %f d: %f\n",dist, DesiredVel );
    if (DesiredVel>m_maxVel)
    {
      DesiredVel = m_maxVel;
    }
    else if (DesiredVel < m_minVel)
    {
      DesiredVel = m_minVel;
    }

    if (dist < 1)
    {
      DesiredVel = 0;
    }

    // printf("didst: %f d: %f\n",dist, DesiredVel );

    const double v[2] = {cos(m_currState->m_state[STATE_THETA] + m_currState->m_state[STATE_STEER_ANGLE]),sin(m_currState->m_state[STATE_THETA] + m_currState->m_state[STATE_STEER_ANGLE])};
    const double c[2] = {cos(m_currState->m_state[STATE_THETA] - m_currState->m_state[STATE_STEER_ANGLE]),sin(m_currState->m_state[STATE_THETA] - m_currState->m_state[STATE_STEER_ANGLE])};
    const double r[2] = {cos(m_currState->m_state[STATE_THETA]),sin(m_currState->m_state[STATE_THETA])};

    const double angleCurrTarget = Algebra2D::AngleNormalize(Algebra2D::VecFromToAngleCCW(r,u),0);

    if ((angleCurrTarget > 0.75 * M_PI ) && (angleCurrTarget < 1.25 * M_PI)
    && (Algebra2D::PointDist(GetCfg(),target)<15) && (m_currState->m_state[STATE_VEL] < m_maxVel/3))
    {
      m_pidVel.SetDesiredValue(RandomUniformReal(0.5, 1.0) * -DesiredVel);
      m_currControl[CONTROL_STEER_VEL] = m_pidSteer.Update(Algebra2D::VecFromToAngleCCW(u, c), m_dt);
    }
    else
    {
      m_pidVel.SetDesiredValue(RandomUniformReal(0.5, 1.0) * DesiredVel);
      m_currControl[CONTROL_STEER_VEL] = m_pidSteer.Update(Algebra2D::VecFromToAngleCCW(u, v), m_dt);
    }

    // m_pidVel.SetDesiredValue(RandomUniformReal(0.5, 1.0) * m_maxVel);
    m_currControl[CONTROL_ACC] = m_pidVel.Update(GetVelocity(), m_dt);
    if(m_currControl[CONTROL_ACC] > m_maxAcc)
    m_currControl[CONTROL_ACC] = m_maxAcc;
    else if(m_currControl[CONTROL_ACC] < m_minAcc)
    m_currControl[CONTROL_ACC] = m_minAcc;

    // m_currControl[CONTROL_STEER_VEL] = m_pidSteer.Update(Algebra2D::VecFromToAngleCCW(u, v), m_dt);
    if(m_currControl[CONTROL_STEER_VEL] > m_maxSteerVel)
    m_currControl[CONTROL_STEER_VEL] = m_maxSteerVel;
    else if(m_currControl[CONTROL_STEER_VEL] < m_minSteerVel)
    m_currControl[CONTROL_STEER_VEL] = m_minSteerVel;
  }

  void MPRobotCar::SteerToPosition(const double target[])
  {
    const double u[2] = {target[0] - m_currState->m_state[0], target[1] - m_currState->m_state[1]};
    const double v[2] = {cos(m_currState->m_state[STATE_THETA] + m_currState->m_state[STATE_STEER_ANGLE]),sin(m_currState->m_state[STATE_THETA] + m_currState->m_state[STATE_STEER_ANGLE])};
    const double c[2] = {cos(m_currState->m_state[STATE_THETA] - m_currState->m_state[STATE_STEER_ANGLE]),sin(m_currState->m_state[STATE_THETA] - m_currState->m_state[STATE_STEER_ANGLE])};
    const double r[2] = {cos(m_currState->m_state[STATE_THETA]),sin(m_currState->m_state[STATE_THETA])};

    const double angleCurrTarget = Algebra2D::AngleNormalize(Algebra2D::VecFromToAngleCCW(r,u),0);

    if ((angleCurrTarget > 0.75 * M_PI ) && (angleCurrTarget < 1.25 * M_PI)
    && (Algebra2D::PointDist(GetCfg(),target)<15) && (m_currState->m_state[STATE_VEL] < m_maxVel/3))
    {
      m_pidVel.SetDesiredValue(RandomUniformReal(0.5, 1.0) * m_minVel);
      m_currControl[CONTROL_STEER_VEL] = m_pidSteer.Update(Algebra2D::VecFromToAngleCCW(u, c), m_dt);
    }
    else
    {
      m_pidVel.SetDesiredValue(RandomUniformReal(0.5, 1.0) * m_maxVel);
      m_currControl[CONTROL_STEER_VEL] = m_pidSteer.Update(Algebra2D::VecFromToAngleCCW(u, v), m_dt);
    }

    // m_pidVel.SetDesiredValue(RandomUniformReal(0.5, 1.0) * m_maxVel);
    m_currControl[CONTROL_ACC] = m_pidVel.Update(GetVelocity(), m_dt);
    if(m_currControl[CONTROL_ACC] > m_maxAcc)
    m_currControl[CONTROL_ACC] = m_maxAcc;
    else if(m_currControl[CONTROL_ACC] < m_minAcc)
    m_currControl[CONTROL_ACC] = m_minAcc;

    // m_currControl[CONTROL_STEER_VEL] = m_pidSteer.Update(Algebra2D::VecFromToAngleCCW(u, v), m_dt);
    if(m_currControl[CONTROL_STEER_VEL] > m_maxSteerVel)
    m_currControl[CONTROL_STEER_VEL] = m_maxSteerVel;
    else if(m_currControl[CONTROL_STEER_VEL] < m_minSteerVel)
    m_currControl[CONTROL_STEER_VEL] = m_minSteerVel;
  }

  void MPRobotCar::StopVehicle()
  {

  }

  void MPRobotCar::SampleCfg(double cfg[])
  {
    cfg[0] = RandomUniformReal(m_scene->m_grid.GetMin()[0], m_scene->m_grid.GetMax()[0]);
    cfg[1] = RandomUniformReal(m_scene->m_grid.GetMin()[1], m_scene->m_grid.GetMax()[1]);
    cfg[2] = RandomUniformReal(-M_PI, M_PI);
  }

  bool MPRobotCar::IsStateValid(void)
  {
    double T3robot[Algebra3D::Trans_NR_ENTRIES];
    double R3robot[Algebra3D::Rot_NR_ENTRIES];

    T3robot[0] = m_currState->m_state[STATE_X];
    T3robot[1] = m_currState->m_state[STATE_Y];
    T3robot[2] = 0;
    Algebra3D::ZAxisAngleAsRot(m_currState->m_state[STATE_THETA], R3robot);
    //robot-obstacles
    if(m_tmeshCollision.Collision(T3robot, R3robot, &m_scene->m_obstacles.m_tmesh, NULL, NULL))
    {
      return false;
    }
    return true;
  }

  void MPRobotCar::GetCfg(double cfg[])
  {
    cfg[0] = m_currState->m_state[0];
    cfg[1] = m_currState->m_state[1];
    cfg[2] = m_currState->m_state[2];
  }

  double* MPRobotCar::GetCfg(void)
  {
    double *cfg;
    cfg = NewCfg();
    cfg[0] = m_currState->m_state[0];
    cfg[1] = m_currState->m_state[1];
    cfg[2] = m_currState->m_state[2];
    return cfg;
  }

}
