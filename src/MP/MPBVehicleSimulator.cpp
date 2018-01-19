 #include "MP/MPBVehicleSimulator.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/TriMeshReader.hpp"
#include <vector>
#include <cstring>
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

namespace MP
{
  MPBVehicleSimulator::MPBVehicleSimulator(void) : MPBSimulator()
  {
  	m_minVel            = Constants::ROBOT_MIN_VELOCITY;
  	m_maxVel            = Constants::ROBOT_MAX_VELOCITY;
  	m_minAcc            = Constants::ROBOT_MIN_ACCELERATION;
  	m_maxAcc            = Constants::ROBOT_MAX_ACCELERATION;

    // m_dt = 0.05;
    m_bodyLength        = Constants::ROBOT_LENGTH;
  	m_bodyWidth         = Constants::ROBOT_WIDTH;
    m_bodyHeight        = Constants::ROBOT_HEIGHT - 0.5;
    m_wheelCylRadius    = 0.5;
    m_wheelThickness    = 0.2;
    // m_carChassisMesh.ReadOgreMesh("textures/chassis.mesh.xml", "textures/Car.jpg");
    // m_carChassisMesh.GetMainMaterial()->SetDiffuse(1, 0, 0);
    //drawing meshes
    GMaterial *gmat;
    gmat = new GMaterial();
    gmat->SetPearl();
    m_carChassisMesh.SetMainMaterial(gmat);
    m_carChassisMesh.AddBox(-0.5 * m_bodyLength,
        			              -0.5 * m_bodyWidth,
                            -0.5 * m_bodyHeight,
                             0.5 * m_bodyLength,
                             0.5 * m_bodyWidth,
                             0.5 * m_bodyHeight);
    m_projShape.AddBox(-0.5 * m_bodyLength,
        			              -0.5 * m_bodyWidth,
                            0,
                             0.5 * m_bodyLength,
                             0.5 * m_bodyWidth,
                             0);

    gmat = new GMaterial();
    gmat->SetRuby();
    m_wheelMesh.SetMainMaterial(gmat);
    m_wheelMesh.AddCylinderX(m_wheelCylRadius,
                -0.5 * m_wheelThickness,
                 0.5 * m_wheelThickness);

    m_maxEngineForce    = 2500;
    m_maxBreakingForce  = 100;
    m_maxSteering       = Constants::ROBOT_MAX_STEER_ANGLE;
    m_maxSteeringStep   = 0.2;

    float CUBE_HALF_EXTENTS = 1;

    float	steeringIncrement = 0.02f;
    float	wheelRadius = 0.5f;
    float	wheelWidth = 0.2f;
    float	wheelFriction = 100000;//BT_LARGE_FLOAT;
    float	suspensionStiffness = 20.f;
  	float	suspensionDamping = 2.3f;
  	float	suspensionCompression = 4.4f;
    float	rollInfluence = 0.1f;//1.0f;
    float connectionHeight = 0.5*m_bodyHeight-0.25f;

    int rightIndex = 0;
  	int upIndex = 1;
  	int forwardIndex = 2;
    btVector3 wheelDirectionCS0(0,0,-1);
    btVector3 wheelAxleCS(0,-1,0);
    btScalar suspensionRestLength(0.6);


    btCollisionShape *chassisShape = new btBoxShape(btVector3(0.5*m_bodyLength,0.5*m_bodyWidth, 0.5*m_bodyHeight));
    btCompoundShape  *compound     = new btCompoundShape();
    btTransform       localTrans;
    btTransform tr;
    //	chassisShape->setUserPointer((void *) &MPBSimulator::TYPE_ROBOT);

    localTrans.setIdentity();
    localTrans.setOrigin(btVector3(0,0,1));
    compound->addChildShape(localTrans,chassisShape);

    tr.setIdentity();
    m_carChassis = CreateRigidBody(200, tr, compound);

    m_carChassis->clearForces();

    m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
    m_carChassis->setLinearVelocity(btVector3(0,0,0));
    m_carChassis->setAngularVelocity(btVector3(0,0,0));
    m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_carChassis->getBroadphaseHandle(),m_dynamicsWorld->getDispatcher());

    m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
    m_vehicle          = new btRaycastVehicle(m_tuning,m_carChassis,m_vehicleRayCaster);

    ///never deactivate the vehicle
    m_carChassis->setActivationState(DISABLE_DEACTIVATION);

    m_dynamicsWorld->addVehicle(m_vehicle);

    bool isFrontWheel=true;
    m_vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);

    btVector3 connectionPointCS0(0.5 * m_bodyLength - wheelRadius,
                                 0.5 * m_bodyWidth + 0.5 * wheelWidth,
                                 connectionHeight);
	  m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,
			    suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

	  connectionPointCS0 = btVector3(0.5 * m_bodyLength - wheelRadius,
                                  -0.5 * m_bodyWidth - 0.5 * wheelWidth,
				                           connectionHeight);
	  m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,
			    suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

    isFrontWheel = false;
	  connectionPointCS0 = btVector3(-0.5 * m_bodyLength + wheelRadius,
                                   -0.5 * m_bodyWidth - 0.5 * wheelWidth,
                                   connectionHeight);
	  m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,
			    suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

	  connectionPointCS0 = btVector3(-0.5 * m_bodyLength+wheelRadius,
                                   0.5 * m_bodyWidth + 0.5 * wheelWidth,
                                   connectionHeight);
	  m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,
			    suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

	  for (int i=0;i<m_vehicle->getNumWheels();i++)
    {
      btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
      wheel.m_suspensionStiffness = suspensionStiffness;
      wheel.m_wheelsDampingRelaxation = suspensionDamping;
      wheel.m_wheelsDampingCompression = suspensionCompression;
      wheel.m_frictionSlip = wheelFriction;
      wheel.m_rollInfluence = rollInfluence;
    }
  }


  void MPBVehicleSimulator::CompleteSetup(void)
  {
    m_currControl     = NULL;
    m_currControl     = NewControl();
    MPBSimulator::CompleteSetup();
    SettleInitialState();
  }

  void MPBVehicleSimulator::SetStateFromCfg(double cfg[])
  {
    double height = m_scene->m_maxGroundHeight + m_bodyHeight/2;
    MPBVehicleState s;
    GetState(&s);
    s.m_transform.setRotation(btQuaternion(btVector3(0, 0, 1), cfg[2]) );
    s.m_transform.setOrigin(btVector3(cfg[0], cfg[1], height));
    SetState(&s);
  }

  void MPBVehicleSimulator::DrawState(MPState * const vs)
  {

  }

  void MPBVehicleSimulator::Draw()
  {
    glEnable(GL_LIGHTING);

    btScalar m[16];
    m_carChassis->getWorldTransform().getOpenGLMatrix(m);

    //id
    GDrawPushTransformation();
    GDrawMultTrans(0, 0, 0.8);
    char      msg[100];
    GMaterial gmat;
    gmat.SetRuby();
    GDrawMaterial(&gmat);
    sprintf(msg, "%d", m_id);
    GDrawString3D(msg, m_carChassis->getWorldTransform().getOrigin()[0],
                       m_carChassis->getWorldTransform().getOrigin()[1],
                       m_carChassis->getWorldTransform().getOrigin()[2]+0.04 , false, 1.5);
    GDrawPopTransformation();

    GDrawPushTransformation();
  	glMultMatrixf(m);
  	m_carChassisMesh.Draw();
    //arrow
    GDrawPushTransformation();
    GDraw2D();
    GDrawColor(0, 0, 1);
    GDrawMultTrans(0, 0, 0.55);
    GDrawLineWidth(4.0);
    GDrawArrow2D(0,0,0.5 * m_bodyLength,0);
    GDraw3D();
    GDrawPopTransformation();
  	GDrawPopTransformation();

    for(int i = 0; i < m_vehicle->getNumWheels(); ++i)
    {
      m_vehicle->updateWheelTransform(i,true);
      m_vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
      if(m_vehicle->getWheelInfo(i).m_raycastInfo.m_isInContact)
      glDisable(GL_TEXTURE_2D);
      else
      glEnable(GL_TEXTURE_2D);

      GDrawColor(1, 1, 1);
      GDrawPushTransformation();
      glMultMatrixf(m);
      m_wheelMesh.Draw();
      GDrawPopTransformation();
    }

    glDisable(GL_LIGHTING);

  }

  void MPBVehicleSimulator::SetState(const MPState * const s)
  {
    const MPBVehicleState *bvs = dynamic_cast<const MPBVehicleState*>(s);

    m_carChassis->clearForces();
    m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->
    cleanProxyFromPairs(m_carChassis->getBroadphaseHandle(),
    m_dynamicsWorld->getDispatcher());

    for(int i = 0; i < 4; ++i)
    if(bvs->m_wheels[i] != NULL)
    m_vehicle->m_wheelInfo[i] = *(bvs->m_wheels[i]);

    m_carChassis->setWorldTransform(bvs->m_transform);
    m_carChassis->setLinearVelocity(bvs->m_linearVel);
    m_carChassis->setAngularVelocity(bvs->m_angularVel);
    for(int i = 0; i < m_vehicle->getNumWheels(); ++i)
    m_vehicle->setSteeringValue(bvs->m_steering[i], i);

    m_vehicle->resetSuspension();
    for (int i=0;i<m_vehicle->getNumWheels();i++)
    m_vehicle->updateWheelTransform(i,false);
  }

  void MPBVehicleSimulator::GetState(MPState * const s)
  {
    MPBVehicleState *bvs = dynamic_cast<MPBVehicleState*>(s);

    bvs->m_transform  = m_carChassis->getWorldTransform();
    bvs->m_linearVel  = m_carChassis->getLinearVelocity();
    bvs->m_angularVel = m_carChassis->getAngularVelocity();

    for(int i = 0; i < m_vehicle->getNumWheels(); ++i)
    {
      if(bvs->m_wheels[i] == NULL)
      bvs->m_wheels[i] = new btWheelInfo(m_vehicle->m_wheelInfo[i]);
      else
      *(bvs->m_wheels[i]) = m_vehicle->m_wheelInfo[i];
      bvs->m_steering[i] = m_vehicle->getSteeringValue(i);
    }
  }

  void MPBVehicleSimulator::GetControl(double * const u)
  {
    for (int i = 0 ; i < GetDimControl(); i++)
    {
      u[i] = m_currControl[i];
    }
  }

  void MPBVehicleSimulator::CopyState(MPState * const dest,
      const MPState * const src) const
  {
    MPBVehicleState       *d = dynamic_cast<MPBVehicleState *>(dest);
    const MPBVehicleState *s = dynamic_cast<const MPBVehicleState *>(src);

    d->m_transform = s->m_transform;
    d->m_linearVel = s->m_linearVel;
    d->m_angularVel= s->m_angularVel;

    for(int i = 0; i < m_vehicle->getNumWheels(); ++i)
    {
      if(d->m_wheels[i] == NULL)
      d->m_wheels[i] = new btWheelInfo(*(s->m_wheels[i]));
      else
      *(d->m_wheels[i]) = *(s->m_wheels[i]);
      d->m_steering[i] = s->m_steering[i];
    }
  }

  void MPBVehicleSimulator::StartSteerToPosition()
  {
    m_pidSteer.Reset();
  	m_pidSteer.SetDesiredValue(0);
    m_pidVel.Reset();
    m_pidVel.SetDesiredValue(m_maxVel);
    m_pidVel.Kp = 0.05 * m_maxEngineForce;
    m_pidVel.Ki = m_pidVel.Kd = 0.1 * m_pidVel.Kp;
  }

  void MPBVehicleSimulator::StartSteerToStop(void)
  {
    m_pidSteer.Reset();
    m_pidSteer.SetDesiredValue(0);

    m_pidVel.Reset();
    m_pidVel.SetDesiredValue(0);
    m_pidVel.Kp = 0.05 * m_maxEngineForce;
    m_pidVel.Ki = m_pidVel.Kd = 0.1 * m_pidVel.Kp;
  }

  void MPBVehicleSimulator::SteerToStop(const double target[])
  {
    btScalar yaw, pitch, roll;
    btMatrix3x3 mat = btMatrix3x3(m_carChassis->getWorldTransform().getBasis());
    mat.getEulerYPR(yaw, pitch, roll);
    double stateTheta = yaw;

    const double    prevSteer = m_vehicle->getSteeringValue(0);
  	const btVector3 pos       = m_carChassis->getWorldTransform().getOrigin();
  	const btVector3 linvel    = m_carChassis->getLinearVelocity();

  	const double    u[2]      = {target[0] - pos[0], target[1] - pos[1]};
  	const double    v[2]      = {linvel[0], linvel[1]};
    const double    r[2]      = {cos(stateTheta),sin(stateTheta)};

    const double    v1[2]      = {cos(stateTheta+prevSteer),sin(stateTheta+prevSteer)};
    const double    v2[2]      = {cos(stateTheta-prevSteer),sin(stateTheta-prevSteer)};

    // const double    c[2]      = {-linvel[0], linvel[1]};
  	const double    vnorm     = v[0] * v[0] + v[1] * v[1];
    const double angleCurrTarget = Algebra2D::AngleNormalize(Algebra2D::VecFromToAngleCCW(r,u),0);

    double steer;
    double force;

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

    if ((angleCurrTarget > 0.75 * M_PI ) && (angleCurrTarget < 1.25 * M_PI)
    && (Algebra2D::PointDist(GetCfg(),target)<15) )
    {
      m_pidVel.SetDesiredValue(RandomUniformReal(0.5, 1.0) * -DesiredVel);
      force = m_pidVel.Update(-GetVelocity(), m_dt);
      steer = vnorm > Constants::EPSILON ? m_pidSteer.Update(Algebra2D::VecFromToAngleCCW(u, v2), m_dt) : 0;
    }
    else
    {
      m_pidVel.SetDesiredValue(RandomUniformReal(0.5, 1.0) * DesiredVel);
      force = m_pidVel.Update(GetVelocity(), m_dt);
      steer = vnorm > Constants::EPSILON ? m_pidSteer.Update(Algebra2D::VecFromToAngleCCW(u, v1), m_dt) : 0;
    }

    if(steer - prevSteer > m_maxSteeringStep)
      	steer = m_maxSteeringStep;
    else if(prevSteer - steer > m_maxSteeringStep)
      	steer = -m_maxSteeringStep;

    if(force > m_maxEngineForce)
        force = m_maxEngineForce;
    else if(force < -m_maxEngineForce)
        force = -m_maxEngineForce;
        // printf("force: %f; steer: %f\n",force, steer);

  	m_currControl[0] = force;
  	m_currControl[1] = 0;
  	m_currControl[2] = steer;
  	SetControl(m_currControl);
  }

  void MPBVehicleSimulator::SteerToPosition(const double target[])
  {
    btScalar yaw, pitch, roll;
    btMatrix3x3 mat = btMatrix3x3(m_carChassis->getWorldTransform().getBasis());
    mat.getEulerYPR(yaw, pitch, roll);
    double stateTheta = yaw;

    const double    prevSteer = m_vehicle->getSteeringValue(0);
  	const btVector3 pos       = m_carChassis->getWorldTransform().getOrigin();
  	const btVector3 linvel    = m_carChassis->getLinearVelocity();

  	const double    u[2]      = {target[0] - pos[0], target[1] - pos[1]};
  	const double    v[2]      = {linvel[0], linvel[1]};
    const double    r[2]      = {cos(stateTheta),sin(stateTheta)};

    const double    v1[2]      = {cos(stateTheta+prevSteer),sin(stateTheta+prevSteer)};
    const double    v2[2]      = {cos(stateTheta-prevSteer),sin(stateTheta-prevSteer)};

    // const double    c[2]      = {-linvel[0], linvel[1]};
  	const double    vnorm     = v[0] * v[0] + v[1] * v[1];
    const double angleCurrTarget = Algebra2D::AngleNormalize(Algebra2D::VecFromToAngleCCW(r,u),0);

    double steer;
    double force;

    if ((angleCurrTarget > 0.75 * M_PI ) && (angleCurrTarget < 1.25 * M_PI)
    && (Algebra2D::PointDist(GetCfg(),target)<15) )
    {
      m_pidVel.SetDesiredValue(RandomUniformReal(0.5, 1.0) * m_minVel);
      force = m_pidVel.Update(-GetVelocity(), m_dt);
      steer = vnorm > Constants::EPSILON ? m_pidSteer.Update(Algebra2D::VecFromToAngleCCW(u, v2), m_dt) : 0;
    }
    else
    {
      m_pidVel.SetDesiredValue(RandomUniformReal(0.5, 1.0) * m_maxVel);
      force = m_pidVel.Update(GetVelocity(), m_dt);
      steer = vnorm > Constants::EPSILON ? m_pidSteer.Update(Algebra2D::VecFromToAngleCCW(u, v1), m_dt) : 0;
    }

    if(steer - prevSteer > m_maxSteeringStep)
      	steer = m_maxSteeringStep;
    else if(prevSteer - steer > m_maxSteeringStep)
      	steer = -m_maxSteeringStep;

    if(force > m_maxEngineForce)
        force = m_maxEngineForce;
    else if(force < -m_maxEngineForce)
        force = -m_maxEngineForce;
        // printf("force: %f; steer: %f\n",force, steer);

  	m_currControl[0] = force;
  	m_currControl[1] = 0;
  	m_currControl[2] = steer;
  	SetControl(m_currControl);
  }

  void MPBVehicleSimulator::StopVehicle()
  {

  }

  void MPBVehicleSimulator::SetControl(const double * const u)
  {
    const double  engineForce   = m_currControl[0] = u[0];
  	const double  breakingForce = m_currControl[1] = u[1];
  	const double  steering      = m_currControl[2] = u[2];

         double         us            = steering + m_vehicle->getSteeringValue(0);

  	if(us > m_maxSteering)
  	    us = m_maxSteering;
  	else if(us < -m_maxSteering)
  	    us = -m_maxSteering;

  	int wheelIndex = 2;
  	m_vehicle->applyEngineForce(engineForce,wheelIndex);
  	m_vehicle->setBrake(breakingForce,wheelIndex);
  	wheelIndex = 3;
  	m_vehicle->applyEngineForce(engineForce,wheelIndex);
  	m_vehicle->setBrake(breakingForce,wheelIndex);

  	wheelIndex = 0;
  	m_vehicle->setSteeringValue(us, wheelIndex);
  	wheelIndex = 1;
  	m_vehicle->setSteeringValue(us, wheelIndex);
  }

  double MPBVehicleSimulator::GetVelocity(void) const
  {
    btScalar yaw, pitch, roll;
    btMatrix3x3 mat = btMatrix3x3(m_carChassis->getWorldTransform().getBasis());
    mat.getEulerYPR(yaw, pitch, roll);
    double stateTheta = yaw;
    const double    r[2]      = {cos(stateTheta),sin(stateTheta)};
    const btVector3 v    = m_carChassis->getLinearVelocity();
    const double    vv[2]      = {v[0], v[1]};

    const double angle = Algebra2D::AngleNormalize(Algebra2D::VecFromToAngleCCW(r,vv),0);
    double dir = cos(angle)/abs(cos(angle));

    // return dir * sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

  }

  void MPBVehicleSimulator::SampleCfg(double cfg[])
  {
    cfg[0] = RandomUniformReal(m_scene->m_grid.GetMin()[0], m_scene->m_grid.GetMax()[0]);
    cfg[1] = RandomUniformReal(m_scene->m_grid.GetMin()[1], m_scene->m_grid.GetMax()[1]);
    cfg[2] = RandomUniformReal(-M_PI, M_PI);
  }

  double MPBVehicleSimulator::RobotDistanceFromTriMesh(TriMesh * const tmesh, const double TRmesh[])
  {
    double TR3[Algebra3D::TransRot_NR_ENTRIES];

    const btTransform t    = m_carChassis->getWorldTransform();
    const btVector3   orig = t.getOrigin();
    const btMatrix3x3 rot  = t.getBasis();

    TR3[0]  = orig[0];
    TR3[1]  = orig[1];
    TR3[2]  = orig[2];
    TR3[3]  = rot[0][0];
    TR3[4]  = rot[0][1];
    TR3[5]  = rot[0][2];
    TR3[6]  = rot[1][0];
    TR3[7]  = rot[1][1];
    TR3[8]  = rot[1][2];
    TR3[9]  = rot[2][0];
    TR3[10] = rot[2][1];
    TR3[11] = rot[2][2];

    return
    m_projShape.Distance(TR3, &TR3[Algebra3D::Trans_NR_ENTRIES],
      tmesh, TRmesh ? TRmesh : NULL,
      TRmesh ? &TRmesh[Algebra3D::Trans_NR_ENTRIES] : NULL);
  }

  bool MPBVehicleSimulator::IsStateValid(void)
  {
    double TR3[Algebra3D::TransRot_NR_ENTRIES];

    const btTransform t    = m_carChassis->getWorldTransform();
    const btVector3   orig = t.getOrigin();
    const btMatrix3x3 rot  = t.getBasis();

    TR3[0]  = orig[0];
    TR3[1]  = orig[1];
    TR3[2]  = orig[2];
    TR3[3]  = rot[0][0];
    TR3[4]  = rot[0][1];
    TR3[5]  = rot[0][2];
    TR3[6]  = rot[1][0];
    TR3[7]  = rot[1][1];
    TR3[8]  = rot[1][2];
    TR3[9]  = rot[2][0];
    TR3[10] = rot[2][1];
    TR3[11] = rot[2][2];

    return
    //	    vnorm <= (30) &&
    TR3[0] >= m_scene->m_grid.GetMin()[0] &&
    TR3[1] >= m_scene->m_grid.GetMin()[1] &&
    TR3[0] <= m_scene->m_grid.GetMax()[0] &&
    TR3[1] <= m_scene->m_grid.GetMax()[1] &&
    !CollidesWithObstacles(&m_projShape, TR3);
  }

  void MPBVehicleSimulator::GetCfg(double cfg[])
  {
    btVector3       orig = m_carChassis->getWorldTransform().getOrigin();
    const btVector3 v    = m_carChassis->getLinearVelocity();

    cfg[0] = orig[0];
    cfg[1] = orig[1];
    cfg[2] = Algebra2D::AngleNormalize(atan2(v[1], v[0]), -M_PI);
  }

  double* MPBVehicleSimulator::GetCfg(void)
  {
    double *cfg;
    cfg = NewCfg();
    btVector3       orig = m_carChassis->getWorldTransform().getOrigin();
    const btVector3 v    = m_carChassis->getLinearVelocity();

    cfg[0] = orig[0];
    cfg[1] = orig[1];
    cfg[2] = Algebra2D::AngleNormalize(atan2(v[1], v[0]), -M_PI);
    return cfg;
  }

}
