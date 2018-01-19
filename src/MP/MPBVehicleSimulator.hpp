#ifndef MP_BVEHICLE_SIMULATOR_HPP_
#define MP_BVEHICLE_SIMULATOR_HPP_

#include "MP/MPBSimulator.hpp"
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"
#include "Utils/Algebra.hpp"
#include "Utils/Misc.hpp"
#include "Utils/PIDController.hpp"

namespace MP
{
  class MPBVehicleState : public MPState
  {
  public:
    MPBVehicleState(void) : MPState()
    {
      m_wheels[0] = m_wheels[1] = m_wheels[2] = m_wheels[3] = NULL;
    }

    virtual ~MPBVehicleState(void)
    {
      if(m_wheels[0]) delete m_wheels[0];
      if(m_wheels[1]) delete m_wheels[1];
      if(m_wheels[2]) delete m_wheels[2];
      if(m_wheels[3]) delete m_wheels[3];
    }

    btTransform   m_transform;
    btVector3     m_linearVel;
    btVector3     m_angularVel;
    btScalar      m_steering[4];
    btWheelInfo  *m_wheels[4];
  };

  class MPBVehicleSimulator : public MPBSimulator
  {
  public:
    MPBVehicleSimulator(void);

    virtual ~MPBVehicleSimulator(void)
    {
    }

    virtual void CompleteSetup(void);

    virtual int GetDimControl(void) const
    {
      return 3;
    }
    virtual void CopyState(MPState * const dest,
           const MPState * const src) const;
    virtual void SetState(const MPState * const s);
    virtual void GetState(MPState * const s);
    virtual void GetControl(double * const u);
  	virtual void SetControl(const double * const u);
    virtual void GetCfg(double cfg[]);

    virtual void DrawState(MPState * const vs);
    virtual void Draw();
    virtual MPState* NewState(void) const
  	{
  	    return new MPBVehicleState();
  	}

    virtual void StartSteerToPosition(void);
    virtual void StartSteerToStop(void);
    virtual void SteerToPosition(const double target[]);
    virtual void SteerToStop(const double target[]);

    virtual int GetDimPos(void) const
    {
      return 2;
    }

    virtual int GetDimCfg(void) const
    {
        return GetDimPos() + 1;
    }
    virtual void SampleCfg(double cfg[]);
    virtual double RobotDistanceFromTriMesh(TriMesh * const tmesh, const double TR3[]);
    virtual bool IsStateValid(void);
    virtual void SetStateFromCfg(double cfg[]);

    virtual void SampleControl(void)
    {
      m_currControl[0] = RandomUniformReal(0.5*m_maxEngineForce, m_maxEngineForce);
    	m_currControl[1] = 0;
    	m_currControl[2] = RandomUniformReal(-m_maxSteering, m_maxSteering);
    }

    virtual double* GetCfg(void);
    virtual double GetVelocity(void) const;

    virtual void StopVehicle();


    btRigidBody                        *m_carChassis;
  	btRaycastVehicle::btVehicleTuning   m_tuning;
  	btVehicleRaycaster                 *m_vehicleRayCaster;
  	btRaycastVehicle	                 *m_vehicle;

  	double m_maxEngineForce;
  	double m_maxBreakingForce;
  	double m_maxSteering;
  	double m_maxSteeringStep;
  	double m_maxVelocity;
    double m_minVel;
    double m_maxVel;
    double m_minAcc;
    double m_maxAcc;

  	TriMesh m_carChassisMesh;
  	TriMesh m_wheelMesh;

    double m_wheelCylRadius;
    double m_wheelThickness;

    double           *m_currControl;
    MPBVehicleState  *m_currState;
    PIDController m_pidSteer;
  	PIDController m_pidVel;
    PIDController m_pidLoc;
  };
}

#endif
