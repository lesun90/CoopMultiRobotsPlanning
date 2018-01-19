#ifndef MP_MPRobotCar_HPP_
#define MP_MPRobotCar_HPP_

#include "MP/MPSimulator.hpp"
#include "Utils/PIDController.hpp"
#include "Utils/TriMesh.hpp"
#include "External/PQP/PQPTriMesh.hpp"

namespace MP
{

  class MPCarState : public MPState
  {
  public:
    MPCarState(void) : MPState(),
    m_state(NULL)
    {

    }

    virtual ~MPCarState(void)
    {
      if(m_state)
      delete [] m_state;
    }

    double *m_state;
  };

  class MPRobotCar : public MPSimulator
  {
  public:

    enum
    {
      STATE_X           = 0,
      STATE_Y           = 1,
      STATE_THETA       = 2,
      STATE_VEL         = 3,
      STATE_STEER_ANGLE = 4,
      CONTROL_ACC       = 0,
      CONTROL_STEER_VEL = 1
    };


    MPRobotCar(void);
    virtual ~MPRobotCar(void)
    {
      m_allocatorControl.Delete(m_currControl);
    }

    virtual void CompleteSetup(void);
    virtual void DrawState(MPState * const vs) ;
    virtual void Draw()
    {
      DrawState(m_currState);
    }

    virtual int GetDimControl(void) const
    {
      return 2;
    }

    virtual MPState* NewState(void) const
    {
      MPCarState *s = new MPCarState();
      s->m_state = new double[GetDimState()];
      for (int i = 0 ; i < GetDimState(); i++)
      {
        s->m_state[i] = 0 ;
      }
      return s;
    }

    virtual int GetDimState(void) const
    {
      return 5;
    }

    virtual void SimulateOneStep(void);
    virtual double TimeStepConstantAcceleration(const double v,const double a,const double d) const;
    virtual double TimeStepConstantVelocity(const double v, const double d) const;
    virtual double GetAcceleration(void) const
    {
      return m_currControl[CONTROL_ACC];
    }
    virtual double GetVelocity(void) const
    {
      return dynamic_cast<MPCarState*>(m_currState)->m_state[STATE_VEL];
    }
    virtual void ClampState(double s[]) const;
    virtual void MotionEqs(const double s[],const double t, const double u[], double ds[]);
    virtual void StartSteerToPosition(void);
    virtual void StartSteerToStop(void);
    virtual void SteerToPosition(const double target[]);
    virtual void SteerToStop(const double target[]);

    virtual void CopyState(MPState * const dest,
  			       const MPState * const src) const
  	{
  	    CopyArray<double>(dynamic_cast<MPCarState*>(dest)->m_state,
  			      GetDimState(),
  			      dynamic_cast<const MPCarState*>(src)->m_state);
  	}

    virtual void SetState(const MPState * const s)
    {
      if(s != m_currState)
      CopyState(m_currState, s);
    }

    virtual void GetState(MPState * const s)
  	{
  	    CopyState(s, m_currState);
  	}

    virtual void SetControl(const double * const u)
    {
      if(u != m_currControl)
      CopyControl(m_currControl, u);
    }

    virtual void GetControl(double * const u)
  	{
  	    CopyControl(u, m_currControl);
  	}

    virtual int GetDimPos(void) const
    {
      return 2;
    }

    virtual int GetDimCfg(void) const
  	{
  	    return GetDimPos() + 1;
  	}
    virtual void SampleCfg(double cfg[]);

    virtual double RobotDistanceFromTriMesh(TriMesh * const tmesh, const double TR3[])
    {
      double T3robot[Algebra3D::Trans_NR_ENTRIES];
      double R3robot[Algebra3D::Rot_NR_ENTRIES];

      T3robot[0] = m_currState->m_state[STATE_X];
      T3robot[1] = m_currState->m_state[STATE_Y];
      T3robot[2] = 0;
      Algebra3D::ZAxisAngleAsRot(m_currState->m_state[STATE_THETA], R3robot);
      //robot-obstacles

      return m_projShape.Distance(T3robot, R3robot, &m_scene->m_obstacles.m_tmesh,
        TR3 ? TR3 : NULL, TR3 ? &TR3[Algebra3D::Trans_NR_ENTRIES] : NULL);
    }

    virtual bool IsStateValid(void);
    virtual void GetCfg(double cfg[]);
    virtual void SetStateFromCfg(double cfg[])
    {
      MPCarState s;
      s.m_state = new double[GetDimState()];
      for (int i = 0 ; i < GetDimCfg(); i++)
      {
        s.m_state[i] = 0;
      }
      s.m_state[0] = cfg[0];
      s.m_state[1] = cfg[1];
      s.m_state[2] = cfg[2];
      SetState(&s);
    }

    virtual void SampleControl(void)
    {
      m_currControl[CONTROL_ACC] = RandomUniformReal(m_minAcc, m_maxAcc);
      m_currControl[CONTROL_STEER_VEL] = RandomUniformReal(m_minSteerVel, m_maxSteerVel);
    }

    double* GetCfg(void);
    virtual void StopVehicle();

    double m_minSteerAngle;
    double m_maxSteerAngle;
    double m_minVel;
    double m_maxVel;
    double m_minAcc;
    double m_maxAcc;
    double m_minSteerVel;
    double m_maxSteerVel;

    double m_wheelCylRadius;
    double m_wheelThickness;
    TriMesh    m_tmeshDrawBody;
    TriMesh    m_tmeshDrawWheel;
    PIDController m_pidSteer;
    PIDController m_pidVel;
    PIDController m_pidLoc;

    MPCarState       *m_currState;
    double           *m_currControl;


  };
}

#endif
