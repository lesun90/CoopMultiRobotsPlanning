#ifndef MP_SIMULATOR_HPP_
#define MP_SIMULATOR_HPP_

#include "MP/MPScene.hpp"
#include "MP/Constants.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Allocator.hpp"
#include "Utils/Algebra.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/Timer.hpp"

namespace MP
{
  class MPState
  {
  public:
    MPState(void)
    {
    }

    virtual ~MPState(void)
    {
    }
  };

  class MPSimulator
  {
  public:
    MPSimulator(void);

    virtual ~MPSimulator(void);

    virtual void CompleteSetup(void)
    {
    }

    virtual void SetScene(MPScene *scene)
    {
      m_scene = scene;
    }

    virtual MPScene* GetScene(void)
    {
      return m_scene;
    }

    virtual int GetDimControl(void) const = 0;
    virtual double* NewControl(void) const
    {
      return new double[GetDimControl()];
    }
    virtual void DeleteControl(double * const u) const
    {
      if(u)
      delete[] u;
    }

    virtual MPState* NewState(void) const = 0;

    virtual void DrawCfgShape(const double cfg[]);
    virtual void DrawState(MPState * const vs) = 0;
    virtual void Draw(void) = 0;

    virtual void SimulateOneStep(void) = 0;
    virtual void SetState(const MPState * const s) = 0;
    virtual void SetStateFromCfg(double cfg[]) = 0;
    virtual void GetState(MPState * const s) = 0;
    virtual void SetControl(const double * const u) = 0;
    virtual void GetControl(double * const u) = 0;
    virtual void GetCfg(double cfg[]) = 0;
    virtual void StartSteerToPosition() = 0;
    virtual void StartSteerToStop(void) = 0;
    virtual void SteerToPosition(const double target[]) = 0;
    virtual void SteerToStop(const double target[]) = 0;
    virtual void StopVehicle() = 0;

    virtual void CopyState(MPState * const dest, const MPState * const src) const = 0;
  	virtual MPState* CopyState(const MPState * const src) const
  	{
  	    MPState *dest = NewState();
  	    CopyState(dest, src);
  	    return dest;
  	}
    virtual void CopyControl(double * const dest, const double * const src) const
  	{
  	    CopyArray<double>(dest, GetDimControl(), src);
  	}
  	virtual double* CopyControl(const double * const src) const
  	{
  	    double *dest = NewControl();
  	    CopyControl(dest, src);
  	    return dest;
  	}

    virtual int GetDimPos(void) const = 0;
    virtual int GetDimCfg(void) const = 0;

    virtual double* NewCfg(void) const
  	{
  	    return new double[GetDimCfg()];
  	}

    virtual void SampleCfg(double cfg[]) = 0;
    virtual void CopyCfg(double dest[], const double src[]) const
  	{
  	    CopyArray<double>(dest, GetDimCfg(), src);
  	}
    virtual double DistanceCfg(const double cfg1[], const double cfg2[]) const
    {
      return Algebra::PointDist(GetDimCfg(), cfg1, cfg2);
    }

    virtual double MinDistFromCfgToObstacles(double cfg[])
    {
      double T3robot[Algebra3D::Trans_NR_ENTRIES];
      double R3robot[Algebra3D::Rot_NR_ENTRIES];

      T3robot[0] = cfg[0];
      T3robot[1] = cfg[1];
      T3robot[2] = 0;
      Algebra3D::ZAxisAngleAsRot(cfg[2], R3robot);
      //robot-obstacles
      double d = m_projShape.Distance(T3robot, R3robot, &m_scene->m_obstacles.m_tmesh, NULL, NULL);

      return d;

    }

    virtual bool CollidesWithObstacles(TriMesh * const tmesh, const double TR3[])
    {
      return m_scene->m_obstacles.m_tmesh.Collision(NULL, NULL, tmesh, TR3, TR3 == NULL ? NULL : &TR3[Algebra3D::Trans_NR_ENTRIES]);
    }

    virtual bool IsStateValid(void) = 0;

    virtual bool IsCfgValid(double cfg[]);
    virtual bool IsCfgValid(void);

    virtual bool IsGoalReached(const double cfg[])
    {
      return Algebra2D::PointDist(m_goal, cfg) < m_goal[2];
    }

    virtual bool IsGoalReached(void)
    {
      return Algebra2D::PointDist(m_goal, GetCfg()) < 1.1*m_goal[2];
    }

    virtual double* GetCfg(void) = 0;

    virtual bool HasReachedPosition(const double target[], const double dtol)
    {
      double *cfg = new double[GetDimCfg()];
      GetCfg(cfg);
      return Algebra2D::PointDistSquared(target, cfg) <= dtol * dtol;
    }

    virtual void SampleControl(void) = 0;

    virtual void SetGoal(const double s[])
    {
      m_goal[0] = s[0];
      m_goal[1] = s[1];
      m_goal[2] = s[2];
    }

    MPScene          *m_scene;
    Allocator<double> m_allocatorControl;
    double            m_dt;
    double            m_minDistOneStep;
    double            m_maxDistOneStep;
    double            m_goal[3];
    int               m_id;
    bool              m_cfgUseRot;

    double m_length;
    double m_width;
    double m_height;

    double m_bodyLength;
    double m_bodyWidth;
    double m_bodyHeight;

    PQPTriMesh m_tmeshCollision;
    PQPTriMesh m_projShape;


  };
}

#endif
