#ifndef MP_ROBOT_PLANNER_HPP_
#define MP_ROBOT_PLANNER_HPP_

#include "MP/MPScene.hpp"
#include "MP/MPSimulator.hpp"
#include "MP/MPRobotCar.hpp"
#include "MP/MPBVehicleSimulator.hpp"
#include "MP/MPAbstraction.hpp"
#include "Utils/GraphSearch.hpp"
#include "Utils/Timer.hpp"
#include <stack>          // std::stack

namespace MP
{
  class RobotPlanner
  {
  public:
    enum Type
    {
      PRM     = 0,
      TRI     = 1,
      RCAR    = 0,
      RBULL    = 1,
      ASTAR   = 0,
      PAS     = 1
    };

    RobotPlanner(MPAbstraction *abstract, int id, int type);

    virtual ~RobotPlanner()
    {
      if(m_sim)
      delete m_sim;
      if(m_abstract)
      delete m_abstract;
    }

    virtual int GetGoalRegion(void)
    {
      return m_goalRid;
    }

    virtual int GetCurrentRegion(void)
    {
      return m_abstract->LocateRegion(m_sim->GetCfg());
    }

    virtual void SetCurrentRegion(int region)
    {
      m_sim->SetStateFromCfg(m_abstract->m_regions[region]->m_cfg);
    }

    virtual void SetAbstraction(MPAbstraction *abstract)
    {
      m_abstract = abstract;
    }

    virtual int GetNextTargetRegion(void)
    {
      return m_abstract->LocateRegion(m_target);
    }

    virtual int IsReachedNextTargetRegion(void)
    {
      return GetNextTargetRegion() == GetCurrentRegion();
    }

    virtual double GetDistToGoalAtRegion(int region)
    {
      return m_abstract->m_regions[region]->m_distToGoals[m_id];
    }

    virtual void GetPathToGoalAtRegion(int region, std::vector<int> &path)
    {
      path.clear();
      path = m_abstract->m_regions[region]->m_pathsToGoal[m_id];
    }

    virtual double GetPathToGoalSizeAtRegion(int region)
    {
      return m_abstract->m_regions[region]->m_pathsToGoal[m_id].size();
    }

    virtual bool ReachedGoalRegion(void)
    {
      return GetCurrentRegion() == GetGoalRegion();
    }

    virtual bool HasReachGoal(void)
    {
      return m_sim->IsGoalReached();
    }

    virtual bool HasReachRegion(int region)
    {
      return GetCurrentRegion() == region;
    }

    virtual bool HasReachRegion(int region, double tol)
    {
      double d = Algebra2D::PointDistSquared(m_sim->GetCfg(),m_abstract->m_regions[region]->m_cfg);
      return d < tol*tol;
    }

    virtual bool HasReachNextTargetRegion(void)
    {
      // double d = Algebra2D::PointDist(m_sim->m_currState,m_abstract->m_regions[GetNextTargetRegion()]->m_cfg);
      // return d < m_tolReachregion;
      return GetCurrentRegion() == m_toRegion;
    }

    virtual void SampleCfgAtGoal(double s[])
    {
      SampleRandomPointInsideCircle2D(m_sim->m_goal, m_sim->m_goal[2], s);
    }

    virtual void SampleNextTargetAtGoal(void)
    {
      SampleRandomPointInsideCircle2D(m_sim->m_goal, m_sim->m_goal[2], m_target);
    }

    virtual void SampleRandomTarget(void)
    {
      m_sim->SampleCfg(m_target);
    }

    virtual double GetCfgPathCost(void)
    {
      double pcost = 0;
      for (int i = 0 ; i < m_cfgPath.size()-1;i++)
      {
        pcost += m_sim->DistanceCfg(m_cfgPath[i],m_cfgPath[i+1]);
      }
      m_cfgPathCost = pcost;
      return pcost;
    }

    virtual void DrawNextTarget(void)
    {
      const bool is2D = GDrawIs2D();

      GDrawPushTransformation();
      GDrawMultTrans(0, 0, m_abstract->m_scene->m_maxGroundHeight+0.3);
      GDraw2D();
      GDrawIndexColor(m_sim->m_id);
      GDrawCircle2D(m_target, 1.7);

      GDrawPopTransformation();

      if(!is2D)
      GDraw3D();
    }

    virtual void DrawDiscretePathToGoal(void);

    virtual void Draw(void);
    virtual void DrawGoal(void);
    virtual void DrawRobot(void);
    virtual void DrawAbstract(void);
    virtual void DrawCfgPath(void);
    virtual void SetRobotStateAtPos(int pos)
    {
      m_sim->SetState(m_statePath[pos]);
    }

    virtual void SetState(const MPState * const s)
    {
      m_sim->SetState(s);
    }

    virtual void SampleTargetInRegion(int region)
    {
      const double *wpt = m_abstract->m_regions[region]->m_cfg;
      if(RandomUniformReal() < 0.25)
      SampleRandomPointInsideCircle2D(wpt, m_tolReach, m_target);
      else
      SampleRandomPointInsideCircle2D(wpt, m_sampleradius, m_target);
    }

    virtual void ResetPathState(void)
    {
      m_currPathStep = 0;
      m_currPathToGoal.clear();
    }

    virtual void SetCurrPathToGoal(std::vector<int> path)
    {
      m_currPathStep = 0;
      m_currPathToGoal.clear();
      m_currPathToGoal = path;
    }

    virtual void GenerateTarget(void);
    virtual void GenerateRandomTarget(void);

    virtual void StartSteerToTarget(void);
    virtual void SteerToTarget(void);
    virtual bool HasReachTarget(void);
    virtual bool HasReachNextRegion(void);

    virtual void GetCurrentPath(const MPState * const s,std::vector<int> path);


    void DrawPathToGoal(void)
    {
      const bool is2D = GDrawIs2D();

      GDrawPushTransformation();
      GDrawMultTrans(0, 0, m_abstract->m_scene->m_maxGroundHeight+1.35);
      GDraw2D();
      GDrawIndexColor(m_sim->m_id);

      const int n = m_currPathToGoal.size();
      	for(int i = 0; i < n - 1; ++i)
      	{
      	    const double *p1   = m_abstract->m_regions[m_currPathToGoal[i]]->m_cfg;
      	    const double *p2   = m_abstract->m_regions[m_currPathToGoal[i+1]]->m_cfg;
      	    const double  d    = p1[2];
      	    const double  vx   = p2[0] - p1[0];
      	    const double  vy   = p2[1] - p1[1];
      	    const double  norm = sqrt(vx * vx + vy * vy);
      	    const double  ux   = -vy * d / norm;
      	    const double  uy   =  vx * d/ norm;
      	    GDrawSegment2D(p1, p2);
      	}

      	GDrawWireframe(true);
      	// for(int i = 0; i < n; ++i)
      	//     GDrawCircle2D(m_abstract->m_regions[m_currPathToGoal[i]]->m_cfg, m_tolReach);
      	GDrawWireframe(false);
        if (n>0)
        {
        GDrawCircle2D(m_target, 0.5);
        GDrawIndexColor(8);
        // GDrawCircle2D(m_abstract->m_regions[m_toRegion]->m_cfg, m_tolReach);
        }
      GDrawPopTransformation();

      if(!is2D)
      GDraw3D();
    }

    int                    m_id;
    int                    m_robotType;
    int                    m_goalRid;
    MPSimulator           *m_sim;
    MPAbstraction         *m_abstract;
    int                    m_abstractionType;
    std::vector<double*>   m_cfgPath;
    std::vector<int>       m_path;
    std::vector<MPState*>  m_statePath;
    double                 m_cfgPathCost;
    std::vector<int>       m_pathToGoal;
    double                *m_target;
    int                    m_currentRegion;
    double                 m_tolReach;
    double                 m_sampleradius;
    double                 m_tolReachregion;
    int                    m_nextToRegion;
    std::vector<int>       m_currPathToGoal;
    int                    m_currPathStep;
    int m_fromRegion;
    int m_toRegion;
    int m_waitAtRegion;
    bool m_wait;
    bool m_shouldStop;
    bool m_prepareToStop;
    std::vector<MPState*>  m_currStatePath;
    std::vector<double*>   m_currCfgPath;
    int m_needNrCfg;

  };
}

#endif
