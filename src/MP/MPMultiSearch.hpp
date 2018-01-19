#ifndef MP_MULTI_SEARCH_HPP_
#define MP_MULTI_SEARCH_HPP_

#include "Utils/GraphSearch.hpp"
#include "MP/MPAbstraction.hpp"
#include "MP/MPTriAbstraction.hpp"
#include "MP/MPPrmAbstraction.hpp"
#include "vector"
#include "Utils/Timer.hpp"

namespace MP
{
  class MPMultiSearch
  {
  public:
    MPMultiSearch(void);

    virtual ~MPMultiSearch(void)
    {
    }

    virtual void SetInits(std::vector<int> inits)
    {
      m_inits = inits;
    }
    virtual void SetGoals(std::vector<int> goals)
    {
      m_goals = goals;
    }

    virtual void SetAbstraction(MPAbstraction *abstract)
    {
      m_abstract = abstract;
      m_nrRobot = m_abstract->m_nrRobot;
      m_goals.resize(m_nrRobot);
      m_inits.resize(m_nrRobot);
    }

    virtual void CompleteSetup(void)
    {
      m_pathsToGoal.resize(m_nrRobot);
    }

    virtual void ClearPaths(void)
    {
      for (int r = 0 ; r<m_nrRobot; r++)
      {
        m_pathsToGoal[r].clear();
      }
    }

    virtual bool isSolved()
    {
      for (int r = 0 ; r < m_nrRobot; r++)
      {
        // printf("%d %d\n",m_pathsToGoal[r].back(),m_goals[r] );
        if (m_pathsToGoal[r].size()==0)
        {
          return false;
        }
        if (m_pathsToGoal[r].back() != m_goals[r])
        {
          return false;
        }
      }
      return true;
    }

    virtual void PrintPaths()
    {
      for (int r = 0 ; r < m_nrRobot; r++)
      {
        printf("robot %d\n  ",r );
        for (int i = 0 ; i < m_pathsToGoal[r].size(); i++)
        {
          printf("%d, ",m_pathsToGoal[r][i] );
        }
        printf("\n");
      }
    }


    virtual bool RunSearch(std::vector<int> inits) = 0;

    virtual void Clear()
    {
      ClearPaths();
      m_weight = HUGE_VAL;
    }
    virtual void DrawPaths(void)
    {
      const bool is2D = GDrawIs2D();

      GDrawPushTransformation();
      GDrawMultTrans(0, 0, 0.9);
      GDraw2D();

      for (int r = 0; r < m_pathsToGoal.size(); r++)
      {
        for (int i = 0; i < m_pathsToGoal[r].size(); i++)
        {
          GDrawLineWidth(100);
          GDrawIndexColor(r);
          GDrawCircle2D(m_abstract->m_regions[m_pathsToGoal[r][i]]->m_cfg[0],
            m_abstract->m_regions[m_pathsToGoal[r][i]]->m_cfg[1], 0.5);
          if(i>0)
          {
              GDrawSegment2D(m_abstract->m_regions[m_pathsToGoal[r][i]]->m_cfg,
                m_abstract->m_regions[m_pathsToGoal[r][i-1]]->m_cfg);
          }
        }
      }
      GDrawPopTransformation();

      if(!is2D)
      GDraw3D();
    }

    double GetPathsCost()
    {
      if (isSolved() == false)
      {
        return HUGE_VAL;
      }
      double tcost = 0;
      for (int r = 0; r < m_nrRobot; r++)
      {
        if (m_pathsToGoal[r].size() == 0)
        {
          continue;
        }
        double costeach = 0;
        for (int i = 0 ; i < m_pathsToGoal[r].size()-1; i++)
        {
          int fr = m_pathsToGoal[r][i];
          int tr = m_pathsToGoal[r][i+1];
          costeach += m_abstract->Dist2Region(fr,tr);
        }
        costeach = costeach*costeach;
        //costeach = costeach;

        tcost = tcost + costeach;
      }
      m_weight = tcost;
      return m_weight;
      // printf("m_weight %f\n",m_weight );
    }

    std::vector<int>              m_inits;
    std::vector<int>              m_goals;
    std::vector<std::vector<int>> m_pathsToGoal;
    MPAbstraction                *m_abstract;
    int                           m_nrRobot;
    double                        m_weight;
    int                           m_depth;
    std::vector<int>              m_robotOrder;


  };
}
#endif
