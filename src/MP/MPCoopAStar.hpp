#ifndef MP_COOP_ASTAR_HPP_
#define MP_COOP_ASTAR_HPP_

#include "MPMultiSearch.hpp"
#include <algorithm>    // std::random_shuffle

namespace MP
{
  class MPCoopAStar : public MPMultiSearch
  {
  public:
    MPCoopAStar(void);

    virtual ~MPCoopAStar(void)
    {
    }

    virtual bool RunSearch(std::vector<int> inits);
    virtual bool CoopAStar(std::vector<int> inits);

    virtual void PermuteRobotOrder()
    {
      int nrshuffle = RandomUniformInteger(3, 5);
      for(int i = 0; i < nrshuffle; i++)
      {
        std::random_shuffle ( m_robotOrder.begin(), m_robotOrder.end() );
      }
    }

    virtual void ClearReservationTable(void)
    {
      m_reservetable.clear();
    }

    virtual bool isSolved();

    virtual void ReservePath(std::vector<int> path);

    virtual void SetPathToGoal(int rid, std::vector<int> path)
    {
      m_pathsToGoal[rid].clear();
      m_pathsToGoal[rid] = path;
      // ReservePath(m_pathsToGoal[rid]);
    }
    virtual void CompletePaths();


    struct SpaceTimeNode
    {
      SpaceTimeNode(void) :
      m_timeid(-1)
      {
      }
      int               m_timeid;
      std::vector<int>  m_reserNodeTo;
      std::vector<int>  m_reserNodeFrom;
      int               m_reserveSize;
    };

    UseMap(int, SpaceTimeNode*)   m_reservetable;

    std::vector<int>  m_robotOrder;

    class CoopSearch : public GraphSearchInfo<std::vector<int>>
    {
    public:
      CoopSearch(void) : GraphSearchInfo<std::vector<int>>()
      {
        m_depth = 0;
      }

      void GetOutEdges(const std::vector<int> u,
        std::vector<std::vector<int>> * const edges,
        std::vector<double> * const costs = NULL) const;

      bool IsGoal(std::vector<int> key) const;

      double HeuristicCostToGoal(std::vector<int> u) const;

      bool CheckWithReserTable(std::vector<int> from, std::vector<int> to) const;

      virtual void SetAbstraction(MPAbstraction *abstract)
      {
        m_abstract = abstract;
        m_nrRobot = m_abstract->m_nrRobot;
      }

      int                           m_ridGoal;
      int                           m_depth;
      UseMap(int, SpaceTimeNode*)   m_reservetable;
      MPAbstraction                *m_abstract;
      std::vector<int>              m_inits;
      int                           m_nrRobot;
      int                           m_rid;
    };

  };
}
#endif
