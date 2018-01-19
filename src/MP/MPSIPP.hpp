#ifndef MP_SIPP_HPP_
#define MP_SIPP_HPP_

#include "MPMultiSearch.hpp"
#include <algorithm>    // std::random_shuffle

namespace MP
{
  class MPSIPP : public MPMultiSearch
  {
  public:
    MPSIPP(void);

    virtual ~MPSIPP(void)
    {
    }

    virtual bool RunSearch(std::vector<int> inits);

    virtual bool SIPP(std::vector<int> inits);
    virtual void SetPathToGoal(int rid, std::vector<int> path);
    virtual void UpdateIntervals();
    virtual bool isSolved();
    virtual void ReservePath(std::vector<int> path);
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
    std::vector<std::vector<std::pair<int,int>>> m_safeInterval;

    class SIPPSearch : public GraphSearchInfo<std::vector<int>>
    {
    public:
      SIPPSearch(void) : GraphSearchInfo<std::vector<int>>()
      {
      }

      void GetOutEdges(const std::vector<int> u,
        std::vector<std::vector<int>> * const edges,
        std::vector<double> * const costs = NULL) const;

      bool IsGoal(std::vector<int> key) const;
      double HeuristicCostToGoal(std::vector<int> u) const;
      virtual void SetAbstraction(MPAbstraction *abstract)
      {
        m_abstract = abstract;
        m_nrRobot = m_abstract->m_nrRobot;
      }
      bool CheckWithReserTable(std::vector<int> from, std::vector<int> to) const;

      int                           m_ridGoal;
      MPAbstraction                *m_abstract;
      std::vector<int>              m_inits;
      int                           m_nrRobot;
      int                           m_rid;
      std::vector<std::vector<std::pair<int,int>>> m_safeInterval;
      UseMap(int, SpaceTimeNode*)   m_reservetable;


    };

  };
}
#endif
