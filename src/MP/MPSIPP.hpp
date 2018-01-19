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

    virtual void SIPP(std::vector<int> inits);
    virtual void SetPathToGoal(int rid, std::vector<int> path);
    virtual void UpdateIntervals();
    virtual bool isSolved();
    virtual void CompletePaths();

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
      bool CheckMovement(int from, int to, int timeid) const;

      int                           m_ridGoal;
      MPAbstraction                *m_abstract;
      std::vector<int>              m_inits;
      int                           m_nrRobot;
      int                           m_rid;
      int                           m_depth;
      std::vector<std::vector<std::pair<int,int>>> m_safeInterval;
      std::vector<std::vector<int>> m_pathsToGoal;

    };

  };
}
#endif
