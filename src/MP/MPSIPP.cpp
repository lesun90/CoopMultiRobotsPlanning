#include "MP/MPSIPP.hpp"
#include <algorithm>

namespace MP
{
  MPSIPP::MPSIPP()
  {
    printf("Use SIPP search\n");
  }

  bool MPSIPP::RunSearch(std::vector<int> inits)
  {
    m_robotOrder.clear();
    for (int r = 0; r < m_nrRobot; r++)
    {
      m_robotOrder.push_back(r);
    }

    int nrtries = RandomUniformInteger(std::max(1,m_nrRobot/2), m_nrRobot/2);
    Timer::Clock clk;
    Timer::Start(&clk);
    for (int iter = 0 ; iter < nrtries && isSolved() == false; iter++)
    {
      SIPP(inits);
    }
    if (isSolved() == false)
    {
      for (int r = 0 ; r < m_nrRobot; r++)
      {
        if (m_pathsToGoal[r].size()==0)
        {
          SetPathToGoal(r, m_abstract->m_regions[inits[r]]->m_pathsToGoal[r]);
        }
      }
    }

    Stats::GetSingleton()->AddValue("FindPathTime", Timer::Elapsed(&clk));
    Stats::GetSingleton()->AddValue("NrFindPath", 1);

    return isSolved();
  }

  void MPSIPP::SIPP(std::vector<int> inits)
  {
    ClearPaths();
    std::random_shuffle ( m_robotOrder.begin(), m_robotOrder.end() );
    int rid = m_robotOrder.front();
    SetPathToGoal(rid, m_abstract->m_regions[inits[rid]]->m_pathsToGoal[rid]);
    // printf("solved %d %d\n",rid, m_pathsToGoal[rid].size() );
    UpdateIntervals();
    for (int i = 1; i < m_nrRobot; i++)
    {
      rid = m_robotOrder[i];
      m_pathsToGoal[rid].clear();
      SIPPSearch                    m_SIPPsearch;
      GraphSearch<std::vector<int>> m_gs;
      const bool                    breakEarly = false;

      m_gs.m_info                   = &m_SIPPsearch;
      std::vector<int>              goalKey;
      std::vector<int>              initKey;
      std::vector<std::vector<int>> astarPath;
      m_SIPPsearch.SetAbstraction(m_abstract);
      m_SIPPsearch.m_ridGoal = m_goals[rid];
      m_SIPPsearch.m_rid = rid;
      m_SIPPsearch.m_safeInterval = m_safeInterval;
      m_SIPPsearch.m_inits = inits;
      m_SIPPsearch.m_pathsToGoal = m_pathsToGoal;

      initKey.push_back(inits[rid]);
      initKey.push_back(0);
      initKey.push_back(m_safeInterval[inits[rid]][0].first);
      initKey.push_back(m_safeInterval[inits[rid]][0].second);
      if(m_gs.AStar(initKey, breakEarly, &goalKey))
      {
        m_gs.GetPathFromStart(goalKey, &astarPath);
      }

      if (astarPath.size()>0)
      {
        // printf("solved %d\n",rid );
        m_pathsToGoal[rid].clear();
        for (int k = 0; k < astarPath.size()-1; ++k)
        {
          // printf("%d %d\n",astarPath[k][0],astarPath[k][1] );
          for(int o = 0; o < astarPath[k+1][1] - astarPath[k][1]; o++)
          {
            m_pathsToGoal[rid].push_back(astarPath[k][0]);
          }
        }
        m_pathsToGoal[rid].push_back(astarPath.back()[0]);
        UpdateIntervals();
      }
      else
      {
        break;
      }
    }
  }

  void MPSIPP::CompletePaths()
  {
    for (int r = 0 ; r < m_nrRobot; r++)
    {
      std::vector<int> pathToEnd = m_abstract->m_regions[m_pathsToGoal[r].back()]->m_pathsToGoal[r];
      if (pathToEnd.size() > 0)
      {
        m_pathsToGoal[r].insert(m_pathsToGoal[r].end(),pathToEnd.begin()+1,pathToEnd.end() );
      }
    }
  }

  void MPSIPP::SIPPSearch::GetOutEdges(const std::vector<int> u,
    std::vector<std::vector<int>>  * const edges,
    std::vector<double> * const costs) const
  {
    int s = u[0];
    // printf("s %d %d [%d %d]\n\n",s,u[1],u[2],u[3] );
    int t = u[1];
    const int n = m_abstract->m_regions[s]->m_neighs.size();
    for(int i = 0; i < n; ++i)
    {

      double cost     = m_abstract->m_regions[s]->m_dists[i];
      int cfg         = m_abstract->m_regions[s]->m_neighs[i];
      int startTime   = u[1] + 1;
      int endTime     = u[3] + 1;
      if (s == cfg)
      {
        continue;
      }
      for (int k = 0 ; k < m_safeInterval[cfg].size();k++)
      {
        // printf("cfg %d ",cfg);
        // printf("[%d %d]\n",m_safeInterval[cfg][k].first,m_safeInterval[cfg][k].second );
        if ((m_safeInterval[cfg][k].first > endTime) || (m_safeInterval[cfg][k].second < startTime))
        {
          // printf("not in interval at time %d\n",startTime);
          continue;
        }
        int t = startTime > m_safeInterval[cfg][k].first ? startTime : m_safeInterval[cfg][k].first;
        if (CheckMovement(s,cfg,t) == true)
        {
          if (k>0)
          {
            cost = cost * (m_safeInterval[cfg][k].first - m_safeInterval[cfg][k-1].second );
          }
          // printf("add %d at time %d\n",cfg, t);
          std::vector<int> neigh;
          neigh.push_back(cfg);
          neigh.push_back(t);
          neigh.push_back(m_safeInterval[cfg][k].first);
          neigh.push_back(m_safeInterval[cfg][k].second);
          edges->push_back(neigh);
          costs->push_back(cost);
        }
        else
        {
          // printf("not %d at time %d\n",cfg, t);
        }
      }
    }

    double cost     = m_abstract->MinCostToNeighbors(s)*0.99;
    int cfg         = s;
    int startTime   = u[1] + 1;
    int endTime     = u[2] + 1;
    for (int k = 0 ; k < m_safeInterval[cfg].size();k++)
    {
      // printf("cfg %d ",cfg);
      // printf("[%d %d]\n",m_safeInterval[cfg][k].first,m_safeInterval[cfg][k].second );
      if ((m_safeInterval[cfg][k].first > endTime) || (m_safeInterval[cfg][k].second < startTime))
      {
        // printf("not in interval at time %d\n",startTime);
        continue;
      }
      int t = startTime > m_safeInterval[cfg][k].first ? startTime : m_safeInterval[cfg][k].first;
      if (CheckMovement(s,cfg,t) == true)
      {
        if (k>0)
        {
          cost = cost * (m_safeInterval[cfg][k].first - m_safeInterval[cfg][k-1].second );
        }
        // printf("add %d at time %d\n",cfg, t);
        std::vector<int> neigh;
        neigh.push_back(cfg);
        neigh.push_back(t);
        neigh.push_back(m_safeInterval[cfg][k].first);
        neigh.push_back(m_safeInterval[cfg][k].second);
        edges->push_back(neigh);
        costs->push_back(cost);
      }
      else
      {
        // printf("not %d at time %d\n",cfg, t);
      }
    }

    // getchar();
  }

  double MPSIPP::SIPPSearch::HeuristicCostToGoal(const std::vector<int> u) const
  {
    return m_abstract->m_regions[u[0]]->m_distToGoals[m_rid];
  }

  bool MPSIPP::SIPPSearch::IsGoal(const std::vector<int>  key) const
  {
    if (key[0] != m_ridGoal)
    {
      return false;
    }
    for (int k = 0 ; k < m_safeInterval[key[0]].size(); k++)
    {
      if (key[1] < m_safeInterval[key[0]][k].first)
      {
        return false;
      }
    }
    return true;
  }

  bool MPSIPP::SIPPSearch::CheckMovement(int from, int to, int timeid) const
  {
    for (int r = 0; r < (int)m_pathsToGoal.size();r++)
    {
      if ((r == m_rid) || ((int)m_pathsToGoal[r].size() == 0))
      {
        continue;
      }
      if (timeid >= (int)m_pathsToGoal[r].size())
      {
        timeid = (int)m_pathsToGoal[r].size()-1;
      }
      int otherTo = m_pathsToGoal[r][timeid];
      int otherFrom = m_pathsToGoal[r][timeid==0?0:timeid-1];
      if(m_abstract->EdgesCheck(from,to,otherFrom,otherTo) == false)
      {
        return false;
      }
      // if(otherTo == to)
      // {
      //   return false;
      // }
      // if((otherTo == from) && (otherFrom == to))
      // {
      //   return false;
      // }
    }
    return true;
  }

  void MPSIPP::UpdateIntervals()
  {
    m_safeInterval.clear();
    m_safeInterval.resize((int)m_abstract->m_regions.size());
    int maxTime = 0;
    for (int r = 0 ; r < m_nrRobot; r++)
    {
      if (maxTime < (int) m_pathsToGoal[r].size())
      {
        maxTime = (int)m_pathsToGoal[r].size();
      }
    }
    maxTime = maxTime*m_nrRobot;
    // printf("maxTime %d\n",maxTime );
    std::vector<std::vector<int>> notsafeTime;
    notsafeTime.resize((int)m_abstract->m_regions.size());

    for (int r = 0 ; r < m_nrRobot; r++)
    {
      if ((int)m_pathsToGoal[r].size()==0)
      {
        continue;
      }
      for (int i = 0 ; i < (int)m_pathsToGoal[r].size(); i++)
      {
        notsafeTime[m_pathsToGoal[r][i]].push_back(i);
      }
    }

    // for (int i = 0; i < (int)m_abstract->m_regions.size();i++)
    // {
    //   printf("region %d: ",i );
    //   for (int k = 0;  k< notsafeTime[i].size(); k++)
    //   {
    //     printf("%d ",notsafeTime[i][k]);
    //   }
    //   printf("\n" );
    // }

    for (int i = 0; i < (int)m_abstract->m_regions.size();i++)
    {
      std::sort (notsafeTime[i].begin(), notsafeTime[i].end());
      int intervalStart = 0;
      int intervalEnd;
      for (int k = 0; k < (int)notsafeTime[i].size(); k++)
      {
        intervalEnd = notsafeTime[i][k];
        if (intervalEnd-1 >= intervalStart)
        {
          std::pair <int,int> interval (intervalStart,intervalEnd-1);
          m_safeInterval[i].push_back(interval);
        }
        intervalStart = intervalEnd+1;
      }
      std::pair <int,int> interval (intervalStart,maxTime);
      m_safeInterval[i].push_back(interval);
    }

    // for (int i = 0; i < (int)m_safeInterval.size();i++)
    // {
    //   printf("region %d: ",i );
    //   for (int k = 0;  k< m_safeInterval[i].size(); k++)
    //   {
    //     printf("[%d %d] ",m_safeInterval[i][k].first,m_safeInterval[i][k].second);
    //   }
    //   printf("\n" );
    // }

  }

  void MPSIPP::SetPathToGoal(int rid, std::vector<int> path)
  {
    m_pathsToGoal[rid].clear();
    m_pathsToGoal[rid] = path;
  }

  bool MPSIPP::isSolved()
  {
    for (int r = 0 ; r < m_nrRobot; r++)
    {
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


}
