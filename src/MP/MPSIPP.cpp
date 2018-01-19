#include "MP/MPSIPP.hpp"
#include <algorithm>

namespace MP
{
  MPSIPP::MPSIPP()
  {
    m_depth = 0;
  }

  bool MPSIPP::RunSearch(std::vector<int> inits)
  {
    m_robotOrder.clear();
    for (int r = 0; r < m_nrRobot; r++)
    {
      m_robotOrder.push_back(r);
    }
    SIPP(inits);

    // int nrtries = RandomUniformInteger(std::max(1,m_nrRobot/2), m_nrRobot);
    // for (int iter = 0 ; iter < nrtries && isSolved() == false; iter++)
    // {
    //   SIPP(inits);
    // }

    return isSolved();
  }

  bool MPSIPP::SIPP(std::vector<int> inits)
  {
    ClearPaths();
    // std::random_shuffle ( m_robotOrder.begin(), m_robotOrder.end() );
    int rid = m_robotOrder.front();
    SetPathToGoal(rid, m_abstract->m_regions[inits[rid]]->m_pathsToGoal[rid]);
    UpdateIntervals();
    ReservePath(m_pathsToGoal[rid]);
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
      m_SIPPsearch.m_reservetable = m_reservetable;
      m_SIPPsearch.m_inits = inits;

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
        printf("solved %d\n",rid );
        m_pathsToGoal[rid].clear();
        for (int k = 0; k < astarPath.size()-1; ++k)
        {
          printf("%d %d\n",astarPath[k][0],astarPath[k][1] );
          for(int o = 0; o < astarPath[k+1][1] - astarPath[k][1]; o++)
          {
            m_pathsToGoal[rid].push_back(astarPath[k][0]);
          }
        }
        m_pathsToGoal[rid].push_back(astarPath.back()[0]);
      }
      UpdateIntervals();
      ReservePath(m_pathsToGoal[rid]);
    }
  }

  void MPSIPP::ReservePath(std::vector<int> path)
  {
    for (int i = 0 ; i < m_depth; i++)
    {
      SpaceTimeNode *st;
      auto it = m_reservetable.find(i);
      if(it == m_reservetable.end())
      {
        st = new SpaceTimeNode();
        st->m_timeid = i;
        if (i < path.size())
        {
          if(i==0)
          {
            st->m_reserNodeFrom.push_back(path[i]);
            st->m_reserNodeTo.push_back(path[i]);
          }
          else
          {
            st->m_reserNodeFrom.push_back(path[i-1]);
            st->m_reserNodeTo.push_back(path[i]);
          }
        }
        else
        {
          st->m_reserNodeFrom.push_back(path.back());
          st->m_reserNodeTo.push_back(path.back());
        }

        st->m_reserveSize = st->m_reserNodeFrom.size();
        m_reservetable.insert(std::make_pair(st->m_timeid, st));
      }
      else
      {
        st = it->second;
        if (i < path.size())
        {
          if(i==0)
          {
            st->m_reserNodeFrom.push_back(path[i]);
            st->m_reserNodeTo.push_back(path[i]);
          }
          else
          {
            st->m_reserNodeFrom.push_back(path[i-1]);
            st->m_reserNodeTo.push_back(path[i]);
          }
        }
        else
        {
          st->m_reserNodeFrom.push_back(path.back());
          st->m_reserNodeTo.push_back(path.back());
        }
        st->m_reserveSize = st->m_reserNodeFrom.size();
      }
    }
  }

  void MPSIPP::SIPPSearch::GetOutEdges(const std::vector<int> u,
    std::vector<std::vector<int>>  * const edges,
    std::vector<double> * const costs) const
  {
    int s = u[0];
    printf("s %d %d [%d %d]\n\n",s,u[1],u[2],u[3] );
    int t = u[1];
    const int n = m_abstract->m_regions[s]->m_neighs.size();
    for(int i = 0; i < n; ++i)
    {
      double cost     = m_abstract->m_regions[s]->m_dists[i];
      int cfg         = m_abstract->m_regions[s]->m_neighs[i];
      int startTime   = u[1] + 1;
      int endTime     = u[3] + 1;

      for (int k = 0 ; k < m_safeInterval[cfg].size();k++)
      {
        printf("cfg %d ",cfg );
        printf("[%d %d]\n",m_safeInterval[cfg][k].first,m_safeInterval[cfg][k].second );
        if ((m_safeInterval[cfg][k].first > endTime) || (m_safeInterval[cfg][k].second < startTime))
        {
          continue;
        }
        int t = startTime > m_safeInterval[cfg][k].first ? startTime : m_safeInterval[cfg][k].first;
          std::vector<int> from;
          from.push_back(s);
          from.push_back(t-1);
          std::vector<int> to;
          to.push_back(cfg);
          to.push_back(t);
          if (CheckWithReserTable(from,to) == true)
          {
            printf("add at time %d %d\n",cfg,t);
            std::vector<int> neigh;
            neigh.push_back(cfg);
            neigh.push_back(t);
            neigh.push_back(m_safeInterval[cfg][k].first);
            neigh.push_back(m_safeInterval[cfg][k].second);
            edges->push_back(neigh);
            costs->push_back(cost);
            break;
          }
          printf("not at time %d %d\n",cfg,t);

      }
    }

    double cost     = m_abstract->MaxCostToNeighbors(s);
    int cfg         = s;
    int startTime   = u[1] + 1;
    int endTime     = u[2] + 1;
    for (int k = 0 ; k < m_safeInterval[cfg].size();k++)
    {
      printf("cfg %d ",cfg );
      printf("[%d %d]\n",m_safeInterval[cfg][k].first,m_safeInterval[cfg][k].second );

      if ((m_safeInterval[cfg][k].first > endTime) || (m_safeInterval[cfg][k].second < startTime))
      {
        continue;
      }
      int t = startTime > m_safeInterval[cfg][k].first ? startTime : m_safeInterval[cfg][k].first;
        std::vector<int> from;
        from.push_back(s);
        from.push_back(t-1);
        std::vector<int> to;
        to.push_back(cfg);
        to.push_back(t);
        if (CheckWithReserTable(from,to) == true)
        {
          printf("add at time %d %d\n",cfg,t);
          std::vector<int> neigh;
          neigh.push_back(cfg);
          neigh.push_back(t);
          neigh.push_back(m_safeInterval[cfg][k].first);
          neigh.push_back(m_safeInterval[cfg][k].second);
          edges->push_back(neigh);
          costs->push_back(cost);
          break;
        }
        printf("not at time %d %d\n",cfg,t);
      }

    // getchar();
  }

  double MPSIPP::SIPPSearch::HeuristicCostToGoal(const std::vector<int> u) const
  {
    return m_abstract->m_regions[u[0]]->m_distToGoals[m_rid];;
  }

  bool MPSIPP::SIPPSearch::IsGoal(const std::vector<int>  key) const
  {
    return key[0] == m_ridGoal;
  }

  bool MPSIPP::SIPPSearch::CheckWithReserTable(std::vector<int> from, std::vector<int> to) const
  {
    // if (to[1] == 1)
    // {
    //   for (int r = 0 ; r < m_nrRobot; r++)
    //   {
    //     if(r == m_rid)
    //     {
    //       continue;
    //     }
    //
    //     if (m_abstract->EdgesCheck(m_inits[r],m_inits[r],from[0],to[0]) == false)
    //     {
    //       return false;
    //     }
    //   }
    // }

    SpaceTimeNode *st;
    auto it = m_reservetable.find(to[1]);
    if(it != m_reservetable.end())
    {
      st = it->second;
      for (int i = 0 ; i < st->m_reserveSize; i++)
      {
        printf("check %d %d %d\n",from[0], to[0], to[1] );
        printf("with %d %d\n",st->m_reserNodeFrom[i],st->m_reserNodeTo[i] );
        if (m_abstract->EdgesCheck(from[0],to[0],st->m_reserNodeFrom[i],st->m_reserNodeTo[i],to[1]) == false)
        {
          return false;
        }
      }
      return true;
    }
    else
    {
      return true;
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
    maxTime = 9999;

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
    if (m_depth < m_pathsToGoal[rid].size())
    {
      m_depth = m_pathsToGoal[rid].size();
    }
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
