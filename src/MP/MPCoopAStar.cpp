#include "MP/MPCoopAStar.hpp"
#include <algorithm>

namespace MP
{
  MPCoopAStar::MPCoopAStar()
  {
    m_depth = 20;
  }

  bool MPCoopAStar::RunSearch(std::vector<int> inits)
  {
    m_robotOrder.clear();
    for (int r = 0; r < m_nrRobot; r++)
    {
      m_robotOrder.push_back(r);
    }
    int nrtries = RandomUniformInteger(std::max(1,m_nrRobot/2), m_nrRobot);
    Timer::Clock clk;
    Timer::Start(&clk);
    for (int iter = 0 ; iter < nrtries && isSolved() == false; iter++)
    {
      CoopAStar(inits);
    }
    Stats::GetSingleton()->AddValue("FindPathTime", Timer::Elapsed(&clk));
    Stats::GetSingleton()->AddValue("NrFindPath", 1);
    return isSolved();
  }

  bool MPCoopAStar::CoopAStar(std::vector<int> inits)
  {
    ClearPaths();
    ClearReservationTable();
    PermuteRobotOrder();
    int rid = m_robotOrder.front();
    SetPathToGoal(rid, m_abstract->m_regions[inits[rid]]->m_pathsToGoal[rid]);
    ReservePath(m_pathsToGoal[rid]);

    for (int i = 1; i < m_nrRobot; i++)
    {
      rid = m_robotOrder[i];
      m_pathsToGoal[rid].clear();
      CoopSearch                    m_coopAStar;
      GraphSearch<std::vector<int>> m_gs;
      const bool                    breakEarly = false;

      m_gs.m_info                   = &m_coopAStar;
      std::vector<int>              goalKey;
      std::vector<int>              initKey;
      std::vector<std::vector<int>> astarPath;
      m_coopAStar.SetAbstraction(m_abstract);
      m_coopAStar.m_depth = m_depth;
      m_coopAStar.m_ridGoal = m_goals[rid];
      m_coopAStar.m_reservetable = m_reservetable;
      m_coopAStar.m_inits = inits;
      m_coopAStar.m_rid = rid;

      initKey.push_back(inits[rid]);
      initKey.push_back(0);

      if(m_gs.AStar(initKey, breakEarly, &goalKey))
      {
        m_gs.GetPathFromStart(goalKey, &astarPath);
      }
      if (astarPath.size()>0)
      {
        m_pathsToGoal[rid].clear();
        for (int k = 0; k < astarPath.size(); ++k)
        {
          m_pathsToGoal[rid].push_back(astarPath[k][0]);
        }
      }
      else
      {
        return false;
      }
      ReservePath(m_pathsToGoal[rid]);
    }
    CompletePaths();
    return true;
  }

  void MPCoopAStar::CompletePaths()
  {
    for (int r = 0 ; r < m_nrRobot; r++)
    {
      std::vector<int> pathToEnd = m_abstract->m_regions[m_pathsToGoal[r].back()]->m_pathsToGoal[r];
      if (pathToEnd.size() > 0)
      {
        m_pathsToGoal[r].insert(m_pathsToGoal[r].end(),pathToEnd.begin()+1,pathToEnd.end() );
      }
      else
      {
        m_pathsToGoal[r].push_back(m_goals[r]);
      }
    }
    for (int r = 0 ; r < m_nrRobot; r++)
    {
      if (m_pathsToGoal[r].size() < 2)
      {
        continue;
      }
      int curr = m_pathsToGoal[r].back();
      for (int i = m_pathsToGoal[r].size()-2; i>=0; i--)
      {
        if (curr == m_pathsToGoal[r][i])
        {
          m_pathsToGoal[r].pop_back();
        }
        else
        {
          break;
        }
      }
    }
  }

  bool MPCoopAStar::isSolved()
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

  // for (int i = 0 ; i < m_depth+1; i++)
  // {
  //   for (int a = 0 ; a < m_nrRobot-1; a++)
  //   {
  //     for (int b = a+1; b < m_nrRobot; b++)
  //     {
  //       int ia = std::min<int>(i, m_pathsToGoal[a].size()-1);
  //       int ib = std::min<int>(i, m_pathsToGoal[b].size()-1);
  //       int ra = m_pathsToGoal[a][ia];
  //       int rb = m_pathsToGoal[b][ib];
  //       if (Algebra2D::PointDist(m_abstract->m_regions[ra]->m_cfg,m_abstract->m_regions[rb]->m_cfg) < 0.5)
  //       {
  //         return false;
  //       }
  //     }
  //   }
  // }

  return true;
}

  void MPCoopAStar::ReservePath(std::vector<int> path)
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

  bool MPCoopAStar::CoopSearch::CheckWithReserTable(std::vector<int> from, std::vector<int> to) const
  {
    if (to[1] == 1)
    {
      for (int r = 0 ; r < m_nrRobot; r++)
      {
        if(r == m_rid)
        {
          continue;
        }

        if (m_abstract->EdgesCheck(m_inits[r],m_inits[r],from[0],to[0]) == false)
        {
          return false;
        }
      }
    }

    SpaceTimeNode *st;
    auto it = m_reservetable.find(to[1]);

    if(it != m_reservetable.end())
    {

      st = it->second;

      for (int i = 0 ; i < st->m_reserveSize; i++)
      {
        // printf("from %d to %d ||r from %d to %d \n",from[0],to[0],st->m_reserNodeFrom[i],st->m_reserNodeTo[i] );

        if (m_abstract->EdgesCheck(from[0],to[0],st->m_reserNodeFrom[i],st->m_reserNodeTo[i]) == false)
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


  void MPCoopAStar::CoopSearch::GetOutEdges(const std::vector<int> u,
    std::vector<std::vector<int>>  * const edges,
    std::vector<double> * const costs) const
  {
    double cost;
    double maxCost = HUGE_VAL;
    const int n = m_abstract->m_regions[u[0]]->m_neighs.size();
    const int nexttimeid = u[1]+1;
    for(int i = 0; i < n; ++i)
  	{
      int m_neigh = m_abstract->m_regions[u[0]]->m_neighs[i];
      if (m_neigh == u[0])
      {
        continue;
      }

      std::vector<int> neigh;
      neigh.push_back(m_neigh);
      neigh.push_back(nexttimeid);
      cost = m_abstract->m_regions[u[0]]->m_weights[i];
      if (cost < maxCost)
      {
        maxCost = cost;
      }
      // printf("CheckWithReserTable: %d %d %f\n",u[0],neigh[0],cost );

      if (CheckWithReserTable(u,neigh) == true)
      {
        // printf("CheckWithReserTable true: %d %d %f\n",u[0],neigh[0],cost );

        edges->push_back(neigh);
        costs->push_back(cost);
      }
    }
    std::vector<int> neigh;
    neigh.push_back(u[0]);
    neigh.push_back(nexttimeid);
    if (CheckWithReserTable(u,neigh) == true)
    {
      if(neigh[0]==m_ridGoal)
      {
        edges->push_back(neigh);
        costs->push_back(0);
      }
      else
      {
        edges->push_back(neigh);
        costs->push_back(0.99*maxCost);
      }

    }
  }

  double MPCoopAStar::CoopSearch::HeuristicCostToGoal(const std::vector<int> u) const
  {
    return m_abstract->m_regions[u[0]]->m_distToGoals[m_rid];;
  }

  bool MPCoopAStar::CoopSearch::IsGoal(const std::vector<int>  key) const
  {
    return (key[1] > m_depth);
  }

}
