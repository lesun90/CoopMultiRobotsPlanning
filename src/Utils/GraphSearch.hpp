#ifndef MP_GRAPH_SEARCH_HPP_
#define MP_GRAPH_SEARCH_HPP_

#include "Utils/Definitions.hpp"
#include "Utils/Map.hpp"
#include "Utils/Set.hpp"
#include "Utils/Heap.hpp"
#include "Utils/Misc.hpp"
#include <stack>
#include <queue>
#include <cstdio>

namespace MP
{
  template <typename Key>
  class GraphSearchInfo
  {
  public:
    GraphSearchInfo(void)
    {
    }

    virtual ~GraphSearchInfo(void)
    {
    }

    virtual void GetOutEdges(const Key u,
      std::vector<Key> * const edges,
      std::vector<double> * const costs = NULL) const = 0;

      virtual bool IsGoal(const Key key) const = 0;

      virtual double HeuristicCostToGoal(const Key u) const
      {
        return 0;
      }

      virtual void PrintKey(const Key u) const
      {
      }

      virtual bool IsConnected(Key key1, Key key2)
      {
        return true;
      }

      virtual double Cost2Key(Key key1, Key key2)
      {
        return 0;
      }


    };


    template <typename Key>
    class GraphSearch
    {
    public:
      GraphSearch(void)
      {
        m_heap.m_lessFn     = LessFn;
        m_heap.m_lessFnData = this;
        m_info              = NULL;
        m_gCostMax          = HUGE_VAL;
        m_useBestFirst      = false;
      }

      virtual ~GraphSearch(void)
      {
      }

      GraphSearchInfo<Key> *m_info;

      bool DFS(const Key start, const bool randomize, Key * const goal);

      bool BFS(const Key start, const bool randomize, Key * const goal);

      bool AStar(const Key start, const bool breakEarly, Key * const goal);

      bool STAStar(const Key start, const bool breakEarly, Key * const goal);

      bool ThetaStar(const Key start, const bool breakEarly, Key * const goal);

      void GetReversePathFromStart(const Key u, std::vector<Key> * const rpath) const
      {
        rpath->clear();

        if(m_map.find(u) == m_map.end())
        return;

        Key p = u, v;

        do
        {
          v = p;
          rpath->push_back(v);
          p = m_map.find(v)->second.m_parent;
        }
        while(!(v == p));
      }

      void GetPathFromStart(const Key u, std::vector<Key> * const path) const
      {
        GetReversePathFromStart(u, path);
        ReverseItems<Key>(path);
      }

      int GetPathLengthFromStart(const Key u) const
      {
        if(m_map.find(u) == m_map.end())
        return -1;

        Key p = u, v;
        int count = 0;
        do
        {
          v = p;
          ++count;
          p = m_map.find(v)->second.m_parent;
        }
        while(!(v == p));

        return count;
      }

      double GetPathCostFromStart(const Key u) const
      {
        auto cur = m_map.find(u);

        if(cur == m_map.end())
        return HUGE_VAL;
        return cur->second.m_gCost;
      }

      double m_gCostMax;
      bool   m_useBestFirst;

    protected:
      struct Data
      {
        Key    m_parent;
        double m_gCost;
        double m_level;
        double m_hCost;
      };

      static bool LessFn(const Key u, const Key v, GraphSearch<Key> * gs);

      UseMap(Key, Data)              m_map;
      std::vector<Key>               m_stack;
      Heap<Key, GraphSearch<Key>* >  m_heap;
      std::vector<std::vector<Key>>  m_dynamicObs;

    };

    template <typename Key>
    bool GraphSearch<Key>::DFS(const Key start, const bool randomize, Key * const goal)
    {
      Key  u, v;
      Data data;

      std::vector<Key> edges;

      m_stack.clear();
      m_map.clear();

      data.m_parent = start;
      m_map.insert(std::make_pair(start, data));

      if(m_info->IsGoal(start))
      {
        *goal = start;
        return true;
      }


      m_stack.push_back(start);
      while(!m_stack.empty())
      {
        u = m_stack.back();
        m_stack.pop_back();

        edges.clear();
        m_info->GetOutEdges(u, &edges);
        const int n = edges.size();

        if(randomize)
        PermuteItems<Key>(&edges, n);

        for(int i = 0; i < n; ++i)
        {
          v = edges[i];
          if(m_map.find(v) == m_map.end())
          {
            data.m_parent = u;
            m_map.insert(std::make_pair(v, data));
            if(m_info->IsGoal(v))
            {
              *goal = v;
              return true;
            }
            m_stack.push_back(v);
          }
        }
      }

      return false;
    }


    template <typename Key>
    bool GraphSearch<Key>::BFS(const Key start, const bool randomize, Key * const goal)
    {
      Key u, v;
      Data data;
      std::vector<Key> edges;
      std::queue<Key>  q;

      data.m_parent = start;
      m_map.clear();
      m_map.insert(std::make_pair(start, data));
      if(m_info->IsGoal(start))
      {
        *goal = start;
        return true;
      }


      q.push(start);
      while(!q.empty())
      {
        u = q.front();
        q.pop();

        edges.clear();
        m_info->GetOutEdges(u, &edges);
        const int n = edges.size();

        if(randomize)
        PermuteItems<Key>(&edges, n);

        for(int i = 0; i < n; ++i)
        {
          v = edges[i];
          if(m_map.find(v) == m_map.end())
          {
            data.m_parent = u;
            m_map.insert(std::make_pair(v, data));
            if(m_info->IsGoal(v))
            {
              *goal = v;
              return true;
            }
            q.push(v);
          }
        }
      }

      return false;
    }

    template <typename Key>
    bool GraphSearch<Key>::LessFn(const Key u, const Key v, GraphSearch<Key> * gs)
    {
      Data datau = gs->m_map.find(u)->second;
      Data datav = gs->m_map.find(v)->second;

      return
      gs->m_useBestFirst ?
      datau.m_hCost < datav.m_hCost :
      ((datau.m_gCost + datau.m_hCost) < (datav.m_gCost + datav.m_hCost));
    }

    template <typename Key>
    bool GraphSearch<Key>::AStar(const Key start, const bool breakEarly, Key * const goal)
    {
      Key                 u;
      Key                 v;
      Data                datau;
      Data                datav;
      std::vector<Key>    edges;
      std::vector<double> costs;
      UseSet(Key)         closed;

      m_gCostMax = -HUGE_VAL;
      m_map.clear();
      m_heap.Clear();

      datau.m_parent= start;
      datau.m_gCost = 0;
      datau.m_hCost = m_info->HeuristicCostToGoal(start);
      m_map.insert(std::make_pair(start, datau));
      m_heap.Insert(start);

      while(!m_heap.IsEmpty())
      {
        //remove top
        u     = m_heap.RemoveTop();
        datau = m_map.find(u)->second;
        if(datau.m_gCost > m_gCostMax)
        m_gCostMax = datau.m_gCost;
        closed.insert(u);
        if(m_info->IsGoal(u))
        {
          *goal = u;
          return true;
        }
        //get edges and costs
        edges.clear();
        costs.clear();
        m_info->GetOutEdges(u, &edges, &costs);

        //process edges
        for(int i = edges.size() - 1; i >= 0; --i)
        if(closed.find(edges[i]) == closed.end())
        {
          v        = edges[i];
          auto cur = m_map.find(v);
          if(cur == m_map.end())
          {
            datav.m_parent = u;
            datav.m_gCost  = datau.m_gCost + costs[i];
            datav.m_hCost  = m_info->HeuristicCostToGoal(v);
            if(datav.m_hCost != HUGE_VAL)
            {
              m_map.insert(std::make_pair(v, datav));
              m_heap.Insert(v);
              if(breakEarly && m_info->IsGoal(v))
              {
                *goal = v;
                if(datav.m_gCost > m_gCostMax)
                m_gCostMax = datav.m_gCost;
                return true;
              }
            }
          }
          else if((datau.m_gCost + costs[i]) < cur->second.m_gCost)
          {
            cur->second.m_parent = u;
            cur->second.m_gCost = datau.m_gCost + costs[i];
            m_heap.Update(v);
          }
        }
      }
      return false;
    }

    template <typename Key>
    bool GraphSearch<Key>::STAStar(const Key start, const bool breakEarly, Key * const goal)
    {
      Key                 u;
      Key                 v;
      Data                datau;
      Data                datav;
      std::vector<Key>    edges;
      std::vector<double> costs;
      UseSet(Key)         closed;

      m_gCostMax = -HUGE_VAL;
      m_map.clear();
      m_heap.Clear();

      datau.m_parent= start;
      datau.m_gCost = 0;
      datau.m_hCost = m_info->HeuristicCostToGoal(start);
      m_map.insert(std::make_pair(start, datau));
      m_heap.Insert(start);

      while(!m_heap.IsEmpty())
      {
        //remove top
        u     = m_heap.RemoveTop();
        datau = m_map.find(u)->second;
        if(datau.m_gCost > m_gCostMax)
        m_gCostMax = datau.m_gCost;
        closed.insert(u);
        if(m_info->IsGoal(u))
        {
          *goal = u;
          return true;
        }
        //get edges and costs
        edges.clear();
        costs.clear();
        m_info->GetOutEdges(u, &edges, &costs);

        //process edges
        for(int i = edges.size() - 1; i >= 0; --i)
        if(closed.find(edges[i]) == closed.end())
        {
          v        = edges[i];
          auto cur = m_map.find(v);
          if(cur == m_map.end())
          {
            datav.m_parent = u;
            datav.m_gCost  = datau.m_gCost + costs[i];
            datav.m_hCost  = m_info->HeuristicCostToGoal(v);
            if(datav.m_hCost != HUGE_VAL)
            {
              m_map.insert(std::make_pair(v, datav));
              m_heap.Insert(v);
              if(m_info->IsGoal(v))
              {
                *goal = v;
                if(datav.m_gCost > m_gCostMax)
                m_gCostMax = datav.m_gCost;
                return true;
              }
            }
          }
          else if((datau.m_gCost + costs[i]) < cur->second.m_gCost)
          {
            cur->second.m_parent = u;
            cur->second.m_gCost = datau.m_gCost + costs[i];
            m_heap.Update(v);
          }
        }
      }
      return false;
    }

    template <typename Key>
    bool GraphSearch<Key>::ThetaStar(const Key start, const bool breakEarly, Key * const goal)
    {
      Key                 u;
      Key                 v;
      Data                datau;
      Data                datav;
      std::vector<Key>    edges;
      std::vector<double> costs;
      UseSet(Key)         closed;

      m_gCostMax = -HUGE_VAL;
      m_map.clear();
      m_heap.Clear();

      datau.m_parent= start;
      datau.m_gCost = 0;
      datau.m_hCost = m_info->HeuristicCostToGoal(start);
      m_map.insert(std::make_pair(start, datau));
      m_heap.Insert(start);

      while(!m_heap.IsEmpty())
      {
        //remove top
        u     = m_heap.RemoveTop();
        datau = m_map.find(u)->second;
        if(datau.m_gCost > m_gCostMax)
        m_gCostMax = datau.m_gCost;
        closed.insert(u);

        if(m_info->IsGoal(u))
        {
          *goal = u;
          return true;
        }
        //get edges and costs
        edges.clear();
        costs.clear();
        m_info->GetOutEdges(u, &edges, &costs);

        //process edges
        for(int i = edges.size() - 1; i >= 0; --i)
        {
          if(closed.find(edges[i]) == closed.end())
          {
            v        = edges[i];
            auto cur = m_map.find(v);
            if(cur == m_map.end())
            {
              datav.m_hCost  = m_info->HeuristicCostToGoal(v);
              datav.m_gCost  = HUGE_VAL;
              if(m_info->IsConnected(datau.m_parent,v))
              {
                auto uparent = m_map.find(datau.m_parent);
                if ((uparent->second.m_gCost + m_info->Cost2Key(datau.m_parent,v)) < datav.m_gCost)
                {
                  datav.m_parent = datau.m_parent;
                  datav.m_gCost = uparent->second.m_gCost + m_info->Cost2Key(datau.m_parent,v) ;
                }
              }
              else
              {
                datav.m_parent = u;
                datav.m_gCost  = datau.m_gCost + costs[i];
              }
              if(datav.m_hCost != HUGE_VAL)
              {
                m_map.insert(std::make_pair(v, datav));
                m_heap.Insert(v);
                if(breakEarly && m_info->IsGoal(v))
                {
                  *goal = v;
                  if(datav.m_gCost > m_gCostMax)
                  m_gCostMax = datav.m_gCost;
                  return true;
                }
              }
            }
            else
            {
              if(m_info->IsConnected(datau.m_parent,v))
              {
                auto uparent = m_map.find(datau.m_parent);
                if ((uparent->second.m_gCost + m_info->Cost2Key(datau.m_parent,v)) < datav.m_gCost)
                {
                  cur->second.m_parent = datau.m_parent;
                  cur->second.m_gCost  = uparent->second.m_gCost + m_info->Cost2Key(datau.m_parent,v) ;
                  m_heap.Update(v);
                }
              }
              else if((datau.m_gCost + costs[i]) < cur->second.m_gCost)
              {
                cur->second.m_parent = u;
                cur->second.m_gCost = datau.m_gCost + costs[i];
                m_heap.Update(v);
              }
            }
          }
        }
      }
      return false;
    }



  }

  #endif
