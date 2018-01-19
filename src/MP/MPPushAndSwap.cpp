#include "MP/MPPushAndSwap.hpp"
#include <iostream>
#include "External/PAS/Instance.hpp"
#include <algorithm>
namespace MP
{
  MPPushAndSwap::MPPushAndSwap()
  {
    printf("Use MPPushAndSwap search\n");
  }

  // Write each step of the plan, annotated with the agent id.
  void MPPushAndSwap::writePlan(const PaS::PushAndSwap::Plan &simulation)
  {
    for (size_t i = 0; i < simulation.size (); i++)
    {
      std::cout << "[Agent " << simulation[i].first << "] ";
      for (size_t j = 0; j < simulation[i].second.size(); ++j)
      {
        std::cout << simulation[i].second[j] << " ";
      }
      std::cout << std::endl;
    }
  }

  void MPPushAndSwap::GetPaths(const PaS::PushAndSwap::Plan &simulation,std::vector<int> inits)
  {
    for (int k = 0 ; k < m_nrRobot; k++)
    {
      m_pathsToGoal[k].push_back(inits[k]);
    }

    int maxsize = -1;

    for (size_t i = 0; i < simulation.size (); i++)
    {
      for (size_t j = 1; j < simulation[i].second.size(); ++j)
      {
        m_pathsToGoal[simulation[i].first].push_back(simulation[i].second[j]);
      }
      for (int k = 0 ; k < m_nrRobot; k++)
      {
        maxsize = std::max(maxsize,(int)m_pathsToGoal[k].size());
      }
      m_pathsToGoal[simulation[i].first].resize(maxsize,m_pathsToGoal[simulation[i].first].back());
    }
    //
    // for (int k = 0 ; k < m_nrRobot; k++)
    // {
    //   printf("rid: \n",k );
    //   for (int i = 0 ; i < m_pathsToGoal[k].size(); i++)
    //   {
    //     printf("%d ,",m_pathsToGoal[k][i]);
    //   }
    //   printf("\n" );
    // }
  }

  bool MPPushAndSwap::RunSearch(std::vector<int> inits)
  {
    PaS::Graph graph;
    // Construct the graph for the instance
    // WriteAbstraction();
    // Instance config("connector.instance");
    // graph.readGraphML(std::string(config.getGraphFilename()).c_str());
    // std::vector<PaS::Agent> agents = config.getAgents();

    graph.SetupFromAbstraction(m_abstract);
    // // Create PushAndSwap planner
    std::vector<PaS::Agent> agents;
    for (int i = 0 ; i < m_nrRobot; i++)
    {
      agents.push_back(PaS::Agent(i, inits[i], m_abstract->m_ridGoals[i]));
      // printf("%d %d\n",m_abstract->m_ridInits[i], m_abstract->m_ridGoals[i] );
    }

    PaS::PushAndSwap pas(agents, &graph);
    Timer::Clock clk;
    Timer::Start(&clk);

    // Solve the problem
    if (pas.solve())
    {
        // std::cout << "SOLUTION FOUND" << std::endl;
        // Retrieving the solution
        PaS::PushAndSwap::Plan plan = pas.getPlan();
        // std::cout << "======== Unsmoothed plan ========" << std::endl;
        // writePlan(plan);

        // Heuristically smooth out some of the redundancy in the plan
        PaS::PushAndSwap::Plan smoothed = PaS::PushAndSwap::smooth(plan);
        // std::cout << "======== Smoothed plan ========" << std::endl;
        // writePlan(smoothed);
        GetPaths(smoothed,inits);
    }
    else
    {
        std::cout << "NO SOLUTION FOUND" << std::endl;
    }
    // PaS::Graph graph;
    // graph.SetupFromAbstraction(m_abstract);
    // std::vector<PaS::Agent> agents;
    // for (int r = 0; r < m_nrRobot; r++)
    // {
    //   int start = inits[r];
    //   int goal = m_abstract->m_ridGoals[r];
    //   agents.push_back(PaS::Agent(r,start,goal));
    //   printf("%d %d\n",start,goal );
    // }
    // PaS::PushAndSwap pas(agents, &graph);
    //
    // if (pas.solve())
    // {
    //     std::cout << "SOLUTION FOUND" << std::endl;
    //     // Retrieving the solution
    //     PaS::PushAndSwap::Plan plan = pas.getPlan();
    //     // std::cout << "======== Unsmoothed plan ========" << std::endl;
    //     writePlan(plan);
    //
    //     // Heuristically smooth out some of the redundancy in the plan
    //     PaS::PushAndSwap::Plan smoothed = PaS::PushAndSwap::smooth(plan);
    //     std::cout << "======== Smoothed plan ========" << std::endl;
    //             writePlan(smoothed);
    // }
    // else
    // {
    //     std::cout << "NO SOLUTION FOUND" << std::endl;
    // }
    Stats::GetSingleton()->AddValue("FindPathTime", Timer::Elapsed(&clk));
    Stats::GetSingleton()->AddValue("NrFindPath", 1);
    return isSolved();
  }
}
