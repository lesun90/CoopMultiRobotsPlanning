#ifndef MP_PAS_HPP_
#define MP_PAS_HPP_

#include "MPMultiSearch.hpp"
#include "External/PAS/PushAndSwap.hpp"

namespace MP
{
  class MPPushAndSwap : public MPMultiSearch
  {
  public:
    MPPushAndSwap(void);

    virtual ~MPPushAndSwap(void)
    {
    }

    virtual bool RunSearch(std::vector<int> inits);
    virtual void writePlan(const PaS::PushAndSwap::Plan &simulation);
    virtual void GetPaths(const PaS::PushAndSwap::Plan &simulation,std::vector<int> inits);

    virtual void WriteAbstraction()
    {

      UseMap(std::vector<int>, bool)  m_edges;

      FILE     *myfile = NULL;
      char      cmd[300];
      sprintf(cmd, "connector.gml", "txt");
      myfile = fopen(cmd, "w");

      fprintf(myfile, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
      fprintf(myfile, "<graphml>\n");
      fprintf(myfile, "<key id=\"key0\" for=\"node\" attr.name=\"Coordinate\" attr.type=\"string\" />\n");
      fprintf(myfile, "<key id=\"key1\" for=\"edge\" attr.name=\"Weight\" attr.type=\"double\"> <!-- Unit edge weights by default -->\n");
      fprintf(myfile, "  <default>1.0</default>\n");
      fprintf(myfile, "</key>\n");
      fprintf(myfile, "<graph id=\"G\" edgedefault=\"undirected\" parse.nodeids=\"free\" parse.edgeids=\"canonical\" parse.order=\"nodesfirst\">\n");


      for (int i = 0 ; i < m_abstract->m_regions.size(); i++)
      {
        fprintf(myfile, "    <node id=\"n%d\">\n",i);
        fprintf(myfile, "      <data key=\"key0\">%f %f 0.0</data>\n",m_abstract->m_regions[i]->m_cfg[0],m_abstract->m_regions[i]->m_cfg[1]);
        fprintf(myfile, "    </node>\n");

      }

      int count = 0;
      for (int i = 0; i < m_abstract->m_regions.size();i ++)
      {
        for (int n = 0 ; n < m_abstract->m_regions[i]->m_neighs.size(); n++)
        {
          int from = i;
          int to = m_abstract->m_regions[i]->m_neighs[n];
          double cost = m_abstract->m_regions[i]->m_weights[n];

          std::vector<int> c1;
          std::vector<int> c2;
          c1.push_back(from);
          c1.push_back(to);
          c2.push_back(to);
          c2.push_back(from);

          auto i1 = m_edges.find(c1);
          auto i2 = m_edges.find(c2);

          if((i1 == m_edges.end()) && (i2 == m_edges.end()))
          {
            fprintf(myfile, "    <edge id=\"e%d\" source=\"n%d\" target=\"n%d\">\n",count++,from,to);
            fprintf(myfile, "      <data key=\"key1\">%f</data>\n",cost);
            fprintf(myfile, "    </edge>\n");
            std::vector<int> a;
            a.push_back(from);
            a.push_back(to);
            m_edges.insert(std::make_pair(a, true));
          }
          else
          {
            continue;
          }
        }
      }

      fprintf(myfile, "</graph>\n");
      fprintf(myfile, "</graphml>\n");

      fclose(myfile);

      sprintf(cmd, "connector.instance", "txt");
      myfile = fopen(cmd, "w");
      fprintf(myfile, "graph = connector.gml\n");
      for (int i = 0 ; i < m_nrRobot; i ++)
      {
        fprintf(myfile, "agent: start = %d, goal = %d\n",m_abstract->m_ridInits[i],m_abstract->m_ridGoals[i]);
      }
      // fprintf(myfile, "</graph>\n");

      fclose(myfile);

    }

    PaS::Graph graph;
  };
}
#endif
