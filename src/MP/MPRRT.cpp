#include "MP/MPRRT.hpp"

namespace MP
{
  void MPRRT::Run(const int nrIters)
  {
    if(m_cfgTarget == NULL)
    m_cfgTarget = m_sim->NewCfg();

    if(m_vertices.size() == 0)
    Initialize();

    if(!m_proximity.IsDataStructureConstructed())
    m_proximity.ConstructDataStructure();

    int                 prop;
    ProximityQuery<int> q;

    q.SetKey(Constants::ID_UNDEFINED);

    for(int i = 0; i < nrIters && GetSolved() < 0; ++i)
    {
      m_sim->SampleCfg(m_cfgTarget);
      if(RandomUniformReal() < m_goalBias)
      {
        m_cfgTarget[0] = m_sim->m_goal[0];
        m_cfgTarget[1] = m_sim->m_goal[1];
      }

      ExtendFrom(m_proximity.Neighbor(&q), m_cfgTarget);
    }
  }
}
