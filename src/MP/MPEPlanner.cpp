#include "MP/MPEPlanner.hpp"
#include "Utils/Colormap.hpp"

namespace MP
{
  MPEPlanner::MPEPlanner(void) : MPTreePlanner()
  {
    m_wmin = Constants::EPSILON;
    m_iter = 0;
    m_discountSelect = 0.85;
    m_hexp           = 8;//50.00;
    m_wexp           = 15.00;
    m_cutoff         = 0.7;
  }

  MPEPlanner::~MPEPlanner(void)
  {
    for(auto it = m_map.begin(); it != m_map.end(); ++it)
    if(it->second)
    delete it->second;
  }

  void MPEPlanner::Run(const int nrIters)
  {
    if(m_vertices.size() == 0)
    {
      Initialize();
    }
    ARData *data;
    int     rid;

    for(int i = 0; i < nrIters && GetSolved() < 0; ++i)
    {
      ++m_iter;

      ARData   *data = SelectARData(&rid);
      SelectTarget(rid);
      const int vid  = SelectVertexFromARData(data);
      ExtendFrom(vid, m_cfgTarget);
      data->m_weight *= m_discountSelect;
      if(data->m_weight <= m_wmin)
      {
        for(auto it = m_map.begin(); it != m_map.end(); ++it)
        it->second->m_weight /= m_discountSelect;
      }
    }
    m_vertices[0]->m_cfg[3] =0;

  }

void MPEPlanner::CompleteARData(const int key, ARData * const data)
{
  //printf("inserting key %d\n", key);
  m_map.insert(std::make_pair(key, data));

  const double dmax = m_abstract->m_maxDistToGoal[m_sim->m_id];
  double       d    = m_abstract->m_regions[key]->m_distToGoals[m_sim->m_id];
  double       w    = d > dmax ? 0 : (dmax - d) / (dmax - m_abstract->m_minDistToGoal[m_sim->m_id]);
  const double alpha = 100000;
  const double eps  = 0.00000000001;

  w = eps + (1 - eps) * w;
  w = alpha * (pow(w, m_hexp));// * pow(m_abstract->m_regions[key]->m_vol, 3);
  if(w < m_wmin)
  w = m_wmin;
  data->m_weight = w;
}

void MPEPlanner::AddVertexToARData(const int vid)
{
  Vertex   *v  = m_vertices[vid];

  v->m_rid = m_abstract->LocateRegion(v->m_cfg);

  if(v->m_rid < 0)
  return;

  const int key   = v->m_rid;
  ARData   *data  = NULL;
  auto      cur   = m_split.find(key);

  //printf("add vid %d to rid %d\n", vid, key);

  if(cur != m_split.end())
  {
    data = cur->second;
    m_split.erase(key);
    m_map.insert(std::make_pair(key, data));
    // printf(" found and removed from split\n");

  }
  else
  {
    cur = m_map.find(key);
    if(cur == m_map.end())
    {
      //	printf("not found in map\n");
      data = new ARData();
      CompleteARData(key, data);
    }
    else
    {
      //printf("found in map\n");
      data = cur->second;
    }

  }

  data->m_vids.push_back(vid);

  // data->m_gCostSum += v->m_gCost;
  // if(v->m_gCost >= Constants::EPSILON)
  // data->m_gCostTotalWeight += 1.0 / v->m_gCost;

}

void MPEPlanner::Initialize(void)
{
  if(m_cfgTarget == NULL)
  m_cfgTarget = m_sim->NewCfg();

  if(!(m_sim->IsStateValid()))
  {
    printf("initial state is in collision\n");
    exit(0);
  }

  Vertex *v   = NewVertex();
  if(AddVertex(v) < 0)
  {
    printf("initial state invalid\n");
    delete v;
    exit(0);
  }
}

int MPEPlanner::AddVertex(Vertex * const v)
{
  if(v->m_cfg == NULL)
  {
    v->m_cfg = m_sim->NewCfg();
  }
  m_sim->GetCfg(v->m_cfg);
  v->m_s = m_sim->NewState();
  m_sim->GetState(v->m_s);
  int rid = m_abstract->LocateRegion(v->m_cfg);

  if((rid < 0) || (m_abstract->m_regions[rid]->m_valid == false))
  {
    return Constants::ID_UNDEFINED;;
  }
  v->m_rid = rid;

  if(v->m_parent >= 0)
  {
    v->m_gCost = m_vertices[v->m_parent]->m_gCost +
    m_sim->DistanceCfg(v->m_cfg, m_vertices[v->m_parent]->m_cfg);
  }

  m_vertices.push_back(v);
  const int vid = m_vertices.size() - 1;

  if(m_sim->IsGoalReached(v->m_cfg))
  {
    m_vidSolved = vid;
  }

  AddVertexToARData(vid);

  return vid;
}

MPEPlanner::ARData* MPEPlanner::SelectARData(int * const key)
{
  ARData *data, *sel = NULL;
  double  wmax = -HUGE_VAL;

  // printf("selecting from %d regions\n", m_map.size());

  for(auto it = m_map.begin(); it != m_map.end(); ++it)
  {
    data = it->second;
    if(data->m_vids.size() > 0 && data->m_weight >= wmax)
    {
      wmax = data->m_weight;
      sel  = data;
      *key  = it->first;
    }
  }
  return sel;
}


void MPEPlanner::SelectTarget(const int rid)
{
  m_sim->SampleCfg(m_cfgTarget);

  double      coin = RandomUniformReal();
  std::vector<int> *path = &(m_abstract->m_regions[rid]->m_pathsToGoal[m_sim->m_id]);

  if(coin < m_goalBias)
  {
    m_cfgTarget[0] = m_sim->m_goal[0];
    m_cfgTarget[1] = m_sim->m_goal[1];
  }
  else if(coin < 0.9 && path->size() > 1)
  {
    const int n     = path->size();
    const int start = 1;//std::min<int>(5, n - 1);
    const int end   = std::min<int>(10, n - 1);
    m_abstract->SamplePointInsideRegion((*path)[RandomUniformInteger(start, end)], m_cfgTarget, m_dtol);
  }
}

int MPEPlanner::SelectVertexFromARData(const ARData * const data)
{
  const int    n    = data->m_vids.size();

  if(RandomUniformReal() < 1 - Constants::PLANNER_PROBABILITY_SELECT_NEAREST_VERTEX)
  {
    return data->m_vids[RandomUniformInteger(0, data->m_vids.size() - 1)];
  }

  double dmin = HUGE_VAL, d;
  int imin = -1;
  for(int i = 0; i < n; ++i)
  {
    d = m_sim->DistanceCfg(m_cfgTarget, m_vertices[data->m_vids[i]]->m_cfg);
    if(d < dmin)
    {
      imin = i;
      dmin = d;
    }

  }
  return data->m_vids[imin];
}

}
