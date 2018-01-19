#ifndef MP_RRT_HPP_
#define MP_RRT_HPP_

#include "MP/MPProximityPlanner.hpp"

namespace MP
{
  class MPRRT : public MPProximityPlanner
  {
  public:
    MPRRT(void) : MPProximityPlanner()
    {
    }

    virtual ~MPRRT(void)
    {
    }

    virtual void Run(const int nrIters);
  };
}

#endif
