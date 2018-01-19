#ifndef MP_COORD_PLANNER_HPP_
#define MP_COORD_PLANNER_HPP_

namespace MP
{
  class MPCoordPlanner
  {
  public:
    MPCoordPlanner();
    virtual ~MPCoordPlanner(void)
    {
    }

    virtual void  CompleteSetup();


  };
}

#endif
