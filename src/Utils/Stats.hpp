#ifndef MP_STATS_HPP_
#define MP_STATS_HPP_

#include "Utils/Map.hpp"
#include <cstdio>
#include <string>

namespace MP
{
  class Stats
  {
  public:
    Stats(void)
    {
    }

    virtual ~Stats(void)
    {
    }

    virtual double GetValue(const char id[]);
    virtual void SetValue(const char id[], const double t)
    {
      m_values[id] = t;
    }

    virtual double AddValue(const char id[], const double t);
    virtual void PrintValues(FILE *out) const;

    static Stats* GetSingleton(void)
    {
      return m_singleton;
    }

  protected:
    static Stats *m_singleton;

    UseMap(const std::string, double) m_values;
  };

}

#endif
