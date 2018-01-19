#ifndef MP_FLAGS_HPP_
#define MP_FLAGS_HPP_

#include "Utils/Definitions.hpp"

namespace MP
{
  enum
  {
    MP_FLAG_PAUSE                   = 1 << 0,
    MP_FLAG_ANIMATE                 = 1 << 1,
    MP_FLAG_DRAW_ABSTRACT_REGIONS   = 1 << 2,
    MP_FLAG_DRAW_ABSTRACT_EDGES     = 1 << 3,
    MP_FLAG_CAMERA_FOLLOW           = 1 << 4,
    MP_FLAG_STEER                   = 1 << 5,
    MP_FLAG_NEIGHS                  = 1 << 6,
    MP_FLAG_SIM_DRAW_PROJ_PROPS     = 1 << 7,
    MP_FLAG_SIM_DRAW_PROJ_OBSTACLES = 1 << 8,
    MP_FLAG_PLANNER_DRAW_VERTICES   = 1 << 9,
    MP_FLAG_PLANNER_DRAW_EDGES      = 1 << 10,
    MP_FLAG_SYCLOP_DRAW_GUIDE       = 1 << 11,
    MP_FLAG_DRAW_OBSTACLES          = 1 << 12,
    MP_FLAG_SPLIT                   = 1 << 13
  };

  typedef unsigned int Flags;

  static inline
  bool HasFlag(const Flags f, const Flags b)
  {
    return f & b;
  }

  static inline
  Flags AddFlag(const Flags f, const Flags b)
  {
    return f | b;
  }


  static inline
  Flags RemoveFlag(const Flags f, const Flags b)
  {
    return f & (~b);
  }

  static inline
  Flags FlipFlag(const Flags f, const Flags b)
  {
    return f ^ b;
  }
}

#endif
