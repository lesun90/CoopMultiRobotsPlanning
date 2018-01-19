#ifndef MP_CCD_IK_HPP_
#define MP_CCD_IK_HPP_

#include <vector>
#include <math.h>

namespace MP
{
  class CCDInverseKinematics
  {
  public:

    CCDInverseKinematics(void);

    virtual ~CCDInverseKinematics(void)
    {

    }

    virtual double SimplifyAngle(double theta)
    {
      return theta - 2 * M_PI * floor((theta + M_PI) / (2 * M_PI));
    }

    virtual double PointDist(double p1[], double p2[])
    {
      return sqrt((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]));
    }

    class JointData
    {
      public:
      JointData()
      {
        x = 0;
        y = 0;
        length = 0;
        angle = 0;
      }
      double x;        // x position in world space
      double y;        // y position in world space
      double length;
      double angle;    // angle in world space
    };

    virtual void CalcIK_2D_CCD ();

    std::vector<JointData> m_joints;
    double m_target[2];
    int nrIter;


  };
}

#endif
