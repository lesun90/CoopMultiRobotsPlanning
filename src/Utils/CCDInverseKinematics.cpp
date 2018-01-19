#include "CCDInverseKinematics.hpp"
#include <stdio.h>

/*
CCD Inverse Kinematic
HOW TO USE:
- Set up joints data and target
- Call function CalcIK_2D_CCD to calculate Inverse kinematic
*/
namespace MP
{
  CCDInverseKinematics::CCDInverseKinematics(void)
  {
    JointData Joint1;
    Joint1.x = 0;
    Joint1.y = 0;
    Joint1.length = 2;
    Joint1.angle = 0;
    m_joints.push_back(Joint1);

    JointData Joint2;
    Joint2.length = 1;
    Joint2.angle = M_PI/2;
    m_joints.push_back(Joint2);

    JointData Joint3;
    Joint3.length = 1;
    Joint3.angle = -M_PI/2;
    m_joints.push_back(Joint3);

    m_target[0] = -1;
    m_target[1] = 1;
    nrIter = 10000;
  }

  void CCDInverseKinematics::CalcIK_2D_CCD (void)
  {
    // Set an epsilon value to prevent division by small numbers.
    const double minRange = 0.00001;//minimum range to the m_target
  	const double zero = 0.0001;//a value to avoid divide by 0

    int numJoints = m_joints.size();
    double endEffector[2];
    double disTotarget;
    int Iteration = 0;
    //===
    // Generate the world space Joint data.
    std::vector<JointData> simulateJoints;

    //value to store sum angle
		double phi = 0;
    // Start with the root Joint.
    JointData rootJoint;
    rootJoint.x = m_joints[0].x;
    rootJoint.y = m_joints[0].y;
    rootJoint.length = m_joints[0].length;
    rootJoint.angle = m_joints[0].angle;
    simulateJoints.push_back(rootJoint);
    phi += rootJoint.angle;

    //forward kinematic
    for( int jointIdx = 1; jointIdx < numJoints; ++jointIdx )
    {
      JointData prevJoint = simulateJoints[jointIdx-1];
      JointData newJoint;

      newJoint.x = prevJoint.x + prevJoint.length * cos(phi);
      newJoint.y = prevJoint.y + prevJoint.length * sin(phi);
      newJoint.length = m_joints[jointIdx].length;
      newJoint.angle = m_joints[jointIdx].angle;

      simulateJoints.push_back(newJoint);
      phi += newJoint.angle;
    }
    //get end effector
    endEffector[0] = simulateJoints.back().x + simulateJoints.back().length * cos(phi);
    endEffector[1] = simulateJoints.back().y + simulateJoints.back().length * sin(phi);
    disTotarget = PointDist(endEffector,m_target);

    printf("Before:\n");
    for( int jointIdx = 0; jointIdx < numJoints; ++jointIdx )
    {
      printf("joint %d angle %f\n",jointIdx, simulateJoints[jointIdx].angle );
    }
    printf("endEffector %f %f\n",endEffector[0], endEffector[1]);


    while ((disTotarget > minRange) && (Iteration < nrIter))
    {
      //forward kinematic
      phi = simulateJoints[0].angle;
      for( int jointIdx = 1; jointIdx < numJoints; ++jointIdx )
      {
        simulateJoints[jointIdx].x = simulateJoints[jointIdx-1].x + simulateJoints[jointIdx-1].length * cos(phi);
        simulateJoints[jointIdx].y = simulateJoints[jointIdx-1].y + simulateJoints[jointIdx-1].length * sin(phi);
        phi += simulateJoints[jointIdx].angle;
      }
      //get end effector
      endEffector[0] = simulateJoints.back().x + simulateJoints.back().length * cos(phi);
      endEffector[1] = simulateJoints.back().y + simulateJoints.back().length * sin(phi);
      disTotarget = PointDist(endEffector,m_target);

      for (int jointIdx = numJoints-1; jointIdx >= 0; --jointIdx)
      {
        //find vector from current joint to end joint
        double curToEnd[2];
        curToEnd[0] = endEffector[0] - simulateJoints[jointIdx].x;
        curToEnd[1] = endEffector[1] - simulateJoints[jointIdx].y;
        double curToEndMag = sqrt(curToEnd[0]*curToEnd[0]+curToEnd[1]*curToEnd[1]);
        //find vector from current joint to m_target
        double curToTar[2];
  			curToTar[0] = m_target[0] - simulateJoints[jointIdx].x;
  			curToTar[1] = m_target[1] - simulateJoints[jointIdx].y;
  			double curToTarMag = sqrt(curToTar[0]*curToTar[0]+curToTar[1]*curToTar[1]);

        //get rotate angle
        double cosAngle;
        double sinAngle;
        double endtargetMag = curToEndMag * curToTarMag;

        if (endtargetMag <= zero)
        {
  				cosAngle = 1;
  				sinAngle = 0;
  			}
        else
        {
  				cosAngle = (curToEnd[0] * curToTar[0] + curToEnd[1] * curToTar[1]) / endtargetMag;
  				sinAngle = (curToEnd[0] * curToTar[1] - curToEnd[1] * curToTar[0]) / endtargetMag;
  			}

        double rotAngle = acos(cosAngle);
        if (sinAngle < 0) {
          rotAngle = -rotAngle;
        }

        simulateJoints[jointIdx].angle = SimplifyAngle(simulateJoints[jointIdx].angle + rotAngle);
        endEffector[0] = simulateJoints[jointIdx].x + cosAngle * curToEnd[0] - sinAngle * curToEnd[1];
				endEffector[1] = simulateJoints[jointIdx].y + sinAngle * curToEnd[0] + cosAngle * curToEnd[1];

        disTotarget = PointDist(m_target,endEffector);
        Iteration++;
      }

    }

    printf("After:\n");
    for( int jointIdx = 0; jointIdx < numJoints; ++jointIdx )
    {
      m_joints[jointIdx].x = simulateJoints[jointIdx].x;
      m_joints[jointIdx].y = simulateJoints[jointIdx].y;
      m_joints[jointIdx].angle = simulateJoints[jointIdx].angle;
      printf("joint %d angle %f\n",jointIdx, m_joints[jointIdx].angle );
    }
    printf("endEffector %f %f\n",endEffector[0], endEffector[1]);


  }
}
