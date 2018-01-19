#ifndef MP_BSIMULATOR_HPP_
#define MP_BSIMULATOR_HPP_

#include "MP/MPSimulator.hpp"
#include "btBulletDynamicsCommon.h"

namespace MP
{
  class MPBSimulator : public MPSimulator
  {
  public:
    MPBSimulator(void);

    virtual ~MPBSimulator(void)
    {
    }

    //from MPSimulator class
    virtual void CompleteSetup(void);

    virtual void SimulateOneStep(void)
    {
      m_dynamicsWorld->stepSimulation(m_dt, 0, m_dt);
    }

    virtual void SettleInitialState(void)
    {
      MPState *s = NewState();

      for(int i = 0; i < 100; ++i)
      {
        SimulateOneStep();
        GetState(s);
      }
    }


  protected:
    btDynamicsWorld *m_dynamicsWorld;
    btRigidBody* CreateRigidBody(float mass,
      const btTransform& startTransform,
      btCollisionShape  *shape);

    btCollisionShape* TriMeshToCollisionShape(PQPTriMesh * const tmesh) const;
  };
}

#endif
