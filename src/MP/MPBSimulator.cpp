#include "MP/MPBSimulator.hpp"
#include "Utils/GDraw.hpp"

namespace MP
{
  MPBSimulator *gSimulator = NULL;


  MPBSimulator::MPBSimulator(void) : MPSimulator()
  {
    gSimulator      = this;

    btVector3 worldMin(-1000,-1000,-1000);
    btVector3 worldMax(1000,1000,1000);

    btDefaultCollisionConfiguration *ccfg = new btDefaultCollisionConfiguration();
    btCollisionDispatcher           *cdis = new btCollisionDispatcher(ccfg);
    btBroadphaseInterface           *opc  = new btAxisSweep3(worldMin,worldMax);
    btConstraintSolver              *cs   = new btSequentialImpulseConstraintSolver();

    m_dynamicsWorld = new btDiscreteDynamicsWorld(cdis, opc, cs, ccfg);
    m_dynamicsWorld->setGravity(btVector3(0, 0, -9.81));

    /*
    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(btVector3(0,0,-0.2));
    CreateRigidBody(0, tr,  new btBoxShape(btVector3(40, 40, 0.2)));
    */
  }

  void MPBSimulator::CompleteSetup(void)
  {
    MPSimulator::CompleteSetup();
    btTransform tr;

    tr.setIdentity();
    btCollisionShape *shape = NULL;

    if(&m_scene->m_obstacles.m_tmesh)
    {
      shape = TriMeshToCollisionShape(&m_scene->m_obstacles.m_tmesh);
      CreateRigidBody(0, tr, shape);
    }


    shape = TriMeshToCollisionShape(m_scene->m_ground);
    CreateRigidBody(0, tr, shape);


  }


  btRigidBody*  MPBSimulator::CreateRigidBody(float mass,
    const btTransform& startTransform,
    btCollisionShape  *shape)
    {
      btVector3 localInertia(0,0,0);
      bool isDynamic = (mass != 0.f);
      if (isDynamic) //dynamic
      shape->calculateLocalInertia(mass,localInertia);

      btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
      body->setWorldTransform(startTransform);
      m_dynamicsWorld->addRigidBody(body);

      return body;
      /*
      btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

      //rigidbody is dynamic if and only if mass is non zero, otherwise static
      bool isDynamic = (mass != 0.f);

      btVector3 localInertia(0,0,0);
      if (isDynamic)
      shape->calculateLocalInertia(mass,localInertia);

      btRigidBody::btRigidBodyConstructionInfo cInfo(mass, NULL, shape, localInertia);
      cInfo.m_startWorldTransform = startTransform;

      btRigidBody* body = new btRigidBody(cInfo);
      body->setContactProcessingThreshold(BT_LARGE_FLOAT);

      m_dynamicsWorld->addRigidBody(body);

      return body;
      */
    }

    btCollisionShape* MPBSimulator::TriMeshToCollisionShape(PQPTriMesh * const tmesh) const
    {
      const int vertStride     = sizeof(btVector3);
      const int indexStride    = 3 * sizeof(int);
      const int totalVerts     = tmesh->GetNrVertices();
      const int totalTriangles = tmesh->GetNrFaces();

      btVector3 *vertices      = new btVector3[totalVerts];
      int       *gIndices      = new int[totalTriangles * 3];

      for(int i = 0; i < totalVerts; ++i)
      {
        const double *v = tmesh->GetVertex(i);
        vertices[i].setValue(v[0], v[1], v[2]);
      }

      for(int i = 0; i < totalTriangles; ++i)
      {
        const TriMesh::Face *face = tmesh->GetFace(i);

        gIndices[3 * i]     = face->m_vids[0];
        gIndices[3 * i + 1] = face->m_vids[1];
        gIndices[3 * i + 2] = face->m_vids[2];
      }

      btTriangleIndexVertexArray *indexVertexArrays =
      new btTriangleIndexVertexArray(totalTriangles,
        gIndices,
        indexStride,
        totalVerts,(btScalar*) &vertices[0].x(),vertStride);

        return new btBvhTriangleMeshShape(indexVertexArrays, true);
      }
    }
