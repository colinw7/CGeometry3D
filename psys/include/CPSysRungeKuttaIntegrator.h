#ifndef CPSysRungeKuttaIntegrator_H
#define CPSysRungeKuttaIntegrator_H

#include <CPSysIntegrator.h>
#include <CArrayList.h>

class CPSysSystem;
class CPSysVector3D;

class CPSysRungeKuttaIntegrator : public CPSysIntegrator {
 public:
  CPSysRungeKuttaIntegrator(CPSysSystem *s);

  void allocateParticles();

  void step(double deltaT) override;

 private:
  using Vector3DArray = CArrayList<CPSysVector3D *>;

  Vector3DArray originalPositions_;
  Vector3DArray originalVelocities_;
  Vector3DArray k1Forces_;
  Vector3DArray k1Velocities_;
  Vector3DArray k2Forces_;
  Vector3DArray k2Velocities_;
  Vector3DArray k3Forces_;
  Vector3DArray k3Velocities_;
  Vector3DArray k4Forces_;
  Vector3DArray k4Velocities_;

  CPSysSystem *s_ { nullptr };
};

#endif
