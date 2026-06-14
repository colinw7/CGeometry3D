#ifndef CPSysEulerIntegrator_H
#define CPSysEulerIntegrator_H

#include <CPSysIntegrator.h>

class CPSysSystem;

class CPSysEulerIntegrator : public CPSysIntegrator {
 public:
  CPSysEulerIntegrator(CPSysSystem *s);

  void step(double t) override;

 private:
  CPSysSystem *s_ { nullptr };
};

#endif
