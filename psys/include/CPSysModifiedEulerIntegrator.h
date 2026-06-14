#ifndef CPSysModifierEulerIntegrator_H
#define CPSysModifierEulerIntegrator_H

#include <CPSysIntegrator.h>

class CPSysSystem;

class CPSysModifiedEulerIntegrator : public CPSysIntegrator {
 public:
  CPSysModifiedEulerIntegrator(CPSysSystem *s);

  void step(double t) override;

 private:
  CPSysSystem *s_ { nullptr };
};

#endif
