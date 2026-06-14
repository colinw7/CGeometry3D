#ifndef CPSysIntegrator_H
#define CPSysIntegrator_H

class CPSysIntegrator {
 public:
  virtual ~CPSysIntegrator() { }

  virtual void step(double t) = 0;
};

#endif
