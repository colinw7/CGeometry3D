#ifndef CPSysSpring_H
#define CPSysSpring_H

#include <CPSysForce.h>

#include <sys/types.h>

class CPSysParticle;

class CPSysSpring : public CPSysForce {
 public:
  CPSysSpring(CPSysParticle *a, CPSysParticle *b, double ks=0.2, double d=0.2, double r=1.0);

  virtual ~CPSysSpring() { }

  const uint &ind() const { return ind_; }
  void setInd(const uint &v) { ind_ = v; }

  void turnOff() override;
  void turnOn () override;

  bool isOn () const override { return   on_; }
  bool isOff() const override { return ! on_; }

  CPSysParticle *getOneEnd     () const { return a_; }
  CPSysParticle *getTheOtherEnd() const { return b_; }

  void setA(CPSysParticle *p);
  void setB(CPSysParticle *p);

  double strength() const { return springConstant_; }
  void setStrength(double ks);

  double damping() const { return damping_; }
  void setDamping(double d);

  double currentLength() const;

  double restLength() const { return restLength_; }
  void setRestLength(double l);

  void apply() override;

 private:
  uint           ind_            { 0 };
  CPSysParticle *a_              { nullptr };
  CPSysParticle *b_              { nullptr };
  double         springConstant_ { 0.2 };
  double         damping_        { 0.2 };
  double         restLength_     { 1.0 };
  bool           on_             { true };
};

#endif
