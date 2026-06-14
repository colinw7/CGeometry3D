#ifndef CPSysAttraction_H
#define CPSysAttraction_H

#include <CPSysParticle.h>
#include <CPSysForce.h>

// k: attract positive, repel negative

class CPSysAttraction : public CPSysForce {
 public:
  CPSysAttraction(CPSysParticle *a, CPSysParticle *b, double k=0.0, double distanceMin=0.0);

  virtual ~CPSysAttraction() { }

 public:
  const uint &ind() const { return ind_; }
  void setInd(const uint &v) { ind_ = v; }

  const CPSysParticle *getOneEnd     () const { return a_; }
  const CPSysParticle *getTheOtherEnd() const { return b_; }

  void setA(CPSysParticle *p);
  void setB(CPSysParticle *p);

  double getStrength() const { return k_; }
  void setStrength(double k);

  bool isOn () const override { return   on_; }
  bool isOff() const override { return ! on_; }

  void turnOff() override;
  void turnOn () override;

  double getMinimumDistance() const { return distanceMin_; }
  void setMinimumDistance(double d);

  void apply() override;

 private:
  uint           ind_                { 0 };
  CPSysParticle *a_                  { nullptr };
  CPSysParticle *b_                  { nullptr };
  double         k_                  { 0.0 };
  bool           on_                 { true };
  double         distanceMin_        { 0.0 };
  double         distanceMinSquared_ { 0.0 };
};

#endif
