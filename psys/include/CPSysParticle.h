#ifndef CPSysParticle_H
#define CPSysParticle_H

#include <CPSysVector3D.h>

class CPSysParticle {
 public:
  CPSysParticle(double m=1.0);

  virtual ~CPSysParticle() { }

  const uint &ind() const { return ind_; }
  void setInd(const uint &v) { ind_ = v; }

  CPSysVector3D *position() const { return position_; }
  void setPosition(double x, double y, double z);

  CPSysVector3D *velocity() const { return velocity_; }
  void setVelocity(double x, double y, double z);

  double mass() const { return mass_; }
  void setMass(double m);

  CPSysVector3D *force() const { return force_; }
  void setForce(double x, double y, double z);

  double age() const { return age_; }
  void setAge(double a) { age_ = a; }

  bool isDead() const { return dead_; }
  void setDead(bool b) { dead_ = b; }

  //---

  void makeFixed();

  bool isFixed() const { return   fixed_; }
  bool isFree () const { return ! fixed_; };

  void makeFree();

  //---

  double distanceTo(CPSysParticle *p) const;

  void reset();

 private:
  uint           ind_      { 0 };
  CPSysVector3D *position_ { nullptr };
  CPSysVector3D *velocity_ { nullptr };
  CPSysVector3D *force_    { nullptr };
  double         mass_     { 1.0 };
  double         age_      { 0.0 };
  bool           dead_     { false };
  bool           fixed_    { false };
};

#endif
