#ifndef CPSysSystem_H
#define CPSysSystem_H

#include <CPSysParticle.h>
#include <CPSysSpring.h>
#include <CPSysAttraction.h>
#include <CPSysForce.h>
#include <CPSysIntegrator.h>

#include <CArrayList.h>

class CPSysSystem {
 public:
  enum { RUNGE_KUTTA    = 0 };
  enum { MODIFIED_EULER = 1 };

 public:
  using ParticleArray   = CArrayList<CPSysParticle * >;
  using SpringArray     = CArrayList<CPSysSpring *   >;
  using AttractionArray = CArrayList<CPSysAttraction*>;
  using ForceArray      = CArrayList<CPSysForce *    >;

 public:
  explicit CPSysSystem(double g, double somedrag=0.001);
  CPSysSystem(double gx, double gy, double gz, double somedrag=0.001);

  virtual ~CPSysSystem();

  void setIntegrator(int type);

  // default down gravity
  CPSysVector3D *getGravity() const { return gravity_; }
  void setGravity(double g);
  void setGravity(double x, double y, double z);

  double drag() const { return drag_; }
  void setDrag(double d);

  void tick(double t=1.0);

  virtual CPSysParticle *makeParticle(double mass=1.0, double x=0.0, double y=0.0, double z=0.0);

  virtual CPSysSpring *makeSpring(CPSysParticle *a, CPSysParticle *b,
                                  double ks=0.2, double d=0.2, double r=1.0);

  virtual CPSysAttraction *makeAttraction(CPSysParticle *a, CPSysParticle *b,
                                          double k=0.0, double minDistance=0.0);

  void addParticle(CPSysParticle *p);
  void addSpring(CPSysSpring *s);
  void addAttraction(CPSysAttraction *a);

  void clear();

  void applyForces();

  void clearForces();

  uint numberOfParticles();

  uint numberOfSprings();

  uint numberOfAttractions();

  CPSysParticle *getParticle(uint i);
  const ParticleArray &getParticles() const { return particles_; }

  CPSysSpring *getSpring(uint i);

  CPSysAttraction *getAttraction(uint i);

  void addCustomForce(CPSysForce *f);

  uint numberOfCustomForces();

  CPSysForce *getCustomForce   (int i);
  CPSysForce *removeCustomForce(int i);

  void removeParticle(CPSysParticle *p);

  CPSysSpring *removeSpring(int i);

  CPSysAttraction *removeAttraction(int i);

  void removeAttraction(CPSysAttraction *s);

  void removeSpring(CPSysSpring *a);

  void removeCustomForce(CPSysForce *f);

 public:
  ParticleArray   particles_;
  SpringArray     springs_;
  AttractionArray attractions_;
  ForceArray      customForces_;

  CPSysVector3D *gravity_ { nullptr };
  double         drag_    { 0.0 };

  bool hasDeadParticles_ { false };

  CPSysIntegrator *integrator_ { nullptr };
};

#endif
