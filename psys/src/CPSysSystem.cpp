#include <CPSysSystem.h>
#include <CPSysRungeKuttaIntegrator.h>
#include <CPSysModifiedEulerIntegrator.h>

#include <cassert>

CPSysSystem::
CPSysSystem(double g, double somedrag)
{
  CPSysSystem::setIntegrator(RUNGE_KUTTA);

  gravity_ = new CPSysVector3D(0, g, 0);
  drag_    = somedrag;
}

CPSysSystem::
CPSysSystem(double gx, double gy, double gz, double somedrag)
{
  CPSysSystem::setIntegrator(RUNGE_KUTTA);

  gravity_ = new CPSysVector3D(gx, gy, gz);
  drag_    = somedrag;
}

CPSysSystem::
~CPSysSystem()
{
}

void
CPSysSystem::
setIntegrator(int type)
{
  delete integrator_;

  switch (type) {
    case RUNGE_KUTTA:
      integrator_ = new CPSysRungeKuttaIntegrator(this);
      break;
    case MODIFIED_EULER:
      integrator_ = new CPSysModifiedEulerIntegrator(this);
      break;
    default:
      assert(false);
      break;
  }
}

// default down gravity
void
CPSysSystem::
setGravity(double g)
{
  setGravity(0, g, 0);
}

void
CPSysSystem::
setGravity(double x, double y, double z)
{
  gravity_->set(x, y, z);
}

void
CPSysSystem::
setDrag(double d)
{
  drag_ = d;
}

void
CPSysSystem::
tick(double t)
{
  integrator_->step(t);
}

CPSysParticle *
CPSysSystem::
makeParticle(double mass, double x, double y, double z)
{
  auto *p = new CPSysParticle(mass);

  p->position()->set(x, y, z);

  addParticle(p);

  return p;
}

void
CPSysSystem::
addParticle(CPSysParticle *p)
{
  p->setInd(particles_.size());

  particles_.add(p);
}

CPSysSpring *
CPSysSystem::
makeSpring(CPSysParticle *a, CPSysParticle *b, double ks, double d, double r)
{
  auto *s = new CPSysSpring(a, b, ks, d, r);

  addSpring(s);

  return s;
}

void
CPSysSystem::
addSpring(CPSysSpring *s)
{
  s->setInd(springs_.size());

  springs_.add(s);
}

CPSysAttraction *
CPSysSystem::
makeAttraction(CPSysParticle *a, CPSysParticle *b, double k, double minDistance)
{
  auto *m = new CPSysAttraction(a, b, k, minDistance);

  addAttraction(m);

  return m;
}

void
CPSysSystem::
addAttraction(CPSysAttraction *a)
{
  a->setInd(attractions_.size());

  attractions_.add(a);
}

void
CPSysSystem::
clear()
{
  particles_  .clear();
  springs_    .clear();
  attractions_.clear();
}

void
CPSysSystem::
applyForces()
{
  if (! gravity_->isZero()) {
    for (uint i = 0; i < particles_.size(); ++i) {
      auto *p = particles_.get(int(i));

      p->force()->add(gravity_);
    }
  }

  for (uint i = 0; i < particles_.size(); ++i) {
    auto *p = particles_.get(int(i));

    p->force()->add(p->velocity()->x() * -drag_,
                    p->velocity()->y() * -drag_,
                    p->velocity()->z() * -drag_);
  }

  for (uint i = 0; i < springs_.size(); i++) {
    auto *f = springs_.get(int(i));

    f->apply();
  }

  for (uint i = 0; i < attractions_.size(); i++) {
    auto *f = attractions_.get(int(i));

    f->apply();
  }

  for (uint i = 0; i < customForces_.size(); i++) {
    auto *f = customForces_.get(int(i));

    f->apply();
  }
}

void
CPSysSystem::
clearForces()
{
  ParticleArray::Iterator i = particles_.iterator();

  while (i.hasNext()) {
    auto *p = i.next();

    p->force()->clear();
  }
}

uint
CPSysSystem::
numberOfParticles()
{
  return particles_.size();
}

uint
CPSysSystem::
numberOfSprings()
{
  return springs_.size();
}

uint
CPSysSystem::
numberOfAttractions()
{
  return attractions_.size();
}

CPSysParticle *
CPSysSystem::
getParticle(uint i)
{
  return particles_.get(i);
}

CPSysSpring *
CPSysSystem::
getSpring(uint i)
{
  return springs_.get(i);
}

CPSysAttraction *
CPSysSystem::
getAttraction(uint i)
{
  return attractions_.get(i);
}

void
CPSysSystem::
addCustomForce(CPSysForce * f)
{
  customForces_.add(f);
}

uint
CPSysSystem::
numberOfCustomForces()
{
  return customForces_.size();
}

CPSysForce *
CPSysSystem::
getCustomForce(int i)
{
  return customForces_.get(i);
}

CPSysForce *
CPSysSystem::
removeCustomForce(int i)
{
  return customForces_.remove(i);
}

void
CPSysSystem::
removeParticle(CPSysParticle * p)
{
  particles_.remove(p);
}

CPSysSpring *
CPSysSystem::
removeSpring(int i)
{
  return springs_.remove(i);
}

CPSysAttraction *
CPSysSystem::
removeAttraction(int i)
{
  return attractions_.remove(i);
}

void
CPSysSystem::
removeAttraction(CPSysAttraction * s)
{
  attractions_.remove(s);
}

void
CPSysSystem::
removeSpring(CPSysSpring * a)
{
  springs_.remove(a);
}

void
CPSysSystem::
removeCustomForce(CPSysForce * f)
{
  customForces_.remove(f);
}
