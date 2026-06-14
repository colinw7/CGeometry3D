#include <CPSysEulerIntegrator.h>
#include <CPSysSystem.h>

CPSysEulerIntegrator::
CPSysEulerIntegrator(CPSysSystem *s) :
 s_(s)
{
}

void
CPSysEulerIntegrator::
step(double t)
{
  s_->clearForces();
  s_->applyForces();

  uint numParticles = s_->numberOfParticles();

  for (uint i = 0; i < numParticles; i++) {
    auto *p = s_->getParticle(int(i));

    if (p->isFree()) {
      p->velocity()->add(p->force()->x()/(p->mass()*t),
                         p->force()->y()/(p->mass()*t),
                         p->force()->z()/(p->mass()*t));

      p->position()->add(p->velocity()->x()/t,
                         p->velocity()->y()/t,
                         p->velocity()->z()/t);
    }
  }
}
