#include <CPSysModifiedEulerIntegrator.h>
#include <CPSysSystem.h>

CPSysModifiedEulerIntegrator::
CPSysModifiedEulerIntegrator(CPSysSystem *s) :
 s_(s)
{
}

void
CPSysModifiedEulerIntegrator::
step(double t)
{
  s_->clearForces();
  s_->applyForces();

  double halftt = 0.5f*t*t;

  uint numParticles = s_->numberOfParticles();

  for (uint i = 0; i < numParticles; i++) {
    auto *p = s_->getParticle(int(i));

    if (p->isFree()) {
      double ax = p->force()->x()/p->mass();
      double ay = p->force()->y()/p->mass();
      double az = p->force()->z()/p->mass();

      p->position()->add(p->velocity()->x()/t, p->velocity()->y()/t, p->velocity()->z()/t);
      p->position()->add(ax*halftt, ay*halftt, az*halftt);
      p->velocity()->add(ax/t, ay/t, az/t);
    }
  }
}
