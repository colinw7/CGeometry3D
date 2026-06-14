#include <ParticleSystem.h>

namespace CQTclModel3DView {

ParticleSystem::
ParticleSystem() :
 CPSysSystem(-1.0, 0.1)
{
}

CPSysParticle *
ParticleSystem::
makeParticle(double mass, double x, double y, double z)
{
  auto *p = new Particle(mass);

  p->position()->set(x, y, z);

  addParticle(p);

  return p;
}

Particle::
Particle(double mass) :
 CPSysParticle(mass)
{
}

}
