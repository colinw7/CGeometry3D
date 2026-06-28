#include <ParticleSystem.h>
#include <GeomObject.h>

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
  auto *particle = new Particle;

  if (particleObj_) {
    particle->setObj(particleObj_);

    particleObj_->setParticle(particle);
  }

  particle->setMass(mass);

  particle->position()->set(x, y, z);

  addParticle(particle);

  if (particleObj_)
    particleObj_->setTranslate(x, y, z);

  return particle;
}

//---

Particle::
Particle(double mass) :
 CPSysParticle(mass)
{
}

void
Particle::
updateParticle()
{
  if (obj_) {
    auto *pos = position();

    obj_->setTranslate(pos->x(), pos->y(), pos->z());
  }
}

}
