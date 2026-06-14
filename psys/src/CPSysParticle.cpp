#include <CPSysParticle.h>

CPSysParticle::
CPSysParticle(double m) :
 mass_(m)
{
  position_ = new CPSysVector3D();
  velocity_ = new CPSysVector3D();
  force_    = new CPSysVector3D();
}

double
CPSysParticle::
distanceTo(CPSysParticle *p) const
{
  return position()->distanceTo(p->position());
}

void
CPSysParticle::
makeFixed()
{
  fixed_ = true;

  velocity_->clear();
}

void
CPSysParticle::
makeFree()
{
  fixed_ = false;
}

void
CPSysParticle::
setPosition(double x, double y, double z)
{
  position_->set(x, y, z);
}

void
CPSysParticle::
setVelocity(double x, double y, double z)
{
  velocity_->set(x, y, z);
}

void
CPSysParticle::
setForce(double x, double y, double z)
{
  force_->set(x, y, z);
}

void
CPSysParticle::
setMass(double m)
{
  mass_ = m;
}

void
CPSysParticle::
reset()
{
  age_  = 0;
  dead_ = false;

  position_->clear();
  velocity_->clear();
  force_   ->clear();

  mass_ = 1.0f;
}
