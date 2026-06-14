#include <CPSysAttraction.h>

CPSysAttraction::
CPSysAttraction(CPSysParticle *a, CPSysParticle *b, double k, double distanceMin) :
 a_(a), b_(b), k_(k), distanceMin_(distanceMin)
{
  distanceMinSquared_ = distanceMin_*distanceMin_;
}

void
CPSysAttraction::
setA(CPSysParticle *p)
{
  a_ = p;
}

void
CPSysAttraction::
setB(CPSysParticle *p)
{
  b_ = p;
}

void
CPSysAttraction::
setMinimumDistance(double d)
{
  distanceMin_        = d;
  distanceMinSquared_ = d*d;
}

void
CPSysAttraction::
turnOff()
{
  on_ = false;
}

void
CPSysAttraction::
turnOn()
{
  on_ = true;
}

void
CPSysAttraction::
setStrength(double k)
{
  k_ = k;
}

void
CPSysAttraction::
apply()
{
  if (! b_) return;

  if (on_ && (a_->isFree() || b_->isFree())) {
    double a2bX = a_->position()->x() - b_->position()->x();
    double a2bY = a_->position()->y() - b_->position()->y();
    double a2bZ = a_->position()->z() - b_->position()->z();

    double a2bDistanceSquared = a2bX*a2bX + a2bY*a2bY + a2bZ*a2bZ;

    if (a2bDistanceSquared < distanceMinSquared_)
      a2bDistanceSquared = distanceMinSquared_;

    double force = k_ * a_->mass() * b_->mass() / a2bDistanceSquared;

    double length = sqrt(a2bDistanceSquared);

    // make unit vector

    a2bX /= length;
    a2bY /= length;
    a2bZ /= length;

    // multiply by force

    a2bX *= force;
    a2bY *= force;
    a2bZ *= force;

    // apply

    if (a_->isFree()) a_->force()->add(-a2bX, -a2bY, -a2bZ);
    if (b_->isFree()) b_->force()->add( a2bX,  a2bY,  a2bZ);
  }
}
