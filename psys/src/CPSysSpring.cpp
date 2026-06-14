#include <CPSysSpring.h>
#include <CPSysParticle.h>

CPSysSpring::
CPSysSpring(CPSysParticle *a, CPSysParticle *b, double ks, double d, double r) :
 a_(a), b_(b), springConstant_(ks), damping_(d), restLength_(r)
{
}

void
CPSysSpring::
turnOff()
{
  on_ = false;
}

void
CPSysSpring::
turnOn()
{
  on_ = true;
}

double
CPSysSpring::
currentLength() const
{
  return a_->position()->distanceTo(b_->position());
}

void
CPSysSpring::
setStrength(double ks)
{
  springConstant_ = ks;
}

void
CPSysSpring::
setDamping( double d)
{
  damping_ = d;
}

void
CPSysSpring::
setRestLength(double l)
{
  restLength_ = l;
}

void
CPSysSpring::
setA(CPSysParticle *p)
{
  a_ = p;
}

void
CPSysSpring::
setB(CPSysParticle *p)
{
  b_ = p;
}

void
CPSysSpring::
apply()
{
  if (! b_) return;

  if (on_ && (a_->isFree() || b_->isFree())) {
    double a2bX = a_->position()->x() - b_->position()->x();
    double a2bY = a_->position()->y() - b_->position()->y();
    double a2bZ = a_->position()->z() - b_->position()->z();

    double a2bDistance = sqrt(a2bX*a2bX + a2bY*a2bY + a2bZ*a2bZ);

    if (a2bDistance == 0) {
      a2bX = 0;
      a2bY = 0;
      a2bZ = 0;
    }
    else {
      a2bX /= a2bDistance;
      a2bY /= a2bDistance;
      a2bZ /= a2bDistance;
    }

    // spring force is proportional to how much it stretched

    double springForce = -(a2bDistance - restLength_)*springConstant_;

    // want velocity along line b/w a & b, damping force is proportional to this

    double Va2bX = a_->velocity()->x() - b_->velocity()->x();
    double Va2bY = a_->velocity()->y() - b_->velocity()->y();
    double Va2bZ = a_->velocity()->z() - b_->velocity()->z();

    double dampingForce = -damping_*(a2bX*Va2bX + a2bY*Va2bY + a2bZ*Va2bZ);

    // forceB is same as forceA in opposite direction

    double r = springForce + dampingForce;

    a2bX *= r;
    a2bY *= r;
    a2bZ *= r;

    if (a_->isFree()) a_->force()->add( a2bX,  a2bY,  a2bZ);
    if (b_->isFree()) b_->force()->add(-a2bX, -a2bY, -a2bZ);
  }
}
