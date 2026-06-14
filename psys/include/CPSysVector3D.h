#ifndef CPSysVector3D_H
#define CPSysVector3D_H

#include <string>
#include <cmath>

class CPSysVector3D {
 public:
  CPSysVector3D(double x, double y, double z) :
   x_(x), y_(y), z_(z) {
  }

  CPSysVector3D() { }

  CPSysVector3D(const CPSysVector3D &p) { x_ = p.x_; y_ = p.y_; z_ = p.z_; }

  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }

  void setX(double x) { x_ = x; }
  void setY(double y) { y_ = y; }
  void setZ(double z) { z_ = z; }

  void set(double x, double y, double z) { x_ = x    ; y_ = y    ; z_ = z    ; }
  void set(CPSysVector3D * p)            { x_ = p->x_; y_ = p->y_; z_ = p->z_; }

  void add(double a, double b, double c) { x_ += a    ; y_ += b    ; z_ += c    ; }
  void add(CPSysVector3D * p)            { x_ += p->x_; y_ += p->y_; z_ += p->z_; }

  void subtract(double a, double b, double c) { x_ -= a    ; y_ -= b    ; z_ -= c    ; }
  void subtract(CPSysVector3D * p)            { x_ -= p->x_; y_ -= p->y_; z_ -= p->z_; }

  CPSysVector3D *multiplyBy(double f) { x_ *= f; y_ *= f; z_ *= f; return this; }

  double distanceTo(CPSysVector3D * p) const { return std::sqrt(distanceSquaredTo(p)); }

  double distanceSquaredTo(CPSysVector3D *p) const {
    double dx = x_ - p->x_; double dy = y_ - p->y_; double dz = z_ - p->z_;

    return dx*dx + dy*dy + dz*dz;
  }

  double distanceTo(double x, double y, double z) const {
    double dx = x_ - x;
    double dy = y_ - y;
    double dz = z_ - z;

    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }

  double dot(CPSysVector3D * p) const { return x_*p->x_ + y_*p->y_ + z_*p->z_; }

  double length()        const { return std::sqrt(x_*x_ + y_*y_ + z_*z_); }
  double lengthSquared() const { return x_*x_ + y_*y_ + z_*z_; }

  void clear() { x_ = 0; y_ = 0; z_ = 0; }

  std::string toString() const;

  CPSysVector3D * cross(CPSysVector3D * p) const {
    return new CPSysVector3D(y_*p->z_ - z_*p->y_,
                             x_*p->z_ - z_*p->x_,
                             x_*p->y_ - y_*p->x_);
  }

  bool isZero() const {
    return x_ == 0 && y_ == 0 && z_ == 0;
  }

 protected:
  double x_ { 0.0 };
  double y_ { 0.0 };
  double z_ { 0.0 };
};

#endif
