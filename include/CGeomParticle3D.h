#ifndef CGEOM_PARTICLE_3D_H
#define CGEOM_PARTICLE_3D_H

#include <CPoint3D.h>
#include <CMatrix3D.h>
#include <CRGBA.h>
#include <CClipSide.h>
#include <CGeomPoint3D.h>

class CGeomObject3D;
class CGeomZBuffer;
class CGeomCamera3D;
class CGeomFace3D;

class CGeomParticle3D : public CGeomPoint3D {
 public:
  CGeomParticle3D(const CPoint3D &point=CPoint3D(0,0,0)) :
   CGeomPoint3D(point) {
  }

  CGeomParticle3D(const CGeomParticle3D &particle);

  virtual ~CGeomParticle3D() { }

  const CRGBA &getColor() const { return color_; }
  void setCOlor(const CRGBA &v) { color_ = v; }

  const CVector3D &getNormal() const { return normal_; }
  void setNormal(const CVector3D &v) { normal_ = v; }

  int getSize() const { return size_; }
  void setSize(int i) { size_ = i; }

  void draw(CGeomZBuffer *zbuffer);

  void print(std::ostream &os) const {
    CGeomPoint3D::print(os);

    os << ", color=" << color_ << ", size="  << size_;
  }

  friend std::ostream &operator<<(std::ostream &os, const CGeomParticle3D &particle) {
    particle.print(os);

    return os;
  }

 protected:
  CRGBA     color_ { 0, 0, 0 };
  CVector3D normal_;
  int       size_ { 1 };
};

#endif
