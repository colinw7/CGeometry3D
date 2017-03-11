#ifndef CGEOM_VERTEX_3D_H
#define CGEOM_VERTEX_3D_H

#include <CPoint3D.h>
#include <CMatrix3D.h>
#include <CRGBA.h>
#include <CClipSide.h>
#include <COptVal.h>
#include <CGeomPoint3D.h>
#include <accessor.h>
#include <vector>

class CGeomObject3D;
class CGeomZBuffer;
class CGeomCamera3D;
class CGeomFace3D;

class CGeomVertex3D : public CGeomPoint3D {
 public:
  CGeomVertex3D(CGeomObject3D *pobject, const CPoint3D &point=CPoint3D(0,0,0));

  CGeomVertex3D(const CGeomVertex3D &vertex);

  virtual ~CGeomVertex3D() { }

  CGeomVertex3D &operator=(const CGeomVertex3D &rhs);

  virtual CGeomVertex3D *dup() const;

  CGeomObject3D *getObject() const { return pobject_; }

  void setObject(CGeomObject3D *object) { pobject_ = object; }

  ACCESSOR(Ind       , uint     , ind      )
  ACCESSOR(Color     , CRGBA    , color    )
  ACCESSOR(VNormal   , CVector3D, vnormal  )
  ACCESSOR(TextureMap, CPoint3D , tmap     )
  ACCESSOR(ClipSide  , CClipSide, clip_side)

  const CVector3D &getNormal() const {
    return normal_.getValue();
  }

  void setNormal(const CVector3D &normal) {
    normal_.setValue(normal);

    vnormal_ = normal;
  }

  bool getNormalSet() const {
    return normal_.isValid();
  }

  void draw(CGeomZBuffer *zbuffer);

  void print(std::ostream &os) const {
    CGeomPoint3D::print(os);

    os << ", color=" << color_ << ", tmap=" << tmap_;
  }

  friend std::ostream &operator<<(std::ostream &os, const CGeomVertex3D &vertex) {
    vertex.print(os);

    return os;
  }

 protected:
  typedef std::vector<CGeomFace3D *> FaceList;

  CGeomObject3D       *pobject_ { nullptr };
  uint                 ind_ { 0 };
  CRGBA                color_ { 0, 0, 0 };
  COptValT<CVector3D>  normal_;
  CVector3D            vnormal_;
  CPoint3D             tmap_;
  CClipSide            clip_side_ { CCLIP_SIDE_NONE };
};

#endif
