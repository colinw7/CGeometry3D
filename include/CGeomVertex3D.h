#ifndef CGEOM_VERTEX_3D_H
#define CGEOM_VERTEX_3D_H

#include <CPoint3D.h>
#include <CMatrix3D.h>
#include <CRGBA.h>
#include <CClipSide.h>
#include <CGeomPoint3D.h>
#include <accessor.h>
#include <vector>
#include <optional>

class CGeomObject3D;
class CGeomZBuffer;
class CGeomCamera3D;
class CGeomFace3D;

class CGeomVertex3D : public CGeomPoint3D {
 public:
  struct JointNodeData {
    int    node   { -1 };
    double weight { 0.0 };
  };

  struct JointData {
    JointNodeData nodeDatas[4];
  };

 public:
  CGeomVertex3D(CGeomObject3D *pobject, const CPoint3D &point=CPoint3D(0,0,0));

  CGeomVertex3D(const CGeomVertex3D &vertex);

  virtual ~CGeomVertex3D() { }

  CGeomVertex3D &operator=(const CGeomVertex3D &rhs);

  virtual CGeomVertex3D *dup() const;

  //---

  CGeomObject3D *getObject() const { return pobject_; }
  void setObject(CGeomObject3D *object) { pobject_ = object; }

  const uint &getInd() const { return ind_; }
  void setInd(const uint &i) { ind_ = i; }

  bool hasColor() const { return bool(color_); }
  CRGBA getColor(const CRGBA &c=CRGBA(1, 1, 1)) const { return color_.value_or(c); }
  void setColor(const CRGBA &c) { color_ = c; }

  bool hasNormal() const { return bool(normal_); }
  CVector3D getNormal(const CVector3D &n=CVector3D(0, 0, 1)) const { return normal_.value_or(n); }
  void setNormal(const CVector3D &n) { normal_ = n; }

  bool hasTextureMap() const { return bool(tmap_); }
  CPoint2D getTextureMap(const CPoint2D &p=CPoint2D(0, 0)) const { return tmap_.value_or(p); }
  void setTextureMap(const CPoint2D &p) { tmap_ = p; }

  const CClipSide &getClipSide() const { return clipSide_; }
  void setClipSide(const CClipSide &v) { clipSide_ = v; }

  const JointData &getJointData() const { return jointData_; }
  void setJointData(const JointData &data) { jointData_ = data; }

  //---

  void draw(CGeomZBuffer *zbuffer);

  void print(std::ostream &os) const {
    CGeomPoint3D::print(os);

    if (hasColor())
      os << ", color=" << getColor();

    if (hasTextureMap())
      os << ", tmap=" << getTextureMap();
  }

  friend std::ostream &operator<<(std::ostream &os, const CGeomVertex3D &vertex) {
    vertex.print(os);

    return os;
  }

 protected:
  using FaceList  = std::vector<CGeomFace3D *>;
  using OptVector = std::optional<CVector3D>;
  using OptColor  = std::optional<CRGBA>;
  using OptPoint2 = std::optional<CPoint2D>;

  CGeomObject3D *pobject_ { nullptr };

  uint      ind_ { 0 };
  OptColor  color_;
  OptVector normal_;
  OptPoint2 tmap_;
  CClipSide clipSide_ { CCLIP_SIDE_NONE };
  JointData jointData_;
};

#endif
