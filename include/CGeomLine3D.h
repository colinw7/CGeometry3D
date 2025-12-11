#ifndef CGEOM_LINE_3D_H
#define CGEOM_LINE_3D_H

#include <CGeomVertex3D.h>
#include <CMaterial.h>
#include <CLineDash.h>

class CGeomObject3D;

class CGeomLine3D {
 public:
  CGeomLine3D(CGeomObject3D *pobject, uint start_ind, uint end_ind);

  CGeomLine3D(const CGeomLine3D &line);

  virtual ~CGeomLine3D() { }

  virtual CGeomLine3D *dup() const;

  CGeomObject3D *getObject() const { return pobject_; }

  void setObject(CGeomObject3D *object) { pobject_ = object; }

  const uint &getInd() const { return ind_; }
  void setInd(const uint &i) { ind_ = i; }

  void setColor(const CRGBA &rgba) {
    material_.setColor(rgba);
  }

  CRGBA getColor() const { return material_.getColor(); }

  CGeomVertex3D &getStartVertex() const;
  CGeomVertex3D &getEndVertex  () const;

  const CMaterial &getMaterial() const { return material_; }

  void setWidth(double width) { width_ = width; }

  double getWidth() const { return width_; }

  void setDashes(const CLineDash &dashes) { dashes_ = dashes; }

  const CLineDash &getDashes() const { return dashes_; }

  void moveTo(const CPoint3D &v1, const CPoint3D &v2);

  void draw(CGeomZBuffer *zbuffer);

 private:
  CGeomLine3D &operator=(const CGeomLine3D &rhs);

 protected:
  CGeomObject3D *pobject_ { nullptr };
  uint           ind_ { 0 };
  uint           start_ind_;
  uint           end_ind_;
  CMaterial      material_;
  double         width_ { 1 };
  CLineDash      dashes_;
};

#endif
