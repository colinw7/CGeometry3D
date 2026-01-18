#ifndef CGEOM_LINE_3D_H
#define CGEOM_LINE_3D_H

#include <CGeomVertex3D.h>
#include <CMaterial.h>
#include <CLineDash.h>

class CGeomObject3D;

class CGeomLine3D {
 public:
  enum {
    SELECTED = (1L<<0),
    VISIBLE  = (1L<<1)
  };

 public:
  CGeomLine3D();
  CGeomLine3D(CGeomObject3D *pobject, uint start_ind, uint end_ind);

  CGeomLine3D(const CGeomLine3D &line);

  CGeomLine3D &operator=(const CGeomLine3D &rhs) = delete;

  virtual ~CGeomLine3D() { }

  virtual CGeomLine3D *dup() const;

  //---

  CGeomObject3D *getObject() const { return pobject_; }
  void setObject(CGeomObject3D *object) { pobject_ = object; }

  //---

  const uint &getInd() const { return ind_; }
  void setInd(const uint &i) { ind_ = i; }

  //---

  bool getSelected() const { return (flags_ & SELECTED); }
  void setSelected(bool b);

  bool getVisible() const { return (flags_ & VISIBLE); }
  void setVisible(bool b);

  //---

  void setFlags  (uint flags) { flags_ |=  flags; }
  void unsetFlags(uint flags) { flags_ &= ~flags; }

  //---

  void setColor(const CRGBA &rgba) { material_.setColor(rgba); }
  CRGBA getColor() const { return material_.getColor(); }

  const CMaterial &getMaterial() const { return material_; }

  //---

  uint getStartInd() const { return startInd_; }
  uint getEndInd  () const { return endInd_; }

  void setVertices(uint startInd, uint endInd) {
    startInd_ = startInd;
    endInd_   = endInd;
  }

  CGeomVertex3D &getStartVertex() const;
  CGeomVertex3D &getEndVertex  () const;

  //---

  double getWidth() const { return width_; }
  void setWidth(double width) { width_ = width; }

  const CLineDash &getDashes() const { return dashes_; }
  void setDashes(const CLineDash &dashes) { dashes_ = dashes; }

  //---

  void moveTo(const CPoint3D &v1, const CPoint3D &v2);

  void draw(CGeomZBuffer *zbuffer);

 private:
  void init();

 protected:
  CGeomObject3D *pobject_ { nullptr }; // parent object

  uint ind_ { 0 }; // unique id

  uint flags_ { 0 }; // state flags

  uint startInd_ { 0 };
  uint endInd_   { 1 };

  CMaterial material_;

  double    width_ { 1 };
  CLineDash dashes_;
};

#endif
