#ifndef CGeomEdge3D_H
#define CGeomEdge3D_H

#include <CVector3D.h>

#include <sys/types.h>

class CGeomObject3D;
class CGeomFace3D;

class CGeomEdge3D {
 public:
  CGeomEdge3D();

  CGeomEdge3D(uint v1, uint v2);

  virtual ~CGeomEdge3D();

  CGeomObject3D *getObject() const { return object_; }
  void setObject(CGeomObject3D *object) { object_ = object; }

  const uint &getInd() const { return ind_; }
  void setInd(const uint &v) { ind_ = v; }

  uint getStart() const { return vertices_[0]; }
  void setStart(uint v1) { vertices_[0] = v1; }

  uint getEnd() const { return vertices_[1]; }
  void setEnd(uint v2) { vertices_[1] = v2; }

  std::vector<uint> getVertices() const { return vertices_; }

  //---

  bool getSelected() const { return selected_; }
  virtual void setSelected(bool b) { selected_ = b; }

  bool getVisible() const { return visible_; }
  virtual void setVisible(bool b) { visible_ = b; }

  //---

  virtual bool cmp(const CGeomEdge3D &rhs) const;

  //---

  void moveBy(const CVector3D &v);

  void scale(double s);

  CGeomFace3D *extrude(double d) const;

  CGeomFace3D *bevel(double s);

  CVector3D vector() const;

  CVector3D calcNormal() const;

  //---

  double distanceTo(const CPoint3D &p) const;

  CPoint3D calcCenter() const;

 private:
  CGeomObject3D*    object_   { nullptr };
  uint              ind_      { 0 };
  std::vector<uint> vertices_;
  bool              selected_ { false };
  bool              visible_  { true };
};

#endif
