#ifndef CGEOMETRY_H
#define CGEOMETRY_H

#define CGeometry3DInst CGeometry3D::getInstance()

#include <CGeomVertex3D.h>
#include <CGeomLine3D.h>
#include <CGeomFace3D.h>
#include <CGeomScene3D.h>
#include <CGeomTexture.h>
#include <CGeomMask.h>
#include <CGeomCamera3D.h>

// default factory class
class CGeometryFactory {
 public:
  CGeometryFactory();

  virtual ~CGeometryFactory();

  virtual CGeomVertex3D *createVertex3D(CGeomObject3D *pobject, const CPoint3D &point) const;

  virtual CGeomLine3D *createLine3D() const;

  virtual CGeomFace3D *createFace3D() const;

  virtual CGeomObject3D *createObject3D(CGeomScene3D *pscene, const std::string &name) const;

  virtual CGeomScene3D *createScene3D() const;

  virtual CGeomTexture *createTexture() const;

  virtual CGeomMask *createMask(CImagePtr image) const;

  virtual CGeomCamera3D *createCamera3D(CGeomScene3D *, const std::string &) const;

  virtual CGeomLight3D *createLight3D(CGeomScene3D *, const std::string &) const;

  virtual CGeomMaterial *createMaterial() const;
};

//---

// factory class for all geometry objects so we can derive classes
// from the base geometry classes and use a custom factory to
// create the correct objects
class CGeometry3D {
 public:
  static CGeometry3D *getInstance();

  CGeometry3D();

  void setFactory(CGeometryFactory *factory);

  // factory APIS
  CGeomVertex3D *createVertex3D(CGeomObject3D *pobject, const CPoint3D &point) const;

  CGeomLine3D *createLine3D(CGeomObject3D *pobject, uint v1, uint v2) const;

  CGeomFace3D *createFace3D(CGeomObject3D *pobject, const std::vector<uint> &vertices) const;

  CGeomObject3D *createObject3D(CGeomScene3D *pscene, const std::string &name) const;
  CGeomObject3D *dupObject(CGeomObject3D *obj) const;

  CGeomScene3D *createScene3D() const;

  CGeomTexture *createTexture(const std::string &filename) const;
  CGeomTexture *createTexture(CImagePtr image) const;

  CGeomMask *createMask(CImagePtr image) const;

  CGeomCamera3D *createCamera3D(CGeomScene3D *pscene, const std::string &name) const;

  CGeomLight3D *createLight3D(CGeomScene3D *, const std::string &) const;

  CGeomMaterial *createMaterial() const;

  //---

  int nextObjectId() { return ++objectId_; }

 private:
  CGeometryFactory *factory_   { nullptr };
  mutable int       objectId_  { -1 };
  mutable int       textureId_ { -1 };
};

#endif
