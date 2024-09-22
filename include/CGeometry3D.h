#ifndef CGEOMETRY_H
#define CGEOMETRY_H

#define CGeometryInst CGeometry3D::getInstance()

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

  virtual CGeomLine3D *createLine3D(CGeomObject3D *pobject, uint v1, uint v2) const;

  virtual CGeomFace3D *createFace3D(CGeomObject3D *pobject,
                                    const std::vector<uint> &vertices) const;

  virtual CGeomObject3D *createObject3D(CGeomScene3D *pscene, const std::string &name) const;

  virtual CGeomScene3D *createScene3D() const;

  virtual CGeomTexture *createTexture(CImagePtr image) const;

  virtual CGeomMask *createMask(CImagePtr image) const;

  virtual CGeomCamera3D *createCamera3D(CGeomScene3D *, const std::string &) const;
};

//---

// factor class for all geometry objects so we can derive classes
// from the base geometry classes and use a custom factory to
// create the correct objects
class CGeometry3D {
 public:
  static CGeometry3D *getInstance();

  CGeometry3D();

  void setFactory(CGeometryFactory *factory);

  CGeomVertex3D *createVertex3D(CGeomObject3D *pobject, const CPoint3D &point) const;

  CGeomLine3D *createLine3D(CGeomObject3D *pobject, uint v1, uint v2) const;

  CGeomFace3D *createFace3D(CGeomObject3D *pobject, const std::vector<uint> &vertices) const;

  CGeomObject3D *createObject3D(CGeomScene3D *pscene, const std::string &name) const;

  CGeomScene3D *createScene3D() const;

  CGeomTexture *createTexture(const std::string &filename) const;
  CGeomTexture *createTexture(CImagePtr image) const;

  CGeomMask *createMask(CImagePtr image) const;

  CGeomCamera3D *createCamera3D(CGeomScene3D *pscene, const std::string &name) const;

 private:
  CGeometryFactory *factory_   { nullptr };
  mutable int       textureId_ { -1 };
};

#endif
