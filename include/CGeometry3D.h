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
  virtual ~CGeometryFactory() { }

  virtual CGeomVertex3D *createVertex3D(CGeomObject3D *pobject, const CPoint3D &point) const {
    return new CGeomVertex3D(pobject, point);
  }

  virtual CGeomLine3D *createLine3D(CGeomObject3D *pobject, uint v1, uint v2) const {
    return new CGeomLine3D(pobject, v1, v2);
  }

  virtual CGeomFace3D *createFace3D(CGeomObject3D *pobject,
                                    const std::vector<uint> &vertices) const {
    return new CGeomFace3D(pobject, vertices);
  }

  virtual CGeomObject3D *createObject3D(CGeomScene3D *pscene, const std::string &name) const {
    return new CGeomObject3D(pscene, name);
  }

  virtual CGeomScene3D *createScene3D() const {
    return new CGeomScene3D();
  }

  virtual CGeomTexture *createTexture(CImagePtr image) const {
    return new CGeomTexture(image);
  }

  virtual CGeomMask *createMask(CImagePtr image) const {
    return new CGeomMask(image);
  }

  virtual CGeomCamera3D *createCamera3D(CGeomScene3D *, const std::string &) const {
    return new CGeomPerspectiveCamera3D();
  }
};

// factor class for all geometry objects so we can derive classes
// from the base geometry classes and use a custom factory to
// create the correct objects
class CGeometry3D {
 public:
  static CGeometry3D *getInstance();

  CGeometry3D() {
    factory_ = new CGeometryFactory;
  }

  void setFactory(CGeometryFactory *factory) {
    delete factory_;

    factory_ = factory;
  }

  CGeomVertex3D *createVertex3D(CGeomObject3D *pobject, const CPoint3D &point) const {
    return factory_->createVertex3D(pobject, point);
  }

  CGeomLine3D *createLine3D(CGeomObject3D *pobject, uint v1, uint v2) const {
    return factory_->createLine3D(pobject, v1, v2);
  }

  CGeomFace3D *createFace3D(CGeomObject3D *pobject, const std::vector<uint> &vertices) const {
    return factory_->createFace3D(pobject, vertices);
  }

  CGeomObject3D *createObject3D(CGeomScene3D *pscene, const std::string &name) const {
    return factory_->createObject3D(pscene, name);
  }

  CGeomScene3D *createScene3D() const {
    return factory_->createScene3D();
  }

  CGeomTexture *createTexture(CImagePtr image) const {
    return factory_->createTexture(image);
  }

  CGeomMask *createMask(CImagePtr image) const {
    return factory_->createMask(image);
  }

  CGeomCamera3D *createCamera3D(CGeomScene3D *pscene, const std::string &name) const {
    return factory_->createCamera3D(pscene, name);
  }

 private:
  CGeometryFactory *factory_ { nullptr };
};

#endif
