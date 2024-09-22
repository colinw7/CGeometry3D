#include <CGeometry3D.h>

CGeometryFactory::
CGeometryFactory()
{
}

CGeometryFactory::
~CGeometryFactory()
{
}

CGeomVertex3D *
CGeometryFactory::
createVertex3D(CGeomObject3D *pobject, const CPoint3D &point) const
{
  return new CGeomVertex3D(pobject, point);
}

CGeomLine3D *
CGeometryFactory::
createLine3D(CGeomObject3D *pobject, uint v1, uint v2) const
{
  return new CGeomLine3D(pobject, v1, v2);
}

CGeomFace3D *
CGeometryFactory::
createFace3D(CGeomObject3D *pobject, const std::vector<uint> &vertices) const
{
  return new CGeomFace3D(pobject, vertices);
}

CGeomObject3D *
CGeometryFactory::
createObject3D(CGeomScene3D *pscene, const std::string &name) const
{
  return new CGeomObject3D(pscene, name);
}

CGeomScene3D *
CGeometryFactory::
createScene3D() const
{
  return new CGeomScene3D();
}

CGeomTexture *
CGeometryFactory::
createTexture(CImagePtr image) const
{
  return new CGeomTexture(image);
}

CGeomMask *
CGeometryFactory::
createMask(CImagePtr image) const
{
  return new CGeomMask(image);
}

CGeomCamera3D *
CGeometryFactory::
createCamera3D(CGeomScene3D *, const std::string &) const
{
  return new CGeomPerspectiveCamera3D();
}

//---

CGeometry3D *
CGeometry3D::
getInstance()
{
  static CGeometry3D *instance;

  if (! instance)
    instance = new CGeometry3D;

  return instance;
}

CGeometry3D::
CGeometry3D()
{
  factory_ = new CGeometryFactory;
}

void
CGeometry3D::
setFactory(CGeometryFactory *factory)
{
  delete factory_;

  factory_ = factory;
}

CGeomVertex3D *
CGeometry3D::
createVertex3D(CGeomObject3D *pobject, const CPoint3D &point) const
{
  return factory_->createVertex3D(pobject, point);
}

CGeomLine3D *
CGeometry3D::
createLine3D(CGeomObject3D *pobject, uint v1, uint v2) const
{
  return factory_->createLine3D(pobject, v1, v2);
}

CGeomFace3D *
CGeometry3D::
createFace3D(CGeomObject3D *pobject, const std::vector<uint> &vertices) const
{
  return factory_->createFace3D(pobject, vertices);
}

CGeomObject3D *
CGeometry3D::
createObject3D(CGeomScene3D *pscene, const std::string &name) const
{
  return factory_->createObject3D(pscene, name);
}

CGeomScene3D *
CGeometry3D::
createScene3D() const
{
  return factory_->createScene3D();
}

CGeomTexture *
CGeometry3D::
createTexture(const std::string &filename) const
{
  CFile imageFile(filename);

  if (! imageFile.exists())
    return nullptr;

  CImageFileSrc src(imageFile);

  auto image = CImageMgrInst->createImage(src);

  image = image->flippedH();

  return createTexture(image);
}

CGeomTexture *
CGeometry3D::
createTexture(CImagePtr image) const
{
  auto *texture = factory_->createTexture(image);
  texture->setId(++textureId_);
  return texture;
}

CGeomMask *
CGeometry3D::
createMask(CImagePtr image) const
{
  return factory_->createMask(image);
}

CGeomCamera3D *
CGeometry3D::
createCamera3D(CGeomScene3D *pscene, const std::string &name) const
{
  return factory_->createCamera3D(pscene, name);
}
