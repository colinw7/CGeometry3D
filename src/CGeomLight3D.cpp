#include <CGeomLight3D.h>
#include <CGeometry3D.h>
#include <CGeomZBuffer.h>
#include <CGeomPyramid3D.h>
#include <CImageMgr.h>

#include "images/Sun.h"

CGeomLight3DMgr::
CGeomLight3DMgr()
{
}

void
CGeomLight3DMgr::
addLight(CGeomLight3D *light)
{
  lights_.push_back(light);

  light->setMgr(this);
}

void
CGeomLight3DMgr::
deleteLight(CGeomLight3D *light)
{
  auto plight1 = lights_.begin();
  auto plight2 = lights_.end  ();

  for ( ; plight1 != plight2; ++plight1)
    if (*plight1 == light)
      break;

  if (plight1 != plight2) {
    auto plight0 = plight1;

    ++plight1;

    for ( ; plight1 != plight2; plight0 = plight1++)
      *plight0 = *plight1;

    lights_.pop_back();
  }
}

void
CGeomLight3DMgr::
modelToPixel(const CGeomCamera3D &camera) const
{
  for (auto *light : lights_)
    light->getObject()->modelToPixel(camera);
}

void
CGeomLight3DMgr::
drawWireframe(CGeomCamera3D &, CGeomZBuffer *)
{
//for (auto *light : lights_)
//  light->drawWireframe(camera, zbuffer);
}

void
CGeomLight3DMgr::
drawSolid(CGeomCamera3D &, CGeomZBuffer *)
{
//for (auto *light : lights_)
//  light->drawSolid(camera, zbuffer);
}

CRGBA
CGeomLight3DMgr::
lightPoint(const CPoint3D &point, const CVector3D &normal, const CMaterial &material) const
{
  auto rgba = material.getEmission();

  rgba += getAmbient()*material.getAmbient();

  for (auto *light : lights_)
    light->lightPoint(rgba, point, normal, material);

  rgba.setAlpha(material.getDiffuse().getAlpha());

  return rgba;
}

CImagePtr
CGeomLight3DMgr::
getImage()
{
  static CImagePtr ptr;
  static bool      read;

  if (! read) {
    CImageNameSrc src("CGeomLight3D/Sun");

    ptr = CImageMgrInst->createImage(src);

    ptr->read(Sun_data, SUN_DATA_LEN);

    read = false;
  }

  return ptr;
}

void
CGeomLight3DMgr::
moveX(double dx)
{
  for (auto *light : lights_)
    light->getObject()->moveX(dx);
}

void
CGeomLight3DMgr::
moveY(double dy)
{
  for (auto *light : lights_)
    light->getObject()->moveY(dy);
}

void
CGeomLight3DMgr::
moveZ(double dz)
{
  for (auto *light : lights_)
    light->getObject()->moveZ(dz);
}

void
CGeomLight3DMgr::
rotateX(double dx)
{
  for (auto *light : lights_)
    light->getObject()->rotateX(dx);
}

void
CGeomLight3DMgr::
rotateY(double dy)
{
  for (auto *light : lights_)
    light->getObject()->rotateY(dy);
}

void
CGeomLight3DMgr::
rotateZ(double dz)
{
  for (auto *light : lights_)
    light->getObject()->rotateZ(dz);
}

//----------

CGeomLight3D::
CGeomLight3D(CGeomScene3D *pscene, const std::string &name)
{
  object_ = CGeometryInst->createObject3D(pscene, name);

  CGeomPyramid3D::addGeometry(object_, 0, 0, 0, 0.1, 0.1);

  object_->rotateModelX(-M_PI*0.5);

  object_->unsetFaceFlags(CGeomFace3D::LIGHTED);

  object_->setFaceColor(CRGBA(1, 1, 0));
}

CGeomLight3D::
CGeomLight3D(const CGeomLight3D &light) :
 mgr_    (light.mgr_),
 object_ (light.object_),
 data_   (light.data_),
 enabled_(light.enabled_)
{
}

CGeomLight3D &
CGeomLight3D::
operator=(const CGeomLight3D &light)
{
  mgr_     = light.mgr_;
  object_  = light.object_;
  data_    = light.data_;
  enabled_ = light.enabled_;

  return *this;
}

void
CGeomLight3D::
drawImage(CGeomZBuffer *zbuffer)
{
  if (mgr_) {
    CImagePtr image = mgr_->getImage();

    const CPoint3D &pixel = object_->getPositionPoint().getPixel();

    zbuffer->drawImage(int(pixel.x), int(pixel.y), pixel.z, image);
  }
}

void
CGeomLight3D::
lightPoint(CRGBA &rgba, const CPoint3D &point, const CVector3D &normal,
           const CMaterial &material) const
{
  if (! getEnabled())
    return;

  // Ambient
  auto ambient = getAmbient()*material.getAmbient();

  // Diffuse
  CVector3D dlight(point, object_->getPositionPoint().getViewed());

  dlight.normalize();

  //uncomment if light both sides
  //double dot = fabs(dlight.dotProduct(normal));
  double dot = dlight.dotProduct(normal);

  if (dot < 0.0)
    dot = 0.0;

  auto diffuse = dot*getDiffuse()*material.getDiffuse();

  // Specular
  CRGBA specular(0, 0, 0, 1);

  if (dot > 0.0) {
    CVector3D viewpoint(0, 0, 1);

    CVector3D sum(viewpoint + dlight);

    sum.normalize();

    double dot1 = sum.dotProduct(normal);

    if (dot1 < 0.0)
      dot1 = 0.0;

    specular = std::pow(dot1, material.getShininess())*getSpecular()*material.getSpecular();
  }

  double dist = CVector3D(point, object_->getPositionPoint().getViewed()).length();

  rgba += getAttenuation(dist)*getSpotEffect(point)*(ambient + diffuse + specular);

  //rgba += diffuse;

  rgba.setAlpha(material.getDiffuse().getAlpha());
}
