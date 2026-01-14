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

CGeomLight3D *
CGeomLight3DMgr::
getNamedLight(const std::string &name) const
{
  for (auto *light : lights_) {
    if (light->name() == name)
      return light;
  }

  return nullptr;
}

void
CGeomLight3DMgr::
modelToPixel(const CGeomCamera3D &camera) const
{
  for (auto *light : lights_) {
    auto *light1 = dynamic_cast<CGeomObjectLight3D *>(light);

    if (light1)
      light1->getObject()->modelToPixel(camera);
  }
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
lightPoint(const CPoint3D &point, const CVector3D &normal, const CGeomMaterial &material,
           bool bothSides) const
{
  auto rgba = material.getEmission(CRGBA(0.0, 0.0, 0.0, 1.0));

  rgba += getAmbient()*material.getAmbient(CRGBA(1.0, 1.0, 1.0, 1.0));

  for (auto *light : lights_)
    light->lightPoint(rgba, point, normal, material, bothSides);

  rgba.setAlpha(material.getDiffuse(CRGBA(1.0, 1.0, 1.0, 1.0)).getAlpha());

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
  for (auto *light : lights_) {
    auto *light1 = dynamic_cast<CGeomObjectLight3D *>(light);

    if (light1)
      light1->getObject()->moveX(dx);
  }
}

void
CGeomLight3DMgr::
moveY(double dy)
{
  for (auto *light : lights_) {
    auto *light1 = dynamic_cast<CGeomObjectLight3D *>(light);

    if (light1)
      light1->getObject()->moveY(dy);
  }
}

void
CGeomLight3DMgr::
moveZ(double dz)
{
  for (auto *light : lights_) {
    auto *light1 = dynamic_cast<CGeomObjectLight3D *>(light);

    if (light1)
      light1->getObject()->moveZ(dz);
  }
}

void
CGeomLight3DMgr::
rotateX(double dx)
{
  for (auto *light : lights_) {
    auto *light1 = dynamic_cast<CGeomObjectLight3D *>(light);

    if (light1)
      light1->getObject()->rotateX(dx);
  }
}

void
CGeomLight3DMgr::
rotateY(double dy)
{
  for (auto *light : lights_) {
    auto *light1 = dynamic_cast<CGeomObjectLight3D *>(light);

    if (light1)
      light1->getObject()->rotateY(dy);
  }
}

void
CGeomLight3DMgr::
rotateZ(double dz)
{
  for (auto *light : lights_) {
    auto *light1 = dynamic_cast<CGeomObjectLight3D *>(light);

    if (light1)
      light1->getObject()->rotateZ(dz);
  }
}

//----------

CGeomLight3D::
CGeomLight3D(CGeomScene3D *pscene, const std::string &name) :
 pscene_(pscene), name_(name)
{
}

void
CGeomLight3D::
lightPoint(CRGBA &rgba, const CPoint3D &point, const CVector3D &normal,
           const CGeomMaterial &material, bool bothSides) const
{
  if (! getEnabled())
    return;

  // Ambient
  auto ambient = getAmbient()*material.getAmbient();

  // Diffuse (TODO: get position)
#if 0
  auto *object = getObject();

  CVector3D dlight(point, object->getPositionPoint().getViewed());
#else
  CVector3D dlight(point, getPosition());
#endif

  dlight.normalize();

  double dot = dlight.dotProduct(normal);

  if (bothSides)
    dot = fabs(dot);

  if (dot < 0.0)
    dot = 0.0;

  auto diffuse = dot*getDiffuse()*material.getDiffuse();

  // Specular
  CRGBA specular(0.0, 0.0, 0.0, 1.0);

  if (dot > 0.0) {
    CVector3D viewpoint(0, 0, 1);

    CVector3D sum(viewpoint + dlight);

    sum.normalize();

    double dot1 = sum.dotProduct(normal);

    if (dot1 < 0.0)
      dot1 = 0.0;

    specular = std::pow(dot1, material.getShininess(1.0))*getSpecular()*material.getSpecular();
  }

#if 0
  double dist = CVector3D(point, object->getPositionPoint().getViewed()).length();
#else
  double dist = CVector3D(point, getPosition()).length();
#endif

  rgba += calcAttenuation(dist)*getSpotEffect(point)*(ambient + diffuse + specular);

  //rgba += diffuse;

  rgba.setAlpha(material.getDiffuse().getAlpha());
}

//---

CGeomObjectLight3D::
CGeomObjectLight3D(CGeomScene3D *pscene, const std::string &name) :
 CGeomLight3D(pscene, name)
{
  object_ = CGeometry3DInst->createObject3D(pscene_, name_);

  CPoint3D center(0.0, 0.0, 0.0);

  CGeomPyramid3D::addGeometry(object_, center, 0.1, 0.1);

  object_->rotateModelX(-M_PI*0.5);

  object_->unsetFaceFlags(CGeomFace3D::LIGHTED);

  object_->setFaceColor(CRGBA::yellow());
}

void
CGeomObjectLight3D::
drawImage(CGeomZBuffer *zbuffer)
{
  if (! mgr_)
    return;

  auto *object = getObject();

  auto image = mgr_->getImage();

  const CPoint3D &pixel = object->getPositionPoint().getPixel();

  zbuffer->drawImage(int(pixel.x), int(pixel.y), pixel.z, image);
}
