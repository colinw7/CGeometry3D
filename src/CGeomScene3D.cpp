#include <CGeomScene3D.h>
#include <CGeometry3D.h>
#include <CGeomZBuffer.h>
#include <CTransform2D.h>

CGeomScene3D::
CGeomScene3D()
{
  camera_ = CameraP(CGeometryInst->createCamera3D(this, "camera"));

  axes_ = std::make_unique<CGeomAxes3D>(this, "axis");
}

void
CGeomScene3D::
setRenderer(CGeom3DRenderer *renderer)
{
  if (renderer != renderer_) {
    renderer_ = renderer;
    zbuffer_  = std::make_unique<CGeomZBuffer>(renderer_);
  }
}

void
CGeomScene3D::
addPrimitive(CGeomObject3D *object)
{
  primitives_.push_back(object);

  primitive_map_[object->getName()] = object;
}

CGeomObject3D &
CGeomScene3D::
getPrimitive(const std::string &name) const
{
  return *getPrimitiveP(name);
}

CGeomObject3D *
CGeomScene3D::
getPrimitiveP(const std::string &name) const
{
  auto p = primitive_map_.find(name);

  if (p != primitive_map_.end())
    return (*p).second;

  return nullptr;
}

void
CGeomScene3D::
addObject(CGeomObject3D *object)
{
  objects_.push_back(object);

  object_map_[object->getName()] = object;
}

void
CGeomScene3D::
removeObject(CGeomObject3D *object, bool force)
{
  object_map_[object->getName()] = nullptr;

  if (force) {
    auto objects = objects_;
    objects_.clear();

    for (auto *object1 : objects)
      if (object1 != object)
        objects_.push_back(object1);
  }
}

CGeomObject3D &
CGeomScene3D::
getObject(const std::string &name) const
{
  return *getObjectP(name);
}

CGeomObject3D *
CGeomScene3D::
getObjectP(const std::string &name) const
{
  auto p = object_map_.find(name);

  if (p != object_map_.end())
    return (*p).second;

  return nullptr;
}

uint
CGeomScene3D::
getNumSelectedObjects() const
{
  auto objects = getSelectedObjects();

  return uint(objects.size());
}

CGeomScene3D::ObjectList
CGeomScene3D::
getSelectedObjects() const
{
  ObjectList objects;

  for (auto *object : objects) {
    if (object->getSelected())
      objects.push_back(object);
  }

  return objects;
}

void
CGeomScene3D::
addLight(CGeomLight3D *light)
{
  light_mgr_.addLight(light);
}

uint
CGeomScene3D::
getNumLights() const
{
  return light_mgr_.getNumLights();
}

CGeomLight3D *
CGeomScene3D::
getLight(uint i)
{
  return light_mgr_.getLight(i);
}

void
CGeomScene3D::
getBBox(CBBox3D &bbox) const
{
  const auto &objects = getObjects();

  for (auto *object : objects) {
    if (! object->getVisible()) continue;

    CBBox3D bbox1;
    object->getModelBBox(bbox1);

    bbox += bbox1;
  }
}

CPoint3D
CGeomScene3D::
getCenter() const
{
  CBBox3D bbox;

  getBBox(bbox);

  return bbox.getCenter();
}

void
CGeomScene3D::
setCenter(const CPoint3D &point)
{
  CPoint3D c = getCenter();

  CVector3D d(c, point);

  objectsMove(d.point());
}

CVector3D
CGeomScene3D::
getSize() const
{
  CBBox3D bbox;

  getBBox(bbox);

  return bbox.getSize();
}

void
CGeomScene3D::
initCamera()
{
  getBBox(bbox_);

  CVector3D size(1,1,1);
  CPoint3D  center(0,0,0);

  if (bbox_.isSet()) {
    center = bbox_.getCenter();
    size   = bbox_.getSize  ();
  }

  double len = size.length();

  if (len <= 0)
    len = 1;

  CPoint3D  pos(center.x, center.y, center.z + 10);
  CVector3D dir(0, 0, -1);

  camera_->setPosition (pos);
  camera_->setDirection(dir);
}

void
CGeomScene3D::
cameraMoveX(double dx)
{
  camera_->moveX(dx);
}

void
CGeomScene3D::
cameraMoveY(double dy)
{
  camera_->moveY(dy);
}

void
CGeomScene3D::
cameraMoveZ(double dz)
{
  camera_->moveZ(dz);
}

void
CGeomScene3D::
cameraRotateX(double dx)
{
  camera_->rotateX(dx);
}

void
CGeomScene3D::
cameraRotateY(double dy)
{
  camera_->rotateY(dy);
}

void
CGeomScene3D::
cameraRotateZ(double dz)
{
  camera_->rotateZ(dz);
}

void
CGeomScene3D::
lightsMoveZ(double dz)
{
  light_mgr_.moveZ(dz);
}

void
CGeomScene3D::
lightsMoveY(double dy)
{
  light_mgr_.moveY(dy);
}

void
CGeomScene3D::
lightsMoveX(double dx)
{
  light_mgr_.moveX(dx);
}

void
CGeomScene3D::
lightsRotateZ(double dz)
{
  light_mgr_.rotateZ(dz);
}

void
CGeomScene3D::
lightsRotateY(double dy)
{
  light_mgr_.rotateY(dy);
}

void
CGeomScene3D::
lightsRotateX(double dx)
{
  light_mgr_.rotateX(dx);
}

void
CGeomScene3D::
objectsMove(const CPoint3D &delta)
{
  const auto &objects = getObjects();

  for (auto *object : objects)
    object->moveBy(delta);
}

void
CGeomScene3D::
objectsMoveX(double dx)
{
  const auto &objects = getObjects();

  for (auto *object : objects)
    object->moveX(dx);
}

void
CGeomScene3D::
objectsMoveY(double dy)
{
  const auto &objects = getObjects();

  for (auto *object : objects)
    object->moveY(dy);
}

void
CGeomScene3D::
objectsMoveZ(double dz)
{
  const auto &objects = getObjects();

  for (auto *object : objects)
    object->moveZ(dz);
}

void
CGeomScene3D::
objectsRotate(const CPoint3D &angle)
{
  const ObjectList &objects = getObjects();

  ObjectList::const_iterator p1 = objects.begin();
  ObjectList::const_iterator p2 = objects.end  ();

  for ( ; p1 != p2; ++p1)
    (*p1)->rotate(angle);
}

void
CGeomScene3D::
objectsRotateX(double dx)
{
  const ObjectList &objects = getObjects();

  ObjectList::const_iterator p1 = objects.begin();
  ObjectList::const_iterator p2 = objects.end  ();

  for ( ; p1 != p2; ++p1)
    (*p1)->rotateX(dx);
}

void
CGeomScene3D::
objectsRotateY(double dy)
{
  const ObjectList &objects = getObjects();

  ObjectList::const_iterator p1 = objects.begin();
  ObjectList::const_iterator p2 = objects.end  ();

  for ( ; p1 != p2; ++p1)
    (*p1)->rotateY(dy);
}

void
CGeomScene3D::
objectsRotateZ(double dz)
{
  const ObjectList &objects = getObjects();

  ObjectList::const_iterator p1 = objects.begin();
  ObjectList::const_iterator p2 = objects.end  ();

  for ( ; p1 != p2; ++p1)
    (*p1)->rotateZ(dz);
}

void
CGeomScene3D::
objectsResizeZ(double dz)
{
  const ObjectList &objects = getObjects();

  ObjectList::const_iterator p1 = objects.begin();
  ObjectList::const_iterator p2 = objects.end  ();

  for ( ; p1 != p2; ++p1)
    (*p1)->resizeModelZ(dz);
}

void
CGeomScene3D::
objectsResizeY(double dy)
{
  const ObjectList &objects = getObjects();

  ObjectList::const_iterator p1 = objects.begin();
  ObjectList::const_iterator p2 = objects.end  ();

  for ( ; p1 != p2; ++p1)
    (*p1)->resizeModelY(dy);
}

void
CGeomScene3D::
objectsResizeX(double dx)
{
  const ObjectList &objects = getObjects();

  ObjectList::const_iterator p1 = objects.begin();
  ObjectList::const_iterator p2 = objects.end  ();

  for ( ; p1 != p2; ++p1)
    (*p1)->resizeModelX(dx);
}

void
CGeomScene3D::
drawInit()
{
  if (isUseZBuffer())
    getZBuffer()->updateSize();

  getCamera()->createProjectionMatrix(-1, 1, -1, 1);

  if (isUseZBuffer())
    getCamera()->createWorldMatrix(int(getZBuffer()->getWidth()), int(getZBuffer()->getHeight()));
  else
    getCamera()->createWorldMatrix(int(renderer_->getWidth()), int(renderer_->getHeight()));

  if (isUseZBuffer())
    getZBuffer()->setForeground(CRGBA(0,0,0));
  else
    renderer_->setForeground(CRGBA(0,0,0));

  modelToPixel();

  drawExec();
}

void
CGeomScene3D::
drawExec()
{
  if (isUseZBuffer()) {
    getZBuffer()->clear();

    if (getDrawType() == WIRE_FRAME)
      drawWireframeZ();
    else
      drawSolidZ();

    // getZBuffer()->setForeground(CRGBA(0,1,0));

    // axes_->drawSubLines(getZBuffer());
  }
  else {
    if (getDrawType() == WIRE_FRAME)
      drawWireframe();
    else
      drawSolid();
  }
}

void
CGeomScene3D::
drawTerm()
{
  if (isUseZBuffer())
    getZBuffer()->draw();
}

void
CGeomScene3D::
drawWireframe()
{
  const ObjectList &objects = getObjects();

  ObjectList::const_iterator p1 = objects.begin();
  ObjectList::const_iterator p2 = objects.end  ();

  for ( ; p1 != p2; ++p1) {
    if (! (*p1)->getVisible()) continue;

    (*p1)->drawWireframe(*getCamera(), renderer_);
  }
}

void
CGeomScene3D::
drawWireframeZ()
{
  const ObjectList &objects = getObjects();

  ObjectList::const_iterator p1 = objects.begin();
  ObjectList::const_iterator p2 = objects.end  ();

  for ( ; p1 != p2; ++p1) {
    if (! (*p1)->getVisible()) continue;

    (*p1)->drawWireframe(*getCamera(), getZBuffer());
  }

  //light_mgr_.drawWireframe(*getCamera(), getZBuffer());
}

void
CGeomScene3D::
drawSolid()
{
  const ObjectList &objects = getObjects();

  ObjectList::const_iterator p1 = objects.begin();
  ObjectList::const_iterator p2 = objects.end  ();

  for ( ; p1 != p2; ++p1) {
    if (! (*p1)->getVisible()) continue;

    (*p1)->drawSolid(*getCamera(), renderer_);
  }

  //light_mgr_.drawSolid(*getCamera(), renderer_);
}

void
CGeomScene3D::
drawSolidZ()
{
  const ObjectList &objects = getObjects();

  ObjectList::const_iterator p1 = objects.begin();
  ObjectList::const_iterator p2 = objects.end  ();

  for ( ; p1 != p2; ++p1) {
    if (! (*p1)->getVisible()) continue;

    (*p1)->drawSolid(*getCamera(), getZBuffer());
  }

  //light_mgr_.drawSolid(*getCamera(), getZBuffer());
}

void
CGeomScene3D::
modelToPixel()
{
  const ObjectList &objects = getObjects();

  ObjectList::const_iterator p1 = objects.begin();
  ObjectList::const_iterator p2 = objects.end  ();

  for ( ; p1 != p2; ++p1) {
    (*p1)->modelToPixel(*getCamera());
  }

  light_mgr_.modelToPixel(*getCamera());

  axes_->modelToPixel(*getCamera());
}

void
CGeomScene3D::
objectsScale(double factor)
{
  ObjectList::iterator p1 = objects_.begin();
  ObjectList::iterator p2 = objects_.end  ();

  for ( ; p1 != p2; ++p1)
    (*p1)->resizeModel(factor);
}

CGeomFace3D *
CGeomScene3D::
getFaceAt(int x, int y)
{
  drawExec();

  return getZBuffer()->getFace(x, y);
}

double
CGeomScene3D::
getZAt(int x, int y)
{
  drawExec();

  return getZBuffer()->getZ(x, y);
}

bool
CGeomScene3D::
lightPoint(const CPoint3D &point, const CVector3D &normal,
           const CMaterial &material, CRGBA &rgba) const
{
  if (getNumLights() == 0)
    return false;

  rgba = light_mgr_.lightPoint(point, normal, material);

  rgba.clamp();

  return true;
}
