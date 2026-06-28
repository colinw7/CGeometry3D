#include <GeomObject.h>
#include <Canvas.h>
#include <ShaderProgram.h>
#include <CQGLBuffer.h>

namespace CQTclModel3DView {

GeomObject::
GeomObject(CGeomScene3D *pscene, const std::string &name) :
 CGeomObject3D(pscene, name)
{
}

GeomObject::
GeomObject(const GeomObject &object) :
 CGeomObject3D(object)
{
}

GeomObject::
~GeomObject()
{
}

GeomObject *
GeomObject::
dup() const
{
  return new GeomObject(*this);
}

//---

CQGLBuffer *
GeomObject::
initBuffer(Canvas *canvas)
{
  assert(canvas->sceneShaderProgram());

  if (! buffer_)
    buffer_ = canvas->sceneShaderProgram()->createBuffer();

  buffer_->clearBuffers();

  faceDatas_.clear();

  return buffer_;
}

void
GeomObject::
addFaceData(const FaceData &faceData)
{
  faceDatas_.push_back(faceData);
}

//---

void
GeomObject::
setProperty(const std::string &name, const std::string &value)
{
  propertyMap_[name] = value;
}

bool
GeomObject::
getProperty(const std::string &name, std::string &value)
{
  auto p = propertyMap_.find(name);
  if (p == propertyMap_.end()) return false;

  value = (*p).second;

  return true;
}

}
