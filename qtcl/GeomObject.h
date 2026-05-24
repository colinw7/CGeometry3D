#ifndef CGeomObject_H
#define CGeomObject_H

#include <FaceData.h>
#include <ShaderProgram.h>

#include <CQGLBuffer.h>
#include <CGeomObject3D.h>
#include <CBBox3D.h>

class CQGLBuffer;

namespace CQTclModel3DView {

class Canvas;

class GeomObject : public CGeomObject3D {
 public:
  using FaceDatas = std::vector<FaceData>;

 public:
  GeomObject(CGeomScene3D *pscene, const std::string &name) :
   CGeomObject3D(pscene, name) {
  }

  GeomObject(const GeomObject &object) :
   CGeomObject3D(object) {
  }

  GeomObject *dup() const override {
    return new GeomObject(*this);
  }

  //---

  const CBBox3D &bbox() const { return bbox_; }
  void setBBox(const CBBox3D &v) { bbox_ = v; }

  //---

  CQGLBuffer *buffer() const { return buffer_; }

  const FaceDatas &faceDatas() const { return faceDatas_; }

  CQGLBuffer *initBuffer(Canvas *canvas) {
    assert(canvas->sceneShaderProgram());

    if (! buffer_)
      buffer_ = canvas->sceneShaderProgram()->createBuffer();

    buffer_->clearBuffers();

    faceDatas_.clear();

    return buffer_;
  }

  void addFaceData(const FaceData &faceData) {
    faceDatas_.push_back(faceData);
  }

  //---

  void setProperty(const std::string &name, const std::string &value) {
    propertyMap_[name] = value;
  }

  bool getProperty(const std::string &name, std::string &value) {
    auto p = propertyMap_.find(name);
    if (p == propertyMap_.end()) return false;

    value = (*p).second;

    return true;
  }

 private:
  using PropertyMap = std::map<std::string, std::string>;

  CBBox3D bbox_;

  CQGLBuffer* buffer_ { nullptr };

  FaceDatas faceDatas_;

  PropertyMap propertyMap_;
};

}

#endif
