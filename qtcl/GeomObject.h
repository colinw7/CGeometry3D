#ifndef CGeomObject_H
#define CGeomObject_H

#include <FaceData.h>

#include <CGeomObject3D.h>
#include <CBBox3D.h>

class CQGLBuffer;

namespace CQTclModel3DView {

class Canvas;
class Particle;

class GeomObject : public CGeomObject3D {
 public:
  using FaceDatas = std::vector<FaceData>;

 public:
  GeomObject(CGeomScene3D *pscene, const std::string &name);

  GeomObject(const GeomObject &object);

 ~GeomObject();

  GeomObject *dup() const override;

  //---

  const CBBox3D &bbox() const { return bbox_; }
  void setBBox(const CBBox3D &v) { bbox_ = v; }

  //---

  CQGLBuffer *buffer() const { return buffer_; }

  const FaceDatas &faceDatas() const { return faceDatas_; }

  CQGLBuffer *initBuffer(Canvas *canvas);

  void addFaceData(const FaceData &faceData);

  //---

  void setProperty(const std::string &name, const std::string &value);

  bool getProperty(const std::string &name, std::string &value);

  //---

  const Particle *particle() const { return particle_; }
  void setParticle(Particle *p) { particle_ = p; }

  //---

  const std::string &meta() const { return meta_; }
  void setMeta(const std::string &s) { meta_ = s; }

 private:
  using PropertyMap = std::map<std::string, std::string>;

  CBBox3D bbox_;

  CQGLBuffer* buffer_ { nullptr };

  FaceDatas faceDatas_;

  PropertyMap propertyMap_;

  Particle* particle_ { nullptr };

  std::string meta_;
};

}

#endif
