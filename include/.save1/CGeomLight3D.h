#ifndef CGEOM_LIGHT_3D_H
#define CGEOM_LIGHT_3D_H

#include <CGeomObject3D.h>

enum class CGeomLight3DType {
  DIRECTIONAL,
  POINT,
  SPOT
};

class CGeomLight3DData {
 public:
  struct Attenuation {
    double constant  { 1.0 };
    double linear    { 0.0 };
    double quadratic { 0.0 };

    Attenuation() { }
  };

 public:
  CGeomLight3DData() { }

  CGeomLight3DData(const CRGBA &ambient, const CRGBA &diffuse, const CRGBA &specular) :
   ambient_(ambient), diffuse_(diffuse), specular_(specular) {
  }

  //---

  // color

  const CRGBA &getAmbient() const { return ambient_; }
  void setAmbient (const CRGBA &ambient ) { ambient_  = ambient; }

  const CRGBA &getDiffuse() const { return diffuse_; }
  void setDiffuse (const CRGBA &diffuse ) { diffuse_  = diffuse; }

  const CRGBA &getSpecular() const { return specular_; }
  void setSpecular(const CRGBA &specular) { specular_ = specular; }

  //---

  // type
  const CGeomLight3DType &getType() const { return type_; }
  void setType(const CGeomLight3DType &t) { type_ = t; }

  bool getDirectional() const { return type_ == CGeomLight3DType::DIRECTIONAL; }

  //---

  // direction light

  const CVector3D &getDirection() const { return directionData_.direction; }
  void setDirection(const CVector3D &dir) { directionData_.direction = dir; }

  //---

  // point light

  double getPointRadius() const { return pointData_.radius; }
  void setPointRadius(double r) { pointData_.radius = r; }

  //---

  // spot light

  const CVector3D &getSpotDirection() const { return spotData_.direction; }
  void setSpotDirection(const CVector3D &dir) { spotData_.direction = dir; }

  double getSpotExponent() const { return spotData_.exponent; }
  void setSpotExponent(double exponent) { spotData_.exponent = exponent; }

  double getSpotCutOffAngle() const { return spotData_.cutoffAngle; }
  void setSpotCutOffAngle(double a) { spotData_.cutoffAngle = a; }

  //---

  // attenuation

  double getConstantAttenuation() const { return attenuation_.constant; }
  void setConstantAttenuation(double attenuation) { attenuation_.constant = attenuation; }

  double getLinearAttenuation() const { return attenuation_.linear; }
  void setLinearAttenuation(double attenuation) { attenuation_.linear = attenuation; }

  double getQuadraticAttenuation() const { return attenuation_.quadratic; }
  void setQuadraticAttenuation(double attenuation) { attenuation_.quadratic = attenuation; }

  void setAttenuation(const Attenuation &attenuation) { attenuation_ = attenuation; }

  double calcAttenuation(double dist) const {
    if (type_ != CGeomLight3DType::POINT)
      return 1.0;

    return 1.0/(attenuation_.constant + dist*(attenuation_.linear + dist*attenuation_.quadratic));
  }

  //---

  // calc
  double getSpotEffect(const CPoint3D &lpoint, const CPoint3D &point) const {
    if (getSpotCutOffAngle() == 180.0)
      return 1.0;

    auto v = CVector3D(lpoint, point);

    v.normalize();

    double vd = v.dotProduct(getSpotDirection());

    if (vd < 0.0)
      vd = 0.0;

    auto cosv = std::cos(M_PI*getSpotCutOffAngle()/180.0);

    if (vd < cosv)
      return 0.0;

    return pow(vd, getSpotExponent());
  }

 private:
  // colors
  CRGBA ambient_  { 0.0, 0.0, 0.0 };
  CRGBA diffuse_  { 1.0, 1.0, 1.0 };
  CRGBA specular_ { 0.0, 0.0, 0.0 };

  CGeomLight3DType type_ { CGeomLight3DType::POINT };

  struct DirectionData {
    CVector3D direction { 0.0, 0.0, -1.0 };
  };

  struct PointData {
    double radius { 100.0 };
  };

  struct SpotData {
    CVector3D direction   { 0.0, 0.0, -1.0 };
    double    exponent    { 1.0 };
    double    cutoffAngle { 180.0 };
  };

  DirectionData directionData_;
  PointData     pointData_;
  SpotData      spotData_;
  Attenuation   attenuation_;
};

//------

class CGeomLight3D;

class CGeomLight3DMgr {
 public:
  using LightList = std::vector<CGeomLight3D *>;

 public:
  CGeomLight3DMgr();
 ~CGeomLight3DMgr() { }

  CGeomLight3DMgr(const CGeomLight3DMgr &mgr) = delete;
  CGeomLight3DMgr &operator=(const CGeomLight3DMgr &mgr) = delete;

  //---

  const CRGBA &getAmbient() const { return ambient_; }

  //---

  const LightList &lights() const { return lights_; }

  void addLight(CGeomLight3D *light);

  void deleteLight(CGeomLight3D *light);

  uint getNumLights() const { return uint(lights_.size()); }

  CGeomLight3D *getNamedLight(const std::string &name) const;

  CGeomLight3D *getLight(uint i) const {
    if (i < getNumLights())
      return lights_[i];

    return nullptr;
  }

  //---

  CRGBA lightPoint(const CPoint3D &point, const CVector3D &normal,
                   const CGeomMaterial &material, bool bothSides=false) const;

  void modelToPixel(const CGeomCamera3D &camera) const;

  void drawWireframe(CGeomCamera3D &camera, CGeomZBuffer *zbuffer);

  void drawSolid(CGeomCamera3D &camera, CGeomZBuffer *zbuffer);

  CImagePtr getImage();

  void moveX(double dx);
  void moveY(double dy);
  void moveZ(double dz);

  void rotateX(double dx);
  void rotateY(double dy);
  void rotateZ(double dz);

 private:
  CRGBA     ambient_ { 0.1, 0.1, 0.1 };
  LightList lights_;
};

//------

// light class
//   directional
//     direction
//   point
//     radius
//   spot
//     direction
//     cutoff angle

class CGeomLight3D {
 public:
  CGeomLight3D(CGeomScene3D *scene, const std::string &name="");

  virtual ~CGeomLight3D() { }

  //---

  void setMgr(CGeomLight3DMgr *mgr) { mgr_ = mgr; }

  //---

  const uint &id() const { return id_; }
  void setId(const uint &v) { id_ = v; }

  const std::string &name() const { return name_; }
  void setName(const std::string &s) { name_ = s; }

  //---

  // position

  virtual void setPosition(const CPoint3D &point) {
    position_ = point;
  }

  virtual const CPoint3D &getPosition() const {
    return position_;
  }

  //---

  // colors

  const CRGBA &getAmbient() const { return data_.getAmbient (); }
  virtual void setAmbient(const CRGBA &rgba) { data_.setAmbient (rgba); }

  const CRGBA &getDiffuse() const { return data_.getDiffuse (); }
  virtual void setDiffuse(const CRGBA &rgba) { data_.setDiffuse (rgba); }

  const CRGBA &getSpecular() const { return data_.getSpecular(); }
  virtual void setSpecular(const CRGBA &rgba) { data_.setSpecular(rgba); }

  //---

  // type

  bool getDirectional() const {
    return data_.getDirectional();
  }

  const CGeomLight3DType &getType() const { return data_.getType(); }
  virtual void setType(const CGeomLight3DType &t) { data_.setType(t); }

  //---

  // directional light
  const CVector3D &getDirection() const { return data_.getDirection(); }
  void setDirection(const CVector3D &dir) { data_.setDirection(dir); }

  //---

  // point light

  double getPointRadius() const { return data_.getPointRadius(); }
  void setPointRadius(double r) { data_.setPointRadius(r); }

  //---

  // spot light

  const CVector3D &getSpotDirection() const { return data_.getSpotDirection(); }
  virtual void setSpotDirection(const CVector3D &dir) { data_.setSpotDirection(dir); }

  virtual double getSpotEffect(const CPoint3D &point) const {
    return data_.getSpotEffect(getPosition(), point);
  }

  double getSpotExponent() const { return data_.getSpotExponent(); }
  virtual void setSpotExponent(double exponent) { data_.setSpotExponent(exponent); }

  double getSpotCutOffAngle() const { return data_.getSpotCutOffAngle(); }
  virtual void setSpotCutOffAngle(double a) { data_.setSpotCutOffAngle(a); }

  //---

  // attenuation

  double getConstantAttenuation() const { return data_.getConstantAttenuation(); }

  virtual void setConstantAttenuation(double attenuation) {
    data_.setConstantAttenuation(attenuation);
  }

  double getLinearAttenuation() const { return data_.getLinearAttenuation(); }

  virtual void setLinearAttenuation(double attenuation) {
    data_.setLinearAttenuation(attenuation);
  }

  double getQuadraticAttenuation() const { return data_.getQuadraticAttenuation(); }

  virtual void setQuadraticAttenuation(double attenuation) {
    data_.setQuadraticAttenuation(attenuation);
  }

  virtual double calcAttenuation(double dist) const {
    return data_.calcAttenuation(dist);
  }

  //---

  // enabled
  bool getEnabled() const { return enabled_; }
  virtual void setEnabled(bool enabled) { enabled_ = enabled; }

  //---

  // light
  void lightPoint(CRGBA &rgba, const CPoint3D &point, const CVector3D &normal,
                  const CGeomMaterial &material, bool bothSides=false) const;

 protected:
  CGeomLight3DMgr* mgr_     { nullptr };
  CGeomScene3D*    pscene_  { nullptr };
  uint             id_      { 0 };
  std::string      name_;
  bool             enabled_ { true };
  CGeomLight3DData data_;
  CPoint3D         position_;
};

//------

class CGeomObjectLight3D : public CGeomLight3D {
 public:
  CGeomObjectLight3D(CGeomScene3D *scene, const std::string &name="");

  CGeomObject3D *getObject() const { return object_; }

  void setPosition(const CPoint3D &point) override {
    CGeomLight3D::setPosition(point);

    object_->setPosition(point);
  }

  const CPoint3D &getPosition() const override {
    return object_->getPositionPoint().getModel();
  }

  const CGeomPoint3D &getPositionPoint() const {
    return object_->getPositionPoint();
  }

  CGeomPoint3D &editPositionPoint() const {
    return object_->editPositionPoint();
  }

  void setPositionPoint(const CGeomPoint3D &point) {
    object_->setPositionPoint(point);
  }

  void setAbsPosition(const CPoint3D &point) {
    object_->setAbsPosition(point);
  }

  // draw
  void drawImage(CGeomZBuffer *zbuffer);

 protected:
  CGeomObject3D* object_ { nullptr };
};

#endif
