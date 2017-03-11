#ifndef CGEOM_LIGHT_3D_H
#define CGEOM_LIGHT_3D_H

#include <CGeomObject3D.h>

class CGeomLight3DData {
 public:
  CGeomLight3DData(const CRGBA &ambient =CRGBA(0,0,0),
                   const CRGBA &diffuse =CRGBA(1,1,1),
                   const CRGBA &specular=CRGBA(0,0,0),
                   const CVector3D &spot_direction=CVector3D(0,0,-1),
                   double spot_exponent=1,
                   double spot_cutoff=180,
                   bool directional=false,
                   double constant_attenuation=1,
                   double linear_attenuation=0,
                   double quadratic_attenuation=0) :
   ambient_(ambient), diffuse_(diffuse), specular_(specular),
   spot_direction_(spot_direction), spot_exponent_(spot_exponent),
   spot_cutoff_(spot_cutoff), directional_(directional),
   constant_attenuation_(constant_attenuation),
   linear_attenuation_(linear_attenuation),
   quadratic_attenuation_(quadratic_attenuation) {
  }

  CGeomLight3DData(const CGeomLight3DData &data) :
   ambient_              (data.ambient_),
   diffuse_              (data.diffuse_),
   specular_             (data.specular_),
   spot_direction_       (data.spot_direction_),
   spot_exponent_        (data.spot_exponent_),
   spot_cutoff_          (data.spot_cutoff_),
   directional_          (data.directional_),
   constant_attenuation_ (data.constant_attenuation_),
   linear_attenuation_   (data.linear_attenuation_),
   quadratic_attenuation_(data.quadratic_attenuation_) {
  }

  const CGeomLight3DData &operator=(const CGeomLight3DData &data) {
    ambient_               = data.ambient_;
    diffuse_               = data.diffuse_;
    specular_              = data.specular_;
    spot_direction_        = data.spot_direction_;
    spot_exponent_         = data.spot_exponent_;
    spot_cutoff_           = data.spot_cutoff_;
    directional_           = data.directional_;
    constant_attenuation_  = data.constant_attenuation_;
    linear_attenuation_    = data.linear_attenuation_;
    quadratic_attenuation_ = data.quadratic_attenuation_;

    return *this;
  }

  const CRGBA &getAmbient () const { return ambient_ ; }
  const CRGBA &getDiffuse () const { return diffuse_ ; }
  const CRGBA &getSpecular() const { return specular_; }

  void setAmbient (const CRGBA &ambient ) { ambient_  = ambient ; }
  void setDiffuse (const CRGBA &diffuse ) { diffuse_  = diffuse ; }
  void setSpecular(const CRGBA &specular) { specular_ = specular; }

  const CVector3D &getSpotDirection() const { return spot_direction_; }
  double           getSpotExponent () const { return spot_exponent_ ; }
  double           getSpotCutOff   () const { return spot_cutoff_   ; }

  void setSpotDirection(const CVector3D &dir) { spot_direction_ = dir; }
  void setSpotExponent (double exponent     ) { spot_exponent_  = exponent; }
  void setSpotCutOff   (double cutoff       ) { spot_cutoff_    = cutoff; }

  bool getDirectional() const { return directional_; }

  void setDirectional(bool flag) { directional_ = flag; }

  double getConstantAttenuation()  const { return constant_attenuation_ ; }
  double getLinearAttenuation()    const { return linear_attenuation_   ; }
  double getQuadraticAttenuation() const { return quadratic_attenuation_; }

  void setAttenuation(double constant_attenuation,
                      double linear_attenuation,
                      double quadratic_attenuation) {
    constant_attenuation_  = constant_attenuation;
    linear_attenuation_    = linear_attenuation;
    quadratic_attenuation_ = quadratic_attenuation;
  }

  void setConstantAttenuation (double attenuation) { constant_attenuation_  = attenuation; }
  void setLinearAttenuation   (double attenuation) { linear_attenuation_    = attenuation; }
  void setQuadraticAttenuation(double attenuation) { quadratic_attenuation_ = attenuation; }

  double getAttenuation(double dist) const {
    if (directional_)
      return 1.0;

    return 1.0/(constant_attenuation_ + dist*(linear_attenuation_ + quadratic_attenuation_*dist));
  }

  double getSpotEffect(const CPoint3D &lpoint, const CPoint3D &point) const {
    if (spot_cutoff_ == 180.0)
      return 1.0;

    CVector3D v = CVector3D(lpoint, point);

    v.normalize();

    double vd = v.dotProduct(spot_direction_);

    if (vd < 0.0)
      vd = 0.0;

    double cosv = cos(M_PI*spot_cutoff_/180.0);

    if (vd < cosv)
      return 0.0;

    return pow(vd, spot_exponent_);
  }

 private:
  CRGBA     ambient_ { 0, 0, 0 };
  CRGBA     diffuse_ { 1, 1, 1 };
  CRGBA     specular_ { 0, 0, 0 };
  CVector3D spot_direction_ { 0, 0, -1 };
  double    spot_exponent_ { 1 };
  double    spot_cutoff_ { 180 };
  bool      directional_ { false };
  double    constant_attenuation_ { 1 };
  double    linear_attenuation_ { 0 };
  double    quadratic_attenuation_ { 0 };
};

//------

class CGeomLight3D;

class CGeomLight3DMgr {
 public:
  typedef std::vector<CGeomLight3D *> LightList;

 public:
  CGeomLight3DMgr();
 ~CGeomLight3DMgr() { }

  const CRGBA &getAmbient() const { return ambient_; }

  void addLight(CGeomLight3D *light);

  void deleteLight(CGeomLight3D *light);

  uint getNumLights() const { return lights_.size(); }

  CGeomLight3D *getLight(uint i) const {
    if (i < getNumLights())
      return lights_[i];

    return nullptr;
  }

  CRGBA lightPoint(const CPoint3D &point, const CVector3D &normal,
                   const CMaterial &material) const;

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
  CGeomLight3DMgr(const CGeomLight3DMgr &mgr);
  CGeomLight3DMgr &operator=(const CGeomLight3DMgr &mgr);

 private:
  CRGBA     ambient_ { 0.1, 0.1, 0.1 };
  LightList lights_;
};

//------

class CGeomLight3D {
 public:
  CGeomLight3D(CGeomScene3D *scene, const std::string &name = "");

  CGeomLight3D(const CGeomLight3D &rhs);

  virtual ~CGeomLight3D() { }

  CGeomLight3D &operator=(const CGeomLight3D &rhs);

  void setMgr(CGeomLight3DMgr *mgr) { mgr_ = mgr; }

  CGeomObject3D *getObject() const { return object_; }

  void setPosition(const CPoint3D &point) {
    object_->setPosition(point);
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

  virtual void setAmbient (const CRGBA &rgba) { data_.setAmbient (rgba); }
  virtual void setDiffuse (const CRGBA &rgba) { data_.setDiffuse (rgba); }
  virtual void setSpecular(const CRGBA &rgba) { data_.setSpecular(rgba); }

  const CRGBA &getAmbient () const { return data_.getAmbient (); }
  const CRGBA &getDiffuse () const { return data_.getDiffuse (); }
  const CRGBA &getSpecular() const { return data_.getSpecular(); }

  double getAttenuation(double dist) const {
    return data_.getAttenuation(dist);
  }

  double getSpotEffect(const CPoint3D &point) const {
    return data_.getSpotEffect(object_->getPositionPoint().getModel(), point);
  }

  virtual void setSpotDirection(const CVector3D &dir) {
    data_.setSpotDirection(dir);
  }

  virtual void setSpotExponent(double exponent) {
    data_.setSpotExponent(exponent);
  }

  virtual void setSpotCutOff(double cutoff) {
    data_.setSpotCutOff(cutoff);
  }

  bool getDirectional() const {
    return data_.getDirectional();
  }

  virtual void setDirectional(bool flag) {
    data_.setDirectional(flag);
  }

  virtual void setConstantAttenuation(double attenuation) {
    data_.setConstantAttenuation(attenuation);
  }

  virtual void setLinearAttenuation(double attenuation) {
    data_.setLinearAttenuation(attenuation);
  }

  virtual void setQuadraticAttenuation(double attenuation) {
    data_.setQuadraticAttenuation(attenuation);
  }

  virtual void setEnabled(bool enabled) { enabled_ = enabled; }

  bool getEnabled() const { return enabled_; }

  void drawImage(CGeomZBuffer *zbuffer);

  void lightPoint(CRGBA &rgba, const CPoint3D &point, const CVector3D &normal,
                  const CMaterial &material) const;

 protected:
  CGeomLight3DMgr  *mgr_ { nullptr };
  CGeomObject3D    *object_ { nullptr };
  CGeomLight3DData  data_;
  bool              enabled_ { true };
};

#endif
