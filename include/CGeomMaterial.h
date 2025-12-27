#ifndef CGeomMaterial_H
#define CGeomMaterial_H

#include <CMaterial.h>
#include <map>
#include <vector>

class CGeomTexture;

class CGeomMaterial {
 public:
  using OptColor = CMaterial::OptColor;
  using OptReal  = CMaterial::OptReal;

 public:
  enum class Shading {
    WIRE,
    FLAT,
    GOURAUD,
    PHONG,
    METAL,
    BLINN
  };

  CGeomMaterial();

  explicit CGeomMaterial(const CMaterial &material);

  const uint &id() const { return id_; }
  void setId(const uint &v) { id_ = v; }

  const std::string &name() const { return name_; }
  void setName(const std::string &s) { name_ = s; }

  //----

  // CMaterial accessors

  void setColor(const CRGBA &c) { material_.setColor(c); } // same as diffuse

  const OptColor &ambient() const { return material_.ambient(); }
  void setAmbient(const CRGBA &c) { material_.setAmbient(c); }
  CRGBA getAmbient(const CRGBA &c=CRGBA(0.1, 0.1, 0.1, 1.0)) const {
    return material_.getAmbient(c); }

  const OptColor &diffuse() const { return material_.diffuse(); }
  void setDiffuse(const CRGBA &c) { material_.setDiffuse(c); }
  CRGBA getDiffuse(const CRGBA &c=CRGBA(0.9, 0.9, 0.9, 1.0)) const {
    return material_.getDiffuse(c); }

  const OptColor &specular() const { return material_.specular(); }
  void setSpecular(const CRGBA &c) { material_.setSpecular(c); }
  CRGBA getSpecular(const CRGBA &c=CRGBA(0.0, 0.0, 0.0, 1.0)) const {
    return material_.getSpecular(c); }

  const OptColor &emission() const { return material_.emission(); }
  void setEmission(const CRGBA &c) { material_.setEmission(c); }
  CRGBA getEmission(const CRGBA &c=CRGBA(0.0, 0.0, 0.0, 1.1)) const {
    return material_.getEmission(c); }

  const OptReal &shininess() const { return material_.shininess(); }
  void setShininess(double shininess) { material_.setShininess(shininess); }
  double getShininess(double r=1.0) const { return material_.getShininess(r); }

  const CMaterial &material() const { return material_; }

  //---

  // 0.0=opaque, 1.0=transparent
  double transparency() const { return transparency_; }
  void setTransparency(double r) { transparency_ = r; }

  //---

  bool isTwoSided() const { return twoSided_; }
  void setTwoSided(bool b) { twoSided_ = b; }

  //---

  const Shading &shading() const { return shading_; }
  void setShading(const Shading &v) { shading_ = v; }

  static const char *shadingName(const Shading &shading);

  //---

  CGeomTexture *ambientTexture() const { return ambientTexture_; }
  void setAmbientTexture(CGeomTexture *p) { ambientTexture_ = p; }

  CGeomTexture *diffuseTexture() const { return diffuseTexture_; }
  void setDiffuseTexture(CGeomTexture *p) { diffuseTexture_ = p; }

  CGeomTexture *normalTexture() const { return normalTexture_; }
  void setNormalTexture(CGeomTexture *p) { normalTexture_ = p; }

  CGeomTexture *specularTexture() const { return specularTexture_; }
  void setSpecularTexture(CGeomTexture *p) { specularTexture_ = p; }

  CGeomTexture *emissiveTexture() const { return emissiveTexture_; }
  void setEmissiveTexture(CGeomTexture *p) { emissiveTexture_ = p; }

 private:
  uint id_ { 0 };

  std::string name_;
  CMaterial   material_;

  double  transparency_ { 0.0 };
  bool    twoSided_     { false };
  Shading shading_      { Shading::FLAT };

  CGeomTexture* ambientTexture_  { nullptr };
  CGeomTexture* diffuseTexture_  { nullptr };
  CGeomTexture* normalTexture_   { nullptr };
  CGeomTexture* specularTexture_ { nullptr };
  CGeomTexture* emissiveTexture_ { nullptr };
};

//---

class CGeomMaterialMgr {
 public:
  CGeomMaterialMgr();

  void addMaterial(CGeomMaterial *material);

  CGeomMaterial *getMaterial(const std::string &name) const;

  std::vector<CGeomMaterial *> getMaterials() const;

  CGeomMaterial *getMaterialById(uint id) const;

 private:
  using MaterialMap = std::map<std::string, CGeomMaterial *>;

  MaterialMap materialMap_;
  uint        ind_ { 0 };
};

#endif
