#ifndef CMATERIAL_H
#define CMATERIAL_H

#include <CRGBA.h>
#include <optional>

class CMaterial {
 public:
  using OptColor = std::optional<CRGBA>;
  using OptReal  = std::optional<double>;

 public:
  CMaterial() { }

  CMaterial(const CRGBA &ambient, const CRGBA &diffuse, const CRGBA &specular=CRGBA(0, 0, 0, 1),
            const CRGBA &emission=CRGBA(0, 0, 0, 1), double shininess=1.0, bool mirror=false) :
   ambient_(ambient), diffuse_(diffuse), specular_(specular), emission_(emission),
   shininess_(shininess), mirror_(mirror) {
  }

  explicit
  CMaterial(const CRGBA &color, const CRGBA &specular=CRGBA(0, 0, 0, 1),
            const CRGBA &emission=CRGBA(0, 0, 0, 1), double shininess=1.0, bool mirror=false) :
   ambient_(color), diffuse_(color), specular_(specular), emission_(emission),
   shininess_(shininess), mirror_(mirror) {
  }

  //---

  void setColor(const CRGBA &c) {
    setAmbient(c);
    setDiffuse(c);
  }

  void setColor(const CRGBA &c, const CRGBA &specular, const CRGBA &emission) {
    setAmbient(c);
    setDiffuse(c);

    setSpecular(specular);
    setEmission(emission);
  }

  const OptColor &ambient() const { return ambient_; }
  void setAmbient(const CRGBA &c) { ambient_ = c; }
  CRGBA getAmbient(const CRGBA &c=CRGBA(1.0, 1.0, 1.0, 1.0)) const {
    return ambient().value_or(c); }

  const OptColor &diffuse() const { return diffuse_; }
  void setDiffuse(const CRGBA &c) { diffuse_ = c; }
  CRGBA getDiffuse(const CRGBA &c=CRGBA(1.0, 1.0, 1.0, 1.0)) const {
    return diffuse().value_or(c); }

  const OptColor &specular() const { return specular_; }
  void setSpecular(const CRGBA &c) { specular_ = c; }
  CRGBA getSpecular(const CRGBA &c=CRGBA(0.0, 0.0, 0.0, 1.0)) const {
    return specular().value_or(c); }

  const OptColor &emission() const { return emission_; }
  void setEmission(const CRGBA &c) { emission_ = c; }
  CRGBA getEmission(const CRGBA &c=CRGBA(0.0, 0.0, 0.0, 1.0)) const {
    return emission().value_or(c); }

  const OptReal &shininess() const { return shininess_; }
  void setShininess(double shininess) { shininess_ = shininess; }
  double getShininess(double r=1.0) const { return shininess().value_or(r); }

  const OptColor &color() const { return diffuse(); }
  CRGBA getColor(const CRGBA &c=CRGBA(1.0, 1.0, 1.0, 1.0)) const { return getDiffuse(c); }

  void setMirror(bool mirror) { mirror_ = mirror; }
  bool isMirror() const { return mirror_; }

  //---

  bool equal(const CRGBA &ambient, const CRGBA &diffuse=CRGBA(1, 1, 1),
             const CRGBA &specular=CRGBA(0, 0, 0), const CRGBA &emission=CRGBA(0, 0, 0),
             double shininess=1.0, bool mirror=false) const {
   return (ambient_   == ambient   && diffuse_  == diffuse &&
           specular_  == specular  && emission_ == emission &&
           shininess_ == shininess && mirror_   == mirror);
  }

  bool equal(const CMaterial &material) const {
    return (ambient_   == material.ambient_   &&
            diffuse_   == material.diffuse_   &&
            specular_  == material.specular_  &&
            emission_  == material.emission_  &&
            shininess_ == material.shininess_ &&
            mirror_    == material.mirror_);
  }

 private:
  OptColor ambient_;
  OptColor diffuse_;
  OptColor specular_;
  OptColor emission_;
  OptReal  shininess_;
  bool     mirror_ { false };
};

#endif
