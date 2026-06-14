#ifndef ParticleSystem_H
#define ParticleSystem_H

#include <CPSysSystem.h>
#include <CPSysParticle.h>
#include <CRGBA.h>
#include <CPoint2D.h>
#include <CSize2D.h>

namespace CQTclModel3DView {

class Particle;
class Texture;

class ParticleSystem : public CPSysSystem {
 public:
  ParticleSystem();

  CPSysParticle *makeParticle(double mass=1.0, double x=0.0, double y=0.0, double z=0.0) override;
};

class Particle : public CPSysParticle {
 public:
  Particle(double mass=1.0);

  const CRGBA &color() const { return color_; }
  void setColor(const CRGBA &v) { color_ = v; }

  Texture *texture() const { return texture_; }
  void setTexture(Texture *t) { texture_ = t; }

  double size() const { return size_; }
  void setSize(double r) { size_ = r; }

  double alpha() const { return alpha_; }
  void setAlpha(double r) { alpha_ = r; }

  double angle() const { return angle_; }
  void setAngle(double r) { angle_ = r; }

  const CPoint2D &tpos() const { return tpos_; }
  void setTPos(const CPoint2D &v) { tpos_ = v; }

  const CSize2D &tsize() const { return tsize_; }
  void setTSize(const CSize2D &v) { tsize_ = v; }

  const std::string &meta() const { return meta_; }
  void setMeta(const std::string &s) { meta_ = s; }

 private:
  CRGBA       color_   { CRGBA::white() };
  Texture*    texture_ { nullptr };
  double      size_    { 1.0 };
  double      alpha_   { 1.0 };
  double      angle_   { 0.0 };
  CPoint2D    tpos_    { 0, 0 };
  CSize2D     tsize_   { 1, 1 };
  std::string meta_;
};

}

#endif
