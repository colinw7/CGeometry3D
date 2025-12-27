#ifndef CGeomAnimationData_H
#define CGeomAnimationData_H

#include <CTranslate3D.h>
#include <CRotate3D.h>
#include <CScale3D.h>

#include <optional>
#include <vector>

// set of transformations and associated range to interplate into
class CGeomAnimationData {
 public:
  enum class Interpolation {
    NONE,
    LINEAR,
    STEP,
    CUBICSPLINE
  };

  using Reals   = std::vector<double>;
  using OptReal = std::optional<double>;

  using Rotations    = std::vector<CQuaternion>;
  using Translations = std::vector<CVector3D>;
  using Scales       = std::vector<CVector3D>;

 public:
  const Reals &range() const { return range_; }
  void setRange(const Reals &v) { range_ = v; }
  void addRangeValue(double r) { range_.push_back(r); }

  const OptReal &rangeMin() const { return rangeMin_; }
  void setRangeMin(const OptReal &v) { rangeMin_ = v; }

  const OptReal &rangeMax() const { return rangeMax_; }
  void setRangeMax(const OptReal &v) { rangeMax_ = v; }

  const Rotations &rotations() const { return rotations_; }
  void setRotations(const Rotations &v) { rotations_ = v; }
  void addRotation(const CQuaternion &q) { rotations_.push_back(q); }

  const Translations &translations() const { return translations_; }
  void setTranslations(const Translations &v) { translations_ = v; }
  void addTranslation(const CVector3D &v) { translations_.push_back(v); }

  const Scales &scales() const { return scales_; }
  void setScales(const Scales &v) { scales_ = v; }
  void addScale(const CVector3D &v) { scales_.push_back(v); }

  const Interpolation &interpolation() const { return interpolation_; }
  void setInterpolation(const Interpolation &v) { interpolation_ = v; }

  const CTranslate3D &animTranslation() const { return animTranslation_; }
  void setAnimTranslation(const CTranslate3D &v) {
    animTranslation_ = v; animMatrixValid_ = false; }

  const CRotate3D &animRotation() const { return animRotation_; }
  void setAnimRotation(const CRotate3D &v) {
    animRotation_ = v; animMatrixValid_ = false; }

  const CScale3D &animScale() const { return animScale_; }
  void setAnimScale(const CScale3D &v) {
    animScale_ = v; animMatrixValid_ = false; }

  const CMatrix3D &animMatrix() const {
    if (! animMatrixValid_) {
      animMatrix_ = animTranslation_.matrix()*animRotation_.matrix()*animScale_.matrix();

      animMatrixValid_ = true;
    }

    return animMatrix_;
  }

 private:
  // interpolation range
  Reals   range_;
  OptReal rangeMin_;
  OptReal rangeMax_;

  // transformations (only one non-empty array of these)
  Rotations    rotations_;
  Translations translations_;
  Scales       scales_;

  // interplation type
  Interpolation interpolation_ { Interpolation::NONE };

  // current (calculated) transformations
  mutable CTranslate3D animTranslation_;
  mutable CRotate3D    animRotation_;
  mutable CScale3D     animScale_;

  // matrix product of above (t*r*s)
  mutable CMatrix3D animMatrix_      { CMatrix3D::identity() };
  mutable bool      animMatrixValid_ { false };
};

#endif
