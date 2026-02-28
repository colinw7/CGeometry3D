#ifndef CGeomAnimationData_H
#define CGeomAnimationData_H

#include <CTranslate3D.h>
#include <CRotate3D.h>
#include <CScale3D.h>

#include <optional>
#include <vector>
#include <map>

enum class CGeomAnimInterpolation {
  NONE,
  LINEAR,
  STEP,
  CUBICSPLINE
};

template<typename T>
struct CGeomAnimValues {
  using Reals         = std::vector<double>;
  using OptReal       = std::optional<double>;
  using Values        = std::vector<T>;
  using Interpolation = CGeomAnimInterpolation;

  // interpolation range
  Reals   range;
  OptReal rangeMin;
  OptReal rangeMax;

  Interpolation interpolation { Interpolation::NONE };

  Values values;
};

// set of transformations and associated range to interplate into
class CGeomAnimationData {
 public:
  using Reals         = std::vector<double>;
  using OptReal       = std::optional<double>;
  using Translations  = std::vector<CVector3D>;
  using Rotations     = std::vector<CQuaternion>;
  using Scales        = std::vector<CVector3D>;
  using Transforms    = std::vector<CMatrix3D>;
  using Interpolation = CGeomAnimInterpolation;

 public:
  // translation
  const Reals &translationRange() const { return translation_.range; }
  void setTranslationRange(const Reals &v) { translation_.range = v; }
  void addTranslationRangeValue(double r) { translation_.range.push_back(r); }

  const OptReal &translationRangeMin() const { return translation_.rangeMin; }
  void setTranslationRangeMin(const OptReal &v) { translation_.rangeMin = v; }

  const OptReal &translationRangeMax() const { return translation_.rangeMax; }
  void setTranslationRangeMax(const OptReal &v) { translation_.rangeMax = v; }

  const Translations &translations() const { return translation_.values; }
  void setTranslations(const Translations &v) { translation_.values = v; }
  void addTranslation(const CVector3D &v) { translation_.values.push_back(v); }

  const Interpolation &translationInterpolation() const { return translation_.interpolation; }
  void setTranslationInterpolation(const Interpolation &v) { translation_.interpolation = v; }

  //---

  // rotation
  const Reals &rotationRange() const { return rotation_.range; }
  void setRotationRange(const Reals &v) { rotation_.range = v; }
  void addRotationRangeValue(double r) { rotation_.range.push_back(r); }

  const OptReal &rotationRangeMin() const { return rotation_.rangeMin; }
  void setRotationRangeMin(const OptReal &v) { rotation_.rangeMin = v; }

  const OptReal &rotationRangeMax() const { return rotation_.rangeMax; }
  void setRotationRangeMax(const OptReal &v) { rotation_.rangeMax = v; }

  const Rotations &rotations() const { return rotation_.values; }
  void setRotations(const Rotations &v) { rotation_.values = v; }
  void addRotation(const CQuaternion &q) { rotation_.values.push_back(q); }

  const Interpolation &rotationInterpolation() const { return rotation_.interpolation; }
  void setRotationInterpolation(const Interpolation &v) { rotation_.interpolation = v; }

  //---

  // scale
  const Reals &scaleRange() const { return scale_.range; }
  void setScaleRange(const Reals &v) { scale_.range = v; }
  void addScaleRangeValue(double r) { scale_.range.push_back(r); }

  const OptReal &scaleRangeMin() const { return scale_.rangeMin; }
  void setScaleRangeMin(const OptReal &v) { scale_.rangeMin = v; }

  const OptReal &scaleRangeMax() const { return scale_.rangeMax; }
  void setScaleRangeMax(const OptReal &v) { scale_.rangeMax = v; }

  const Scales &scales() const { return scale_.values; }
  void setScales(const Scales &v) { scale_.values = v; }
  void addScale(const CVector3D &v) { scale_.values.push_back(v); }

  const Interpolation &scaleInterpolation() const { return scale_.interpolation; }
  void setScaleInterpolation(const Interpolation &v) { scale_.interpolation = v; }

  //---

  // transform
  const Reals &transformRange() const { return transform_.range; }
  void setTransformRange(const Reals &v) { transform_.range = v; }
  void addTransformRangeValue(double r) { transform_.range.push_back(r); }

  const OptReal &transformRangeMin() const { return transform_.rangeMin; }
  void setTransformRangeMin(const OptReal &v) { transform_.rangeMin = v; }

  const OptReal &transformRangeMax() const { return transform_.rangeMax; }
  void setTransformRangeMax(const OptReal &v) { transform_.rangeMax = v; }

  const Transforms &transforms() const { return transform_.values; }
  void setTransforms(const Transforms &v) { transform_.values = v; }
  void addTransform(const CMatrix3D &v) { transform_.values.push_back(v); }

  const Interpolation &transformInterpolation() const { return transform_.interpolation; }
  void setTransformInterpolation(const Interpolation &v) { transform_.interpolation = v; }

  //---

  // transform
  const CTranslate3D &animTranslation() const { return animTranslation_; }
  void setAnimTranslation(const CTranslate3D &v) {
    animTranslation_ = v; animMatrixParts_ = true; animMatrixValid_ = false; }

  const CRotate3D &animRotation() const { return animRotation_; }
  void setAnimRotation(const CRotate3D &v) {
    animRotation_ = v; animMatrixParts_ = true; animMatrixValid_ = false; }

  const CScale3D &animScale() const { return animScale_; }
  void setAnimScale(const CScale3D &v) {
    animScale_ = v; animMatrixParts_ = true; animMatrixValid_ = false; }

  const CMatrix3D &animTransform() const { return animTransform_; }
  void setAnimTransform(const CMatrix3D &v) {
    animTransform_ = v; animMatrixParts_ = false; animMatrixValid_ = false; }

  //---

  const CMatrix3D &animMatrix() const {
    if (! animMatrixValid_) {
      if (animMatrixParts_)
        animMatrix_ = animTranslation_.matrix()*animRotation_.matrix()*animScale_.matrix();
      else
        animMatrix_ = animTransform_;

      animMatrixValid_ = true;
    }

    return animMatrix_;
  }

 private:
  // transformations (only one non-empty array of these)
  CGeomAnimValues<CVector3D>   translation_;
  CGeomAnimValues<CQuaternion> rotation_;
  CGeomAnimValues<CVector3D>   scale_;
  CGeomAnimValues<CMatrix3D>   transform_;

  // current (calculated) transformations
  mutable CTranslate3D animTranslation_;
  mutable CRotate3D    animRotation_;
  mutable CScale3D     animScale_;
  mutable CMatrix3D    animTransform_;

  // matrix product of above (t*r*s)
  mutable CMatrix3D animMatrix_      { CMatrix3D::identity() };
  mutable bool      animMatrixValid_ { false };
  mutable bool      animMatrixParts_ { false };
};

#endif
