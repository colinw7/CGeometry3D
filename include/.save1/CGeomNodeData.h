#ifndef CGEOM_NODE_DATA_H
#define CGEOM_NODE_DATA_H

#include <CGeomAnimationData.h>

#include <CTranslate3D.h>
#include <CRotate3D.h>
#include <CScale3D.h>

#include <map>
#include <string>

class CGeomObject3D;

// node (joint) data
class CGeomNodeData {
 public:
  using AnimationData  = CGeomAnimationData;
  using AnimationDatas = std::map<std::string, AnimationData>;

 public:
  CGeomNodeData();

  const AnimationDatas &animationDatas() const { return animationDatas_; }

  bool isValid() const { return valid_; }
  void setValid(bool b) { valid_ = b; }

  bool isSelected() const { return selected_; }
  void setSelected(bool b) { selected_ = b; }

  int ind() const { return ind_; }
  void setInd(int i) { ind_ = i; }

  int index() const { return index_; }
  void setIndex(int i) { index_ = i; }

  const std::string &name() const { return name_; }
  void setName(const std::string &s) { name_ = s; }

  bool isJoint() const { return isJoint_; }
  void setJoint(bool b) { isJoint_ = b; }

  int parent() const { return parent_; }
  void setParent(int i) { parent_ = i; }

  void resizeChildren(uint n) { children_.resize(n); }
  void setChild(uint i, int child) { assert(i < children_.size()); children_[i] = child; }

  const CMatrix3D &inverseBindMatrix() const { return inverseBindMatrix_; }
  void setInverseBindMatrix(const CMatrix3D &v) { inverseBindMatrix_ = v; }

  const CGeomObject3D *object() const { return object_; }
  void setObject(CGeomObject3D *p) { object_ = p; }

  void setAnimationData(const std::string &name, const AnimationData &data) {
    animationDatas_[name] = data;
  }

  bool hasAnimationData(const std::string &name) const {
    return (animationDatas_.find(name) != animationDatas_.end());
  }

  AnimationData &getAnimationData(const std::string &name,
                                  const AnimationData &data=AnimationData()) {
    auto pn = animationDatas_.find(name);

    if (pn == animationDatas_.end())
      pn = animationDatas_.insert(pn, AnimationDatas::value_type(name, data));

    return (*pn).second;
  }

  const CTranslate3D &localTranslation() const { return localTranslation_; }
  void setLocalTranslation(const CTranslate3D &v) {
    localTranslation_ = v; localTransformValid_ = false; }

  const CRotate3D &localRotation() const { return localRotation_; }
  void setLocalRotation(const CRotate3D &v) {
    localRotation_ = v; localTransformValid_ = false; }

  const CScale3D &localScale() const { return localScale_; }
  void setLocalScale(const CScale3D &v) {
    localScale_ = v; localTransformValid_ = false; }

  const CMatrix3D &globalTransform() const { return globalTransform_; }
  void setGlobalTransform(const CMatrix3D &v) { globalTransform_ = v; }

  const CMatrix3D &localTransform() const {
    if (! localTransformValid_) {
      localTransform_ = localTranslation_.matrix()*localRotation_.matrix()*localScale_.matrix();

      localTransformValid_ = true;
    }

    return localTransform_;
  }

  const CMatrix3D &animMatrix() const { return animMatrix_; }
  void setAnimMatrix(const CMatrix3D &v) { animMatrix_ = v; }

  const CMatrix3D &hierAnimMatrix() const { return hierAnimMatrix_; }
  void setHierAnimMatrix(const CMatrix3D &v) { hierAnimMatrix_ = v; }

  CMatrix3D calcNodeAnimMatrix(const CMatrix3D &inverseMeshMatrix) const {
    return inverseMeshMatrix*hierAnimMatrix()*inverseBindMatrix();
  }

 private:
  bool valid_    { false };
  bool selected_ { false };

  int         ind_     { -1 };
  int         index_   { -1 };
  std::string name_;
  bool        isJoint_ { false };

  // parent and children nodes (bones)
  int              parent_ { -1 };
  std::vector<int> children_;

  CGeomObject3D *object_ { nullptr };

  // inverse bind matrix (transform to parent coords)
  CMatrix3D inverseBindMatrix_ { CMatrix3D::identity() };

  // animation data per animation name
  AnimationDatas animationDatas_;

  // explicit/default joint transformations
  mutable CTranslate3D localTranslation_;
  mutable CRotate3D    localRotation_;
  mutable CScale3D     localScale_;

  // matrix product of above (t*r*s)
  mutable CMatrix3D localTransform_      { CMatrix3D::identity() };
  mutable bool      localTransformValid_ { false };

  // explicit/default joint hier transformations
  mutable CMatrix3D globalTransform_ { CMatrix3D::identity() };

  // current animation matrix (local, hierarchical)
  mutable CMatrix3D animMatrix_     { CMatrix3D::identity() };
  mutable CMatrix3D hierAnimMatrix_ { CMatrix3D::identity() };
};

#endif
