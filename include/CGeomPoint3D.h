#ifndef CGEOM_POINT_3D_H
#define CGEOM_POINT_3D_H

#include <CPoint3D.h>
#include <CCoordFrame3D.h>

class CGeomCamera3D;

class CGeomPoint3D {
 public:
  CGeomPoint3D(const CPoint3D &point=CPoint3D(0, 0, 0)) :
   model_(point), current_(point), viewed_(point), projected_(point), pixel_(point) {
  }

  CGeomPoint3D(const CGeomPoint3D &point);

  virtual ~CGeomPoint3D();

  CGeomPoint3D &operator=(const CGeomPoint3D &vertex);

  const CPoint3D &getModel() const { return model_; }
  void setModel(const CPoint3D &v) { model_ = v; }

  const CPoint3D &getCurrent() const { return current_; }
  void setCurrent(const CPoint3D &v) { current_ = v; }

  const CPoint3D &getViewed() const { return viewed_; }
  void setViewed(const CPoint3D &v) { viewed_ = v; }

  const CPoint3D &getProjected() const { return projected_; }
  void setProjected(const CPoint3D &v) { projected_ = v; }

  const CPoint3D &getPixel() const { return pixel_; }
  void setPixel(const CPoint3D &v) { pixel_ = v; }

  void place(const CMatrix3D &matrix);

  void view(const CMatrix3D &matrix);

  void modelToPixel(const CCoordFrame3D &coordFrame, const CGeomCamera3D &camera);

  void currentToPixel(const CGeomCamera3D &camera);

  void currentToProjected(const CGeomCamera3D &camera);

  void currentToViewed(const CGeomCamera3D &camera);

  void project(const CGeomCamera3D &camera);

  void toPixel(const CGeomCamera3D &camera);

  void print(std::ostream &os) const {
    os << "model="     << model_     << ", " <<
          "current="   << current_   << ", " <<
          "viewed="    << viewed_    << ", " <<
          "projected=" << projected_ << ", " <<
          "pixel="     << pixel_;
  }

  friend std::ostream &operator<<(std::ostream &os, const CGeomPoint3D &point) {
    point.print(os);

   return os;
  }

 protected:
  CPoint3D model_;
  CPoint3D current_;
  CPoint3D viewed_;
  CPoint3D projected_;
  CPoint3D pixel_;
};

#endif
