#ifndef CGEOM_CAMERA_3D_H
#define CGEOM_CAMERA_3D_H

#include <CMatrix3DH.h>
#include <CCoordFrame3D.h>
#include <CGeomObject3D.h>
#include <CGeomVertex3D.h>

class CGeomCamera3D {
 public:
  enum ProjectionType {
    PROJECTION_PERSPECTIVE,
    PROJECTION_FRUSTRUM,
    PROJECTION_ORTHOGONAL,
    PROJECTION_OBLIQUE
  };

 public:
  CGeomCamera3D();

  virtual ~CGeomCamera3D() { }

  const uint &getId() const { return id_; }
  void setId(const uint &v) { id_ = v; }

  double getFieldOfView() const { return fov_; }
  void setFieldOfView(double r) { fov_ = r; }

  double getNear() const { return near_; }
  void setNear(double r) { near_ = r; }

  double getFar() const { return far_; }
  void setFar(double r) { far_ = r; }

  CPoint3D getPosition() const { return coordFrame_.getOrigin().point(); }

  const CVector3D &getDirection() const { return direction_; }

  void setPosition(const CPoint3D &position);
  void setDirection(const CVector3D &dir);

  const CMatrix3DH &getProjectionMatrix() const { return projectionMatrix_; }

  const CMatrix3D &getWorldMatrix() const { return worldMatrix_; }

  CPoint3D transformTo(const CPoint3D &p) const { return coordFrame_.transformTo(p); }

  virtual void createProjectionMatrix(double left, double right, double bottom, double top) = 0;

  void createWorldMatrix(int w, int h);

  void moveX(double dx);
  void moveY(double dy);
  void moveZ(double dz);

  void rotateX(double dx);
  void rotateY(double dy);
  void rotateZ(double dz);

 protected:
  uint          id_        { 0 };
  CCoordFrame3D coordFrame_;
  CVector3D     direction_ { 0, 0, 1 };
  double        fov_       { 90 };
  double        near_      { 0.1 };
  double        far_       { 1000 };
  CMatrix3DH    projectionMatrix_;
  CMatrix3D     worldMatrix_;
};

//-------------

class CGeomPerspectiveCamera3D : public CGeomCamera3D {
 public:
  CGeomPerspectiveCamera3D() : CGeomCamera3D() { }

  void createProjectionMatrix(double left, double right, double bottom, double top) override;
};

class CGeomFrustrumCamera3D : public CGeomCamera3D {
 public:
  CGeomFrustrumCamera3D() : CGeomCamera3D() { }

  void createProjectionMatrix(double left, double right, double bottom, double top) override;
};

class CGeomOrthogonalCamera3D : public CGeomCamera3D {
 public:
  CGeomOrthogonalCamera3D() : CGeomCamera3D() { }

  void createProjectionMatrix(double left, double right, double bottom, double top) override;
};

class CGeomObliqueCamera3D : public CGeomCamera3D {
 public:
  CGeomObliqueCamera3D() : CGeomCamera3D() { }

  void createProjectionMatrix(double left, double right, double bottom, double top) override;
};

#endif
