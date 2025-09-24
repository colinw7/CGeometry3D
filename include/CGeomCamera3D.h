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

  ACCESSOR(FieldOfView, double, fov )
  ACCESSOR(Near       , double, near)
  ACCESSOR(Far        , double, far )

  CPoint3D getPosition() const { return coordFrame_.getOrigin().point(); }

  const CVector3D &getDirection() const { return direction_; }

  void setPosition(const CPoint3D &position);
  void setDirection(const CVector3D &dir);

  const CMatrix3DH &getProjectionMatrix() const { return projection_matrix_; }

  const CMatrix3D &getWorldMatrix() const { return world_matrix_; }

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
  CCoordFrame3D coordFrame_;
  CVector3D     direction_ { 0, 0, 1 };
  double        fov_ { 90 };
  double        near_ { 0.1 }, far_ { 1000 };
  CMatrix3DH    projection_matrix_;
  CMatrix3D     world_matrix_;
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
