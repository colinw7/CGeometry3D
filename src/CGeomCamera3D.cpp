#include <CGeomCamera3D.h>
#include <CGeometry3D.h>
#include <CGeomZBuffer.h>
#include <CTransform3D.h>

CGeomCamera3D::
CGeomCamera3D()
{
}

void
CGeomCamera3D::
setPosition(const CPoint3D &position)
{
  coord_frame_.setOrigin(position);
}

void
CGeomCamera3D::
setDirection(const CVector3D &dir)
{
  CVector3D right, up, dir1;

  coord_frame_.getBasis(right, up, dir1);

  dir1 = dir.unit();

  right = dir1 .crossProduct(up );
  up    = right.crossProduct(dir1);

  if (COrthonormalBasis3D::validate(right, up, dir1)) {
    coord_frame_.setBasis(right, up, dir1);

    direction_ = dir;
  }
}

void
CGeomCamera3D::
createWorldMatrix(int w, int h)
{
  CTransform3D transform(-1,    -1, 0,     1, 1, 1,
                          0, h - 1, 0, w - 1, 0, 1);

  world_matrix_ = *transform.getMatrix();
}

void
CGeomCamera3D::
moveX(double dx)
{
  coord_frame_.moveX(dx);
}

void
CGeomCamera3D::
moveY(double dy)
{
  coord_frame_.moveY(dy);
}

void
CGeomCamera3D::
moveZ(double dz)
{
  coord_frame_.moveZ(dz);
}

void
CGeomCamera3D::
rotateX(double dx)
{
  coord_frame_.rotateAboutX(dx);
}

void
CGeomCamera3D::
rotateY(double dy)
{
  coord_frame_.rotateAboutY(dy);
}

void
CGeomCamera3D::
rotateZ(double dz)
{
  coord_frame_.rotateAboutZ(dz);
}

//----------------------

void
CGeomPerspectiveCamera3D::
createProjectionMatrix(double left, double right, double bottom, double top)
{
  double fov = getFieldOfView();

  double w = fabs(right - left  );
  double h = fabs(top   - bottom);

  double aspect = w/h;

  double near = getNear();
  double far  = getFar ();

  projection_matrix_.buildPerspective(fov, aspect, near, far);
}

void
CGeomFrustrumCamera3D::
createProjectionMatrix(double left, double right, double bottom, double top)
{
  double near = getNear();
  double far  = getFar ();

  projection_matrix_.buildFrustrum(left, right, bottom, top, near, far);
}

void
CGeomOrthogonalCamera3D::
createProjectionMatrix(double left, double right, double bottom, double top)
{
  double near = getNear();
  double far  = getFar ();

  projection_matrix_.buildOrtho(left, right, bottom, top, near, far);
}

void
CGeomObliqueCamera3D::
createProjectionMatrix(double left, double right, double bottom, double top)
{
  double near = getNear();
  double far  = getFar ();

  double w = right - left  ;
  double h = top   - bottom;
  double d = near  - far   ;

  double rpl = right + left  ;
  double tpb = top   + bottom;

  double iw = 1.0/w;
  double ih = 1.0/h;

  double id = 1.0/d;

  double a = 2.0*iw;
  double e = 2.0*ih;
  double i = id;

  double c = iw;
  double f = ih;

  double tx = -(rpl - near)*iw;
  double ty = -(tpb - near)*ih;
  double tz =  near*id;

  projection_matrix_ = CMatrix3DH(a, 0, c, tx,
                                  0, e, f, ty,
                                  0, 0, i, tz,
                                  0, 0, 0, 1);
}
