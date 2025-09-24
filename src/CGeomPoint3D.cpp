#include <CGeomPoint3D.h>
#include <CGeomCamera3D.h>

CGeomPoint3D::
CGeomPoint3D(const CGeomPoint3D &vertex) :
 model_    (vertex.model_),
 current_  (vertex.current_),
 viewed_   (vertex.viewed_),
 projected_(vertex.projected_),
 pixel_    (vertex.pixel_)
{
}

CGeomPoint3D::
~CGeomPoint3D()
{
}

CGeomPoint3D &
CGeomPoint3D::
operator=(const CGeomPoint3D &vertex)
{
  model_     = vertex.model_;
  current_   = vertex.current_;
  viewed_    = vertex.viewed_;
  projected_ = vertex.projected_;
  pixel_     = vertex.pixel_;

  return *this;
}

void
CGeomPoint3D::
place(const CMatrix3D &matrix)
{
  matrix.multiplyPoint(model_, current_);
}

void
CGeomPoint3D::
view(const CMatrix3D &matrix)
{
  matrix.multiplyPoint(current_, viewed_);
}

void
CGeomPoint3D::
modelToPixel(const CCoordFrame3D &coordFrame, const CGeomCamera3D &camera)
{
  current_ = coordFrame.transformFrom(model_);

  currentToPixel(camera);
}

void
CGeomPoint3D::
currentToPixel(const CGeomCamera3D &camera)
{
  currentToProjected(camera);

  toPixel(camera);
}

void
CGeomPoint3D::
currentToProjected(const CGeomCamera3D &camera)
{
  currentToViewed(camera);

  project(camera);
}

void
CGeomPoint3D::
currentToViewed(const CGeomCamera3D &camera)
{
  viewed_ = camera.transformTo(current_);
}

void
CGeomPoint3D::
project(const CGeomCamera3D &camera)
{
#if 0
  if (projection > 0) {
    if (viewed_.z < 0) {
      double factor = -projection/viewed_.z;

      projected_.x = viewed_.x*factor;
      projected_.y = viewed_.y*factor;
    }
    else {
      projected_.x = viewed_.x;
      projected_.y = viewed_.y;
    }
  }
  else {
    projected_.x = viewed_.x;
    projected_.y = viewed_.y;
  }
#else
  camera.getProjectionMatrix().multiplyPoint(viewed_, projected_);
#endif
}

void
CGeomPoint3D::
toPixel(const CGeomCamera3D &camera)
{
  camera.getWorldMatrix().multiplyPoint(projected_, pixel_);
}
