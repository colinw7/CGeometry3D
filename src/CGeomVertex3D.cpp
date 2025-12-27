#include <CGeomVertex3D.h>
#include <CGeomZBuffer.h>
#include <CGeomCamera3D.h>

CGeomVertex3D::
CGeomVertex3D(CGeomObject3D *pobject, const CPoint3D &point) :
 CGeomPoint3D(point), pobject_(pobject)
{
}

CGeomVertex3D::
CGeomVertex3D(const CGeomVertex3D &vertex) :
 CGeomPoint3D(vertex),
 pobject_   (vertex.pobject_),
 ind_       (0),
 color_     (vertex.color_),
 normal_    (vertex.normal_),
 tmap_      (vertex.tmap_),
 clipSide_  (vertex.clipSide_)
{
}

CGeomVertex3D &
CGeomVertex3D::
operator=(const CGeomVertex3D &vertex)
{
  pobject_  = vertex.pobject_;
  ind_      = 0;
  color_    = vertex.color_;
  normal_   = vertex.normal_;
  tmap_     = vertex.tmap_;
  clipSide_ = vertex.clipSide_;

  return *this;
}

CGeomVertex3D *
CGeomVertex3D::
dup() const
{
  return new CGeomVertex3D(*this);
}

void
CGeomVertex3D::
draw(CGeomZBuffer *zbuffer)
{
  zbuffer->getRenderer()->drawPoint(CIPoint2D(int(pixel_.x), int(pixel_.y)));
}

void
CGeomVertex3D::
swapXY()
{
  if (normal_)
    normal_ = CVector3D(normal_->y(), normal_->x(), normal_->z());

  CGeomPoint3D::swapXY();
}

void
CGeomVertex3D::
swapYZ()
{
  if (normal_)
    normal_ = CVector3D(normal_->x(), normal_->z(), normal_->y());

  CGeomPoint3D::swapYZ();
}

void
CGeomVertex3D::
swapZX()
{
  if (normal_)
    normal_ = CVector3D(normal_->z(), normal_->y(), normal_->x());

  CGeomPoint3D::swapZX();
}

void
CGeomVertex3D::
invertX()
{
  if (normal_)
    normal_ = CVector3D(-normal_->x(), normal_->y(), normal_->z());

  CGeomPoint3D::invertX();
}

void
CGeomVertex3D::
invertY()
{
  if (normal_)
    normal_ = CVector3D(normal_->x(), -normal_->y(), normal_->z());

  CGeomPoint3D::invertY();
}

void
CGeomVertex3D::
invertZ()
{
  if (normal_)
    normal_ = CVector3D(normal_->x(), normal_->y(), -normal_->z());

  CGeomPoint3D::invertZ();
}
