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
