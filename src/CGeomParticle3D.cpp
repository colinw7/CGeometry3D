#include <CGeomParticle3D.h>
#include <CGeometry3D.h>

void
CGeomParticle3D::
draw(CGeomZBuffer *zbuffer)
{
  zbuffer->setForeground(color_);

  if (size_ <= 1)
    zbuffer->getRenderer()->drawPoint(CIPoint2D(pixel_.x, pixel_.y));
  else
    zbuffer->getRenderer()->fillCircle(CIPoint2D(pixel_.x, pixel_.y), size_);
}
