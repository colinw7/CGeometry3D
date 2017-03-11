#include <CGeomLine3D.h>
#include <CGeomObject3D.h>
#include <CGeomZBuffer.h>

CGeomLine3D::
CGeomLine3D(CGeomObject3D *pobject, uint start_ind, uint end_ind) :
 pobject_  (pobject),
 start_ind_(start_ind),
 end_ind_  (end_ind)
{
}

CGeomLine3D::
CGeomLine3D(const CGeomLine3D &line) :
 pobject_  (line.pobject_),
 ind_      (0),
 start_ind_(line.start_ind_),
 end_ind_  (line.end_ind_),
 material_ (line.material_),
 width_    (line.width_),
 dashes_   (line.dashes_)
{
}

CGeomLine3D *
CGeomLine3D::
dup() const
{
  return new CGeomLine3D(*this);
}

CGeomVertex3D &
CGeomLine3D::
getStartVertex() const
{
  return pobject_->getVertex(start_ind_);
}

CGeomVertex3D &
CGeomLine3D::
getEndVertex() const
{
  return pobject_->getVertex(end_ind_);
}

void
CGeomLine3D::
moveTo(const CPoint3D &p1, const CPoint3D &p2)
{
  CGeomVertex3D &v1 = getStartVertex();
  CGeomVertex3D &v2 = getEndVertex  ();

  v1.setModel(p1);
  v2.setModel(p2);
}

void
CGeomLine3D::
draw(CGeomZBuffer *zbuffer)
{
  CGeomVertex3D &v1 = getStartVertex();
  CGeomVertex3D &v2 = getEndVertex  ();

  const CPoint3D &point1 = v1.getPixel();
  const CPoint3D &point2 = v2.getPixel();

  zbuffer->drawZLine(int(point1.x), int(point1.y), point1.z,
                     int(point2.x), int(point2.y), point2.z);
}
