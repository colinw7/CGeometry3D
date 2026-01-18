#include <CGeomLine3D.h>
#include <CGeomObject3D.h>
#include <CGeomZBuffer.h>

CGeomLine3D::
CGeomLine3D()
{
  init();
}

CGeomLine3D::
CGeomLine3D(CGeomObject3D *pobject, uint startInd, uint endInd) :
 pobject_(pobject), startInd_(startInd), endInd_(endInd)
{
  init();
}

CGeomLine3D::
CGeomLine3D(const CGeomLine3D &line) :
 pobject_ (line.pobject_),
 ind_     (0),
 startInd_(line.startInd_),
 endInd_  (line.endInd_),
 material_(line.material_),
 width_   (line.width_),
 dashes_  (line.dashes_)
{
  init();
}

void
CGeomLine3D::
init()
{
  setVisible(true);
}

CGeomLine3D *
CGeomLine3D::
dup() const
{
  return new CGeomLine3D(*this);
}

//---

void
CGeomLine3D::
setSelected(bool b)
{
  if (b)
    flags_ |= SELECTED;
  else
    flags_ &= uint(~SELECTED);
}

void
CGeomLine3D::
setVisible(bool b)
{
  if (b)
    flags_ |= VISIBLE;
  else
    flags_ &= uint(~VISIBLE);
}

//---

CGeomVertex3D &
CGeomLine3D::
getStartVertex() const
{
  return pobject_->getVertex(startInd_);
}

CGeomVertex3D &
CGeomLine3D::
getEndVertex() const
{
  return pobject_->getVertex(endInd_);
}

void
CGeomLine3D::
moveTo(const CPoint3D &p1, const CPoint3D &p2)
{
  auto &v1 = getStartVertex();
  auto &v2 = getEndVertex  ();

  v1.setModel(p1);
  v2.setModel(p2);
}

void
CGeomLine3D::
draw(CGeomZBuffer *zbuffer)
{
  auto &v1 = getStartVertex();
  auto &v2 = getEndVertex  ();

  const auto &point1 = v1.getPixel();
  const auto &point2 = v2.getPixel();

  zbuffer->drawZLine(int(point1.x), int(point1.y), point1.z,
                     int(point2.x), int(point2.y), point2.z);
}
