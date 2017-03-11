#include <CGeomPyramid3D.h>
#include <CGeometry3D.h>
#include <CConv.h>

CGeomPyramid3D::
CGeomPyramid3D(CGeomScene3D *pscene, const std::string &name,
               double xc, double yc, double zc, double w, double h) :
 CGeomObject3D(pscene, name)
{
  addGeometry(xc, yc, zc, w, h);
}

void
CGeomPyramid3D::
addGeometry(double xc, double yc, double zc, double w, double h)
{
  addGeometry(this, xc, yc, zc, w, h);
}

void
CGeomPyramid3D::
addGeometry(CGeomObject3D *object, double xc, double yc, double zc,
            double w, double h)
{
  uint vertex1 = object->addVertex(CPoint3D(xc - w/2, yc    , zc - w/2));
  uint vertex2 = object->addVertex(CPoint3D(xc - w/2, yc    , zc + w/2));
  uint vertex3 = object->addVertex(CPoint3D(xc + w/2, yc    , zc + w/2));
  uint vertex4 = object->addVertex(CPoint3D(xc + w/2, yc    , zc - w/2));
  uint vertex5 = object->addVertex(CPoint3D(xc      , yc + h, zc      ));

  object->addFace(CConv::toVector(vertex4, vertex3, vertex2, vertex1));
  object->addFace(CConv::toVector(vertex1, vertex2, vertex5));
  object->addFace(CConv::toVector(vertex2, vertex3, vertex5));
  object->addFace(CConv::toVector(vertex3, vertex4, vertex5));
  object->addFace(CConv::toVector(vertex4, vertex1, vertex5));
}
