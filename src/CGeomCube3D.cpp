#include <CGeomCube3D.h>
#include <CConv.h>

CGeomCube3D::
CGeomCube3D(CGeomScene3D *pscene, const std::string &name,
            double xc, double yc, double zc, double r) :
 CGeomObject3D(pscene, name)
{
  addGeometry(this, xc, yc, zc, r);
}

void
CGeomCube3D::
addGeometry(CGeomObject3D *object, double xc, double yc, double zc, double r)
{
  uint vertices[8];

  vertices[0] = object->addVertex(CPoint3D(xc + r/2, yc - r/2, zc + r/2));
  vertices[1] = object->addVertex(CPoint3D(xc + r/2, yc - r/2, zc - r/2));
  vertices[2] = object->addVertex(CPoint3D(xc + r/2, yc + r/2, zc - r/2));
  vertices[3] = object->addVertex(CPoint3D(xc + r/2, yc + r/2, zc + r/2));
  vertices[4] = object->addVertex(CPoint3D(xc - r/2, yc - r/2, zc + r/2));
  vertices[5] = object->addVertex(CPoint3D(xc - r/2, yc - r/2, zc - r/2));
  vertices[6] = object->addVertex(CPoint3D(xc - r/2, yc + r/2, zc - r/2));
  vertices[7] = object->addVertex(CPoint3D(xc - r/2, yc + r/2, zc + r/2));

  object->addFace(CConv::toVector(vertices[0], vertices[1], vertices[2], vertices[3]));
  object->addFace(CConv::toVector(vertices[1], vertices[5], vertices[6], vertices[2]));
  object->addFace(CConv::toVector(vertices[5], vertices[4], vertices[7], vertices[6]));
  object->addFace(CConv::toVector(vertices[4], vertices[0], vertices[3], vertices[7]));
  object->addFace(CConv::toVector(vertices[3], vertices[2], vertices[6], vertices[7]));
  object->addFace(CConv::toVector(vertices[4], vertices[5], vertices[1], vertices[0]));
}
