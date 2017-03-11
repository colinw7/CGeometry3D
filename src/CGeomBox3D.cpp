#include <CGeomBox3D.h>
#include <CConv.h>

CGeomBox3D::
CGeomBox3D(CGeomScene3D *pscene, const std::string &name,
           double xc, double yc, double zc, double xs, double ys, double zs) :
 CGeomObject3D(pscene, name)
{
  addGeometry(this, xc, yc, zc, xs, ys, zs);
}

void
CGeomBox3D::
addGeometry(CGeomObject3D *object,
            double xc, double yc, double zc, double xs, double ys, double zs)
{
  uint vertices[8];

  vertices[0] = object->addVertex(CPoint3D(xc + xs/2, yc - ys/2, zc + zs/2));
  vertices[1] = object->addVertex(CPoint3D(xc + xs/2, yc - ys/2, zc - zs/2));
  vertices[2] = object->addVertex(CPoint3D(xc + xs/2, yc + ys/2, zc - zs/2));
  vertices[3] = object->addVertex(CPoint3D(xc + xs/2, yc + ys/2, zc + zs/2));
  vertices[4] = object->addVertex(CPoint3D(xc - xs/2, yc - ys/2, zc + zs/2));
  vertices[5] = object->addVertex(CPoint3D(xc - xs/2, yc - ys/2, zc - zs/2));
  vertices[6] = object->addVertex(CPoint3D(xc - xs/2, yc + ys/2, zc - zs/2));
  vertices[7] = object->addVertex(CPoint3D(xc - xs/2, yc + ys/2, zc + zs/2));

  object->addFace(CConv::toVector(vertices[0], vertices[1], vertices[2], vertices[3]));
  object->addFace(CConv::toVector(vertices[1], vertices[5], vertices[6], vertices[2]));
  object->addFace(CConv::toVector(vertices[5], vertices[4], vertices[7], vertices[6]));
  object->addFace(CConv::toVector(vertices[4], vertices[0], vertices[3], vertices[7]));
  object->addFace(CConv::toVector(vertices[3], vertices[2], vertices[6], vertices[7]));
  object->addFace(CConv::toVector(vertices[4], vertices[5], vertices[1], vertices[0]));
}
