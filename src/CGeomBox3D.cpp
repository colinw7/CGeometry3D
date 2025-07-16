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

  uint faces[6];

  faces[0] = object->addFace(CConv::toVector(vertices[0], vertices[1], vertices[2], vertices[3]));
  faces[1] = object->addFace(CConv::toVector(vertices[1], vertices[5], vertices[6], vertices[2]));
  faces[2] = object->addFace(CConv::toVector(vertices[5], vertices[4], vertices[7], vertices[6]));
  faces[3] = object->addFace(CConv::toVector(vertices[4], vertices[0], vertices[3], vertices[7]));
  faces[4] = object->addFace(CConv::toVector(vertices[3], vertices[2], vertices[6], vertices[7]));
  faces[5] = object->addFace(CConv::toVector(vertices[4], vertices[5], vertices[1], vertices[0]));

  auto setFaceTextureMap = [&](int i) {
    auto &face = object->getFace(faces[i]);
    std::vector<CPoint2D> points;
    points.push_back(CPoint2D(0, 0));
    points.push_back(CPoint2D(1, 0));
    points.push_back(CPoint2D(1, 1));
    points.push_back(CPoint2D(0, 1));
    face.setTexturePoints(points);
  };

  for (int i = 0; i < 6; ++i)
    setFaceTextureMap(faces[i]);
}
