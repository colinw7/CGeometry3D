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

CGeomPyramid3D::
CGeomPyramid3D(CGeomScene3D *pscene, const std::string &name,
               const CPoint3D &c, double w, double h) :
 CGeomObject3D(pscene, name)
{
  addGeometry(c.x, c.y, c.z, w, h);
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
  return addGeometry(object, CPoint3D(xc, yc, zc), w, h);
}

void
CGeomPyramid3D::
addGeometry(CGeomObject3D *object, const CPoint3D &c, double w, double h)
{
  // bottom
  auto p1 = CPoint3D(c.x - w/2, c.y    , c.z - w/2);
  auto p2 = CPoint3D(c.x - w/2, c.y    , c.z + w/2);
  auto p3 = CPoint3D(c.x + w/2, c.y    , c.z + w/2);
  auto p4 = CPoint3D(c.x + w/2, c.y    , c.z - w/2);

  auto vertex1 = object->addVertex(p1);
  auto vertex2 = object->addVertex(p2);
  auto vertex3 = object->addVertex(p3);
  auto vertex4 = object->addVertex(p4);

  (void) object->addFace(CConv::toVector(vertex4, vertex3, vertex2, vertex1));

  object->setVertexNormal(vertex1, CVector3D(0, -1, 0));
  object->setVertexNormal(vertex2, CVector3D(0, -1, 0));
  object->setVertexNormal(vertex3, CVector3D(0, -1, 0));
  object->setVertexNormal(vertex4, CVector3D(0, -1, 0));

  //---

  // sides
  auto p5 = CPoint3D(c.x, c.y + h, c.z); // tip

  auto calcNormal = [](const CPoint3D &fp1, const CPoint3D &fp2, const CPoint3D &fp3) {
    CVector3D diff1(fp1, fp2);
    CVector3D diff2(fp2, fp3);

    return diff1.crossProduct(diff2).normalized();
  };

  auto addFace = [&](const CPoint3D &fp1, const CPoint3D &fp2, const CPoint3D &fp3) {
    auto v1 = object->addVertex(fp1);
    auto v2 = object->addVertex(fp2);
    auto v3 = object->addVertex(fp3);

#if 0
    auto faceNum = object->addFace(CConv::toVector(v1, v2, v3));

    CVector3D normal;
    auto *face = object->getFaceP(faceNum);
    face->calcNormal(normal);
#else
    (void) object->addFace(CConv::toVector(v1, v2, v3));

    auto normal = calcNormal(fp1, fp2, fp3);
#endif

    object->setVertexNormal(v1, normal);
    object->setVertexNormal(v2, normal);
    object->setVertexNormal(v3, normal);
  };

  addFace(p1, p2, p5);
  addFace(p2, p3, p5);
  addFace(p3, p4, p5);
  addFace(p4, p1, p5);
}
