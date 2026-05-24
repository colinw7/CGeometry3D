#include <CGeomCube3D.h>
#include <CConv.h>

CGeomCube3D::
CGeomCube3D(CGeomScene3D *pscene, const std::string &name, const CPoint3D &c, double r) :
 CGeomObject3D(pscene, name)
{
  addGeometry(this, c.x, c.y, c.z, r);
}

CGeomCube3D::
CGeomCube3D(CGeomScene3D *pscene, const std::string &name,
            double xc, double yc, double zc, double r) :
 CGeomObject3D(pscene, name)
{
  addGeometry(this, xc, yc, zc, r);
}

void
CGeomCube3D::
addGeometry(CGeomObject3D *object, const CPoint3D &c, double r)
{
  addGeometry(object, c.x, c.y, c.z, r);
}

void
CGeomCube3D::
addGeometry(CGeomObject3D *object, double xc, double yc, double zc, double r)
{
  uint v[8];

  v[0] = object->addVertex(CPoint3D(xc + r/2, yc - r/2, zc + r/2));
  v[1] = object->addVertex(CPoint3D(xc + r/2, yc - r/2, zc - r/2));
  v[2] = object->addVertex(CPoint3D(xc + r/2, yc + r/2, zc - r/2));
  v[3] = object->addVertex(CPoint3D(xc + r/2, yc + r/2, zc + r/2));
  v[4] = object->addVertex(CPoint3D(xc - r/2, yc - r/2, zc + r/2));
  v[5] = object->addVertex(CPoint3D(xc - r/2, yc - r/2, zc - r/2));
  v[6] = object->addVertex(CPoint3D(xc - r/2, yc + r/2, zc - r/2));
  v[7] = object->addVertex(CPoint3D(xc - r/2, yc + r/2, zc + r/2));

  struct FaceData {
    uint        ind    { 0 };
    CVector3D   normal { 0, 1, 0 };
    std::string name   { 0, 1, 0 };
  };

  std::vector<FaceData> faceData;
  faceData.resize(6);

  faceData[0].ind = object->addFace(CConv::toVector(v[0], v[1], v[2], v[3])); // Right
  faceData[1].ind = object->addFace(CConv::toVector(v[1], v[5], v[6], v[2])); // Back
  faceData[2].ind = object->addFace(CConv::toVector(v[5], v[4], v[7], v[6])); // Left
  faceData[3].ind = object->addFace(CConv::toVector(v[4], v[0], v[3], v[7])); // Front
  faceData[4].ind = object->addFace(CConv::toVector(v[3], v[2], v[6], v[7])); // Top
  faceData[5].ind = object->addFace(CConv::toVector(v[4], v[5], v[1], v[0])); // Bottom

  faceData[0].normal = CVector3D( 1.0,  0.0,  0.0);
  faceData[1].normal = CVector3D( 0.0,  0.0, -1.0);
  faceData[2].normal = CVector3D(-1.0,  0.0,  0.0);
  faceData[3].normal = CVector3D( 0.0,  0.0,  1.0);
  faceData[4].normal = CVector3D( 0.0,  1.0,  0.0);
  faceData[5].normal = CVector3D( 0.0, -1.0,  0.0);

  faceData[0].name = "right";
  faceData[1].name = "back";
  faceData[2].name = "left";
  faceData[3].name = "front";
  faceData[4].name = "top";
  faceData[5].name = "bottom";

  auto setFaceData = [&](const FaceData &data) {
    auto *face = object->getFaceP(data.ind);

    CVector3D n;
    face->calcModelNormal(n);

    face->setNormal(data.normal);
    assert(n == data.normal);

    std::vector<CPoint2D> points;

    points.push_back(CPoint2D(0.0, 0.0));
    points.push_back(CPoint2D(1.0, 0.0));
    points.push_back(CPoint2D(1.0, 1.0));
    points.push_back(CPoint2D(0.0, 1.0));

    face->setTexturePoints(points);

    face->setName(data.name);
  };

  for (const auto &d : faceData)
    setFaceData(d);
}
