#include <CGeomCircle3D.h>
#include <CGeometry3D.h>

CGeomCircle3D::
CGeomCircle3D(CGeomScene3D *pscene, const std::string &name,
              const CPoint3D &center, double radius, const ConfigData &configData) :
 CGeomObject3D(pscene, name), center_(center), radius_(radius), configData_(configData)
{
  addGeometry(this, center_, radius_, configData_);
}

CGeomCircle3D *
CGeomCircle3D::
dup() const
{
  return new CGeomCircle3D(*this);
}

void
CGeomCircle3D::
addGeometry(CGeomObject3D *object, const CPoint3D &center, double radius,
            const ConfigData &configData)
{
  if (configData.num_xy < 4)
    return;

  // create points on arc from angleStart to angleStart + angleDelta;
  double a  = configData.angleStart;
  double da = configData.angleDelta/(configData.num_xy - 1);

  std::vector<uint> inds;

  int ii = 0;

  if (configData.type == Type::FAN) {
    inds.resize(configData.num_xy + 1);

    inds[ii] = object->addVertex(CPoint3D(0, 0, 0));

    auto &vertex = object->getVertex(inds[ii]);

    vertex.setTextureMap(CPoint2D(0, 0));

    ++ii;
  }
  else
    inds.resize(configData.num_xy);

  for (uint i = 0; i < configData.num_xy; ++i) {
    auto c = std::cos(a);
    auto s = std::sin(a);

    auto x = radius*c;
    auto y = radius*s;

    inds[ii] = object->addVertex(CPoint3D(x, y, 0));

    auto &vertex = object->getVertex(inds[ii]);

    vertex.setTextureMap(CPoint2D((c + 1)/2.0, (s + 1)/2.0));

    a += da;

    ++ii;
  }

  if      (configData.type == Type::FAN) {
    for (uint i = 0; i < configData.num_xy; ++i) {
      uint i1 = i  + 1;
      uint i2 = i1 + 1;

      if (i2 > configData.num_xy)
        i2 = 1;

      VertexIList vertices;

      vertices.push_back(inds[i1]);
      vertices.push_back(inds[0 ]);
      vertices.push_back(inds[i2]);

      auto faceNum = object->addFace(vertices);

      auto *face = object->getFaceP(faceNum);

      face->setNormal(CVector3D(0, 0, 1));
      face->setTwoSided(true);
    }
  }
  else if (configData.type == Type::NGON) {
    auto faceNum = object->addFace(inds);

    auto *face = object->getFaceP(faceNum);

    face->setNormal(CVector3D(0, 1, 0));
    face->setTwoSided(true);
  }
  else if (configData.type == Type::LINE) {
    for (uint i = 0; i < configData.num_xy; ++i) {
      uint i1 = i  + 1;
      uint i2 = i1 + 1;

      if (i2 > configData.num_xy)
        i2 = 1;

      (void) object->addLine(i1, i2);
    }
  }

  object->moveBy(center);
}
