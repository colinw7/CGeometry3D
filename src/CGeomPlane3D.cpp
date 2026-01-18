#include <CGeomPlane3D.h>
#include <CGeometry3D.h>

CGeomPlane3D::
CGeomPlane3D(CGeomScene3D *pscene, const std::string &name,
             const CPoint3D &center, double w, double h, const ConfigData &configData) :
 CGeomObject3D(pscene, name), center_(center), w_(w), h_(h), configData_(configData)
{
  addGeometry(this, center_, w_, h_, configData_);
}

CGeomPlane3D *
CGeomPlane3D::
dup() const
{
  return new CGeomPlane3D(*this);
}

void
CGeomPlane3D::
addGeometry(CGeomObject3D *object, const CPoint3D &center, double w, double h,
            const ConfigData & /*configData*/)
{
  std::vector<uint> inds;

  auto addVertex = [&](const CPoint3D &p, const CPoint2D &t) {
    auto ind = object->addVertex(center + p);

    inds.push_back(ind);

    auto &vertex = object->getVertex(ind);

    vertex.setTextureMap(t);

    return ind;
  };

  addVertex(CPoint3D(-w/2, 0, -h/2), CPoint2D(0, 0));
  addVertex(CPoint3D( w/2, 0, -h/2), CPoint2D(1, 0));
  addVertex(CPoint3D( w/2, 0,  h/2), CPoint2D(1, 1));
  addVertex(CPoint3D(-w/2, 0,  h/2), CPoint2D(0, 1));

  auto faceNum = object->addFace(inds);

  auto *face = object->getFaceP(faceNum);

  face->setNormal(CVector3D(0, 1, 0));
  face->setTwoSided(true);
}
