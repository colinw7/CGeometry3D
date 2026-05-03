#include <CGeomEdge3D.h>
#include <CGeomObject3D.h>

#include <CLine3D.h>

CGeomEdge3D::
CGeomEdge3D()
{
  vertices_.resize(2);
}

CGeomEdge3D::
CGeomEdge3D(uint v1, uint v2)
{
  vertices_.resize(2);
  vertices_[0] = v1;
  vertices_[1] = v2;
}

CGeomEdge3D::
~CGeomEdge3D()
{
}

bool
CGeomEdge3D::
cmp(const CGeomEdge3D &rhs) const
{
  if ((vertices_[0] == rhs.vertices_[0] && vertices_[1] == rhs.vertices_[1]) ||
      (vertices_[0] == rhs.vertices_[1] && vertices_[1] == rhs.vertices_[0]))
    return 0;

  if (vertices_[0] < rhs.vertices_[0]) return -1;

  return 1;
}

void
CGeomEdge3D::
moveBy(const CVector3D &v)
{
  auto &v1 = object_->getVertex(vertices_[0]);
  auto &v2 = object_->getVertex(vertices_[1]);

  v1.setModel(v1.getModel() + v);
  v2.setModel(v2.getModel() + v);
}

// duplicate edge and move along normal
CGeomFace3D *
CGeomEdge3D::
extrude(double d) const
{
  std::vector<uint> inds;

  auto normal = calcNormal();

  // add new vertices
  for (auto &vind : vertices_) {
    auto *vertex = object_->getVertexP(vind);

    auto ind = object_->addVertex(vertex->getModel() + d*normal);

    object_->setVertexNormal(ind, normal);

    inds.push_back(ind);
  }

  // complete face
  inds.push_back(vertices_[1]);
  inds.push_back(vertices_[0]);

  auto faceId = object_->addFace(inds);

  return object_->getFaceP(faceId);
}

CVector3D
CGeomEdge3D::
calcNormal() const
{
  auto v1 = CVector3D(object_->getVertexP(vertices_[0])->getModel(),
                      object_->getVertexP(vertices_[1])->getModel()).normalized();

  if      (std::abs(v1.x()) > std::abs(v1.y()) && std::abs(v1.x()) > std::abs(v1.z()))
    return v1.crossProduct(CVector3D(0, 1, 0));
  else if (std::abs(v1.y()) > std::abs(v1.x()) && std::abs(v1.y()) > std::abs(v1.z()))
    return v1.crossProduct(CVector3D(0, 0, 1));
  else if (std::abs(v1.z()) > std::abs(v1.z()) && std::abs(v1.z()) > std::abs(v1.y()))
    return v1.crossProduct(CVector3D(1, 0, 0));
  else
    return CVector3D(0, 1, 0);
}

double
CGeomEdge3D::
distanceTo(const CPoint3D &p) const
{
  std::vector<CPoint3D> points;

  for (const auto &vertex : vertices_) {
    const auto &p1 = object_->getVertex(vertex).getModel();

    points.push_back(p1);
  }

  CLine3D line(points[0], points[1]);

  double dist;
  if (! line.pointDistance(p, &dist))
    dist = 1E50;

  return dist;
}
