#include <CGeomVertex3D.h>
#include <CGeomEdge3D.h>
#include <CGeomZBuffer.h>
#include <CGeomCamera3D.h>

CGeomVertex3D::
CGeomVertex3D(CGeomObject3D *pobject, const CPoint3D &point) :
 CGeomPoint3D(point), pobject_(pobject)
{
  setVisible(true);
}

CGeomVertex3D *
CGeomVertex3D::
dup() const
{
  return new CGeomVertex3D(*this);
}

void
CGeomVertex3D::
draw(CGeomZBuffer *zbuffer)
{
  zbuffer->getRenderer()->drawPoint(CIPoint2D(int(pixel_.x), int(pixel_.y)));
}

//---

void
CGeomVertex3D::
setJointData(const JointData &data)
{
  assert(! jointData_.set);

  jointData_     = data;
  jointData_.set = true;

  pobject_->setJointed(true);
}

//---

void
CGeomVertex3D::
swapXY()
{
  if (normal_)
    normal_ = CVector3D(normal_->y(), normal_->x(), normal_->z());

  CGeomPoint3D::swapXY();
}

void
CGeomVertex3D::
swapYZ()
{
  if (normal_)
    normal_ = CVector3D(normal_->x(), normal_->z(), normal_->y());

  CGeomPoint3D::swapYZ();
}

void
CGeomVertex3D::
swapZX()
{
  if (normal_)
    normal_ = CVector3D(normal_->z(), normal_->y(), normal_->x());

  CGeomPoint3D::swapZX();
}

void
CGeomVertex3D::
invertX()
{
  if (normal_)
    normal_ = CVector3D(-normal_->x(), normal_->y(), normal_->z());

  CGeomPoint3D::invertX();
}

void
CGeomVertex3D::
invertY()
{
  if (normal_)
    normal_ = CVector3D(normal_->x(), -normal_->y(), normal_->z());

  CGeomPoint3D::invertY();
}

void
CGeomVertex3D::
invertZ()
{
  if (normal_)
    normal_ = CVector3D(normal_->x(), normal_->y(), -normal_->z());

  CGeomPoint3D::invertZ();
}

void
CGeomVertex3D::
flipX(double x)
{
  if (normal_)
    normal_ = CVector3D(-normal_->x(), normal_->y(), normal_->z());

  CGeomPoint3D::flipX(x);
}

void
CGeomVertex3D::
flipY(double y)
{
  if (normal_)
    normal_ = CVector3D(normal_->x(), -normal_->y(), normal_->z());

  CGeomPoint3D::flipY(y);
}

void
CGeomVertex3D::
flipZ(double z)
{
  if (normal_)
    normal_ = CVector3D(normal_->x(), normal_->y(), -normal_->z());

  CGeomPoint3D::flipZ(z);
}

//---

CGeomFace3D *
CGeomVertex3D::
bevel(double d)
{
  auto p = pobject_->getVertexP(getInd())->getModel();

  auto edges = pobject_->getVertexEdges(this);

  std::vector<uint>             vinds;
  std::map<CGeomEdge3D *, uint> edgeInd;

  for (auto *edge : edges) {
    auto vind1 = edge->getOtherVertex(getInd());

    auto p1 = pobject_->getVertexP(vind1)->getModel();

    auto p2 = p + (p1 - p)*d;

    auto vind2 = pobject_->addVertex(p2);

    vinds.push_back(vind2);

    edgeInd[edge] = vind2;
  }

  auto faces = pobject_->getVertexFaces(this);

  for (auto *face : faces) {
    const auto &vertices = face->getVertices();

    auto nv = vertices.size();

    std::vector<uint> vertices1;

    for (uint i1 = 0; i1 < nv; ++i1) {
      auto i2 = i1 + 1; if (i2 >= nv) i2 = 0;

      if (vertices[i1] != getInd())
        vertices1.push_back(vertices[i1]);

      for (const auto &pe : edgeInd) {
        if (pe.first->isEdgeInds(vertices[i1], vertices[i2]))
          vertices1.push_back(pe.second);
      }
    }

    face->setVertices(vertices1);
  }

  auto faceInd = pobject_->addFace(vinds);

  auto *face = pobject_->getFaceP(faceInd);

  face->fixNormal();

  return face;
}
