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
  auto *v1 = object_->getVertexP(vertices_[0]);
  auto *v2 = object_->getVertexP(vertices_[1]);

  v1->setModel(v1->getModel() + v);
  v2->setModel(v2->getModel() + v);
}

void
CGeomEdge3D::
scale(double s)
{
  auto *v1 = object_->getVertexP(vertices_[0]);
  auto *v2 = object_->getVertexP(vertices_[1]);

  auto c = calcCenter();

  auto cv1 = CVector3D(c, v1->getModel())*s;
  auto cv2 = CVector3D(c, v2->getModel())*s;

  v1->setModel(cv1.point() + c);
  v2->setModel(cv2.point() + c);
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

CGeomFace3D *
CGeomEdge3D::
bevel(double d)
{
  // get edge faces
  auto faces = object_->getEdgeFaces(this);
  if (faces.size() != 2) return nullptr;

  auto *face1 = faces[0];
  auto *face2 = faces[1];

  if (! face1->checkModelOrientation()) std::cerr << "Bad face1 orient\n";
  if (! face2->checkModelOrientation()) std::cerr << "Bad face2 orient\n";

  // get each face/edge perp
  auto fv1 = face1->edgeVector(this);
  auto fv2 = face2->edgeVector(this);

  //---

  // move existing vertices along first face
  auto *v1 = object_->getVertexP(vertices_[0]);
  auto *v2 = object_->getVertexP(vertices_[1]);

  auto pv1 = v1->getModel();
  auto pv2 = v2->getModel();

  v1->setModel(pv1 + fv1*d);
  v2->setModel(pv2 + fv1*d);

  //---

  // add two new vertices along other face
  auto ind1 = object_->addVertex(pv1 + fv2*d);
  auto ind2 = object_->addVertex(pv2 + fv2*d);

  auto *v3 = object_->getVertexP(ind1);
  auto *v4 = object_->getVertexP(ind2);

  //---

  // add new taper face
  std::vector<uint> inds;

  inds.push_back(vertices_[0]);
  inds.push_back(vertices_[1]);
  inds.push_back(ind2);
  inds.push_back(ind1);

  auto faceId = object_->addFace(inds);

  auto *taperFace = object_->getFaceP(faceId);

  if (! taperFace->checkModelOrientation()) std::cerr << "Bad taperFace orient\n";

  //---

  // replace other face vertices
  auto vertices2 = face2->getVertices();

  for (uint i2 = 0; i2 < vertices2.size(); ++i2) {
    if      (vertices2[i2] == vertices_[0])
      vertices2[i2] = ind1;
    else if (vertices2[i2] == vertices_[1])
      vertices2[i2] = ind2;
  }

  face2->setVertices(vertices2);

  //---

  // add new point to other (non-edge) faces
  struct FaceData {
    CGeomFace3D*               face             { nullptr };
    CGeomFace3D::VertexList    vertices;
    bool                       hasNormals       { false };
    bool                       hasTexturePoints { false };
    CGeomFace3D::Normals       normals;
    CGeomFace3D::TexturePoints texturePoints;

    FaceData() { }

    FaceData(CGeomFace3D *face1) {
      face = face1;

      hasNormals       = face->hasVertexNormals();
      hasTexturePoints = face->hasVertexTexturePoints();

      vertices      = face->getVertices();
      normals       = face->getVertexNormals();
      texturePoints = face->getTexturePoints();
    }

    void addData(const FaceData &data, uint i) {
      vertices.push_back(data.vertices[i]);

      if (hasNormals)
        normals.push_back(data.normals[i]);

      if (hasTexturePoints)
        texturePoints.push_back(data.texturePoints[i]);
    }

    void addData(uint vind, const CVector3D &normal, const CPoint2D &texturePoint) {
      vertices.push_back(vind);

      if (hasNormals)
        normals.push_back(normal);

      if (hasTexturePoints)
        texturePoints.push_back(texturePoint);
    }

    void initFace() {
      face->setVertices(vertices);

      if (hasNormals)
        face->setVertexNormals(normals);

      if (hasTexturePoints)
        face->setTexturePoints(texturePoints);
    }
  };

  // v3 is new vertex on v1 faces, v1 is old (moved) vertex on v1 faces
  auto faces1 = object_->getVertexFaces(v1);

  for (auto *face11 : faces1) {
    // skip edge faces
    if (face11 == face1 || face11 == face2)
      continue;

    FaceData faceData11(face11);

    FaceData faceData12;

    faceData12.face             = face11;
    faceData12.hasNormals       = faceData11.hasNormals;
    faceData12.hasTexturePoints = faceData11.hasTexturePoints;

    auto n1 = faceData11.vertices.size();

    uint i11 = 0;

    for (uint i1 = 0; i1 < n1; ++i1) {
      auto v11 = faceData11.vertices[i1];

      if (v11 == v1->getInd()) {
        i11 = i1;

        CVector3D normal;
        face11->calcModelNormal(normal);

        CPoint2D texturePoint(0, 0);

        faceData12.addData(v3->getInd(), normal, texturePoint);
      }

      faceData12.addData(faceData11, i1);
    }

    faceData12.initFace();

    if (! face11->checkModelOrientation()) {
      std::cerr << "Bad face11 orient\n";

      auto i12 = i11 + 1;

      std::swap(faceData12.vertices[i11], faceData12.vertices[i12]);

      if (faceData12.hasNormals)
        std::swap(faceData12.normals[i11], faceData12.normals[i12]);

      if (faceData12.hasTexturePoints)
        std::swap(faceData12.texturePoints[i11], faceData12.texturePoints[i12]);

      faceData12.initFace();

      if (! face11->checkModelOrientation())
        std::cerr << "Bad face11 orient\n";
    }
  }

  // v4 is new vertex on v2 faces, v2 is old (moved) vertex on v2 faces
  auto faces2 = object_->getVertexFaces(v2);

  for (auto *face21 : faces2) {
    // skip edge faces
    if (face21 == face1 || face21 == face2)
      continue;

    FaceData faceData21(face21);

    FaceData faceData22;

    faceData22.face             = face21;
    faceData22.hasNormals       = faceData21.hasNormals;
    faceData22.hasTexturePoints = faceData21.hasTexturePoints;

    auto n2 = faceData21.vertices.size();

    uint i11 = 0;

    for (uint i2 = 0; i2 < n2; ++i2) {
      auto v21 = faceData21.vertices[i2];

      if (v21 == v2->getInd()) {
        i11 = i2;

        CVector3D normal;
        face21->calcModelNormal(normal);

        CPoint2D texturePoint(0, 0);

        faceData22.addData(v4->getInd(), normal, texturePoint);
      }

      faceData22.addData(faceData21, i2);
    }

    faceData22.initFace();

    if (! face21->checkModelOrientation()) {
      std::cerr << "Bad face21 orient\n";

      auto i12 = i11 + 1;

      std::swap(faceData22.vertices[i11], faceData22.vertices[i12]);

      if (faceData22.hasNormals)
        std::swap(faceData22.normals[i11], faceData22.normals[i12]);

      if (faceData22.hasTexturePoints)
        std::swap(faceData22.texturePoints[i11], faceData22.texturePoints[i12]);

      faceData22.initFace();

      if (! face21->checkModelOrientation())
        std::cerr << "Bad face21 orient\n";
    }
  }

  return taperFace;
}

CVector3D
CGeomEdge3D::
vector() const
{
  return CVector3D(object_->getVertexP(vertices_[0])->getModel(),
                   object_->getVertexP(vertices_[1])->getModel()).normalized();
}

CVector3D
CGeomEdge3D::
calcNormal() const
{
  auto v1 = vector();

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

CPoint3D
CGeomEdge3D::
calcCenter() const
{
  auto *v1 = object_->getVertexP(vertices_[0]);
  auto *v2 = object_->getVertexP(vertices_[1]);

  return (v1->getModel() + v2->getModel())/2.0;
}
