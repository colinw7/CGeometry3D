#include <CGeomEdge3D.h>
#include <CGeomObject3D.h>

#include <CMathGeom3D.h>
#include <CLine3D.h>

#include <set>

namespace {
  uint nextInd(uint i, uint n) {
    ++i;

    if (i >= n)
      i = 0;

    return i;
  }

  uint prevInd(uint i, uint n) {
    if (i == 0)
      i = n;

    --i;

    return i;
  }
}

//---

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

  auto c = calcModelCenter();

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
  Inds inds;

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

  CVector3D n1;
  face1->calcModelNormal(n1);

  CVector3D n2;
  face1->calcModelNormal(n2);

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
  Inds inds;

  inds.push_back(vertices_[0]);
  inds.push_back(vertices_[1]);
  inds.push_back(ind2);
  inds.push_back(ind1);

  auto faceId = object_->addFace(inds);

  auto *taperFace = object_->getFaceP(faceId);

  if (! taperFace->checkModelOrientation()) std::cerr << "Bad taperFace orient\n";

  taperFace->setNormal((n1 + n2)/2);

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
    CGeomFace3D*            face { nullptr };
    CGeomFace3D::VertexData vertexData;

    FaceData() { }

    FaceData(CGeomFace3D *face1) {
      face = face1;

      face->getVertexData(vertexData);
    }

    void addData(const FaceData &data, uint i) {
      vertexData.vertices.push_back(data.vertexData.vertices[i]);

      if (vertexData.hasNormals)
        vertexData.normals.push_back(data.vertexData.normals[i]);

      if (vertexData.hasTexturePoints)
        vertexData.texturePoints.push_back(data.vertexData.texturePoints[i]);
    }

    void addData(uint vind, const CVector3D &normal, const CPoint2D &texturePoint) {
      vertexData.vertices.push_back(vind);

      if (vertexData.hasNormals)
        vertexData.normals.push_back(normal);

      if (vertexData.hasTexturePoints)
        vertexData.texturePoints.push_back(texturePoint);
    }

    void initFace() {
      face->setVertices(vertexData.vertices);

      if (vertexData.hasNormals)
        face->setVertexNormals(vertexData.normals);

      if (vertexData.hasTexturePoints)
        face->setTexturePoints(vertexData.texturePoints);
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

    faceData12.face                        = face11;
    faceData12.vertexData.hasNormals       = faceData11.vertexData.hasNormals;
    faceData12.vertexData.hasTexturePoints = faceData11.vertexData.hasTexturePoints;

    uint i11 = 0;

    for (uint i1 = 0; i1 < faceData11.vertexData.vertices.size(); ++i1) {
      auto v11 = faceData11.vertexData.vertices[i1];

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

      std::swap(faceData12.vertexData.vertices[i11], faceData12.vertexData.vertices[i12]);

      if (faceData12.vertexData.hasNormals)
        std::swap(faceData12.vertexData.normals[i11], faceData12.vertexData.normals[i12]);

      if (faceData12.vertexData.hasTexturePoints)
        std::swap(faceData12.vertexData.texturePoints[i11],
                  faceData12.vertexData.texturePoints[i12]);

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

    faceData22.face                        = face21;
    faceData22.vertexData.hasNormals       = faceData21.vertexData.hasNormals;
    faceData22.vertexData.hasTexturePoints = faceData21.vertexData.hasTexturePoints;

    uint i11 = 0;

    for (uint i2 = 0; i2 < faceData21.vertexData.vertices.size(); ++i2) {
      auto v21 = faceData21.vertexData.vertices[i2];

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

      std::swap(faceData22.vertexData.vertices[i11], faceData22.vertexData.vertices[i12]);

      if (faceData22.vertexData.hasNormals)
        std::swap(faceData22.vertexData.normals[i11], faceData22.vertexData.normals[i12]);

      if (faceData22.vertexData.hasTexturePoints)
        std::swap(faceData22.vertexData.texturePoints[i11],
                  faceData22.vertexData.texturePoints[i12]);

      faceData22.initFace();

      if (! face21->checkModelOrientation())
        std::cerr << "Bad face21 orient\n";
    }
  }

  return taperFace;
}

// cut N times at factor
std::vector<CGeomEdge3D::Inds>
CGeomEdge3D::
loopCut(uint n) const
{
  assert(n > 0);

  std::vector<Inds> faceIds;

  faceIds.resize(n + 1);

  std::set<CGeomEdge3D *> processedEdges;
  std::set<CGeomFace3D *> processedFaces;

  auto *th = const_cast<CGeomEdge3D *>(this);

  //---

  struct EdgePoint {
    CGeomEdge3D*                 edge   { nullptr };
    std::vector<CPoint3D>        points;
    std::vector<CGeomVertex3D *> vertices;
  };

  using EdgePointMap   = std::map<CGeomEdge3D *, EdgePoint *>;
  using EdgePoints     = std::vector<EdgePoint *>;
  using FaceEdgePoints = std::map<CGeomFace3D *, EdgePoints>;

  FaceEdgePoints faceEdgePoints;

  //---

  // calc cut factor
  double f = 1.0/(n + 1);

  //---

  // start from first edge face (either will do)
  auto faces = object_->getEdgeFaces(this);
  if (faces.empty()) return faceIds;

  auto *face = faces[0];

  //---

  EdgePointMap edgePointMap;

  auto getEdgePoint = [&](CGeomEdge3D *edge) {
    auto pe = edgePointMap.find(edge);

    if (pe == edgePointMap.end()) {
      auto edgePoint = new EdgePoint;

      edgePoint->edge = edge;

      pe = edgePointMap.insert(pe, EdgePointMap::value_type(edge, edgePoint));
    }

    return (*pe).second;
  };

  //---

  // add first edge points (use face ordered edge start/end)
  auto *edgePoint = getEdgePoint(th);

  auto pfe1 = face->edgeStart(this);
  auto pfe2 = face->edgeEnd  (this);

  for (uint i = 1; i <= n; ++i) {
    auto p1 = pfe1 + f*i*(pfe2 - pfe1);

    edgePoint->points.push_back(p1);
  }

  // create new start edge vertices
  for (const auto &pe : edgePoint->points) {
    auto vind = object_->addVertex(pe);

    //std::cerr << "Vertex " << vind << ": " << object_->getVertexP(vind)->getModel() << "\n";

    edgePoint->vertices.push_back(object_->getVertexP(vind));
  }

  processedEdges.insert(th);

  faceEdgePoints[face].push_back(edgePoint); // first face edge

  processedFaces.insert(face);

  auto *startEdgePoint = edgePoint;

  //---

  while (true) {
    // calc perp line on current edge (first point)
    auto edgeVector = face->edgeVector(edgePoint->edge);

    auto p1 = edgePoint->points[0];

    CLine3D line(p1, edgeVector);

    // intersect with other face edges
    bool found = false;

    const auto &edges = face->getEdges();

    for (auto *edge : edges) {
      // skip if processed
      auto pe = processedEdges.find(edge);
      if (pe != processedEdges.end())
        continue;

      //---

      // check intersect perp line with edge line
      auto line1 = edge->modelLine();

      CPoint3D pa, pb;
      double t1, t2;
      if (! CMathGeom3D::LineLineIntersect(line.start(), line.end(), line1.start(), line1.end(),
                                           pa, pb, t1, t2))
        continue;

      // we have intersect so this is next edge on loop

      //---

      // calc intersect line points for next edge
      auto *edgePoint1 = getEdgePoint(edge);

      edgePoint1->points.push_back(pa); // interset from edgePoint->points[0]

      for (uint ip = 1; ip < n; ++ip) {
        // intersect other loop points (should always hit)
        auto ip1 = edgePoint->points[ip];

        CLine3D iline(ip1, edgeVector); // perp edge at cut

        CPoint3D pa1, pb1;
        if (CMathGeom3D::LineLineIntersect(iline.start(), iline.end(), line1.start(), line1.end(),
                                           pa1, pb1, t1, t2))
          pa = pa1;

        edgePoint1->points.push_back(pa);
      }

      // create new edge vertices
      for (const auto &pe1 : edgePoint1->points) {
        auto vind = object_->addVertex(pe1);

        //std::cerr << "Vertex " << vind << ": " << object_->getVertexP(vind)->getModel() << "\n";

        edgePoint1->vertices.push_back(object_->getVertexP(vind));
      }

      processedEdges.insert(edge);

      faceEdgePoints[face].push_back(edgePoint1); // second face edge

      edgePoint = edgePoint1; // update start edge point

      found = true;

      break;
    }

    if (! found)
      break;

    // get next unused face for start edge point
    face = nullptr;

    auto faces1 = object_->getEdgeFaces(edgePoint->edge);

    for (auto *face1 : faces1) {
      auto pf = processedFaces.find(face1);
      if (pf != processedFaces.end())
        continue;

      face = face1;

      break;
    }

    if (! face)
      break;

    faceEdgePoints[face].push_back(edgePoint); // first face edge

    processedFaces.insert(face);
  }

  // loop complete so use first edge point to terminate
  if (face)
    faceEdgePoints[face].push_back(startEdgePoint); // second face edge

  //---

  // split each face at cut points
  for (auto &pf : faceEdgePoints) {
    auto *face1 = pf.first;

    //std::cerr << "Face: " << face1->getInd() << "\n";

    // get edge vertices (first and second edge)
    assert(pf.second.size() == 2);

    auto *edgeData1 = pf.second[0];
    auto *edgeData2 = pf.second[1];

    Inds einds1, einds2;

    for (auto *vertex : edgeData1->vertices)
      einds1.push_back(vertex->getInd());

    for (auto *vertex : edgeData2->vertices)
      einds2.push_back(vertex->getInd());

    auto ni = einds1.size();
    assert(ni == einds2.size());

    // update existing face and add new face
    auto vertices = face1->getVertices();

    auto nv = vertices.size();

    std::vector<Inds> inds1;

    inds1.resize(n + 1);

    //---

    uint ii = 0;

    for (uint i1 = 0; i1 < nv; ++i1) {
      uint i2 = nextInd(i1, uint(nv));

      inds1[ii].push_back(vertices[i1]);

      // at first edge
      if      (edgeData1->edge->isEdgeInds(vertices[i1], vertices[i2])) {
        for (uint ii1 = 0; ii1 < ni; ++ii1) {
          inds1[ii].push_back(einds1[ii1]);

          ii = nextInd(ii, n + 1);

          inds1[ii].push_back(einds1[ii1]);
        }
      }
      // at second edge
      else if (edgeData2->edge->isEdgeInds(vertices[i1], vertices[i2])) {
        for (uint ii1 = 0; ii1 < ni; ++ii1) {
          auto ii2 = ni - 1 - ii1;

          inds1[ii].push_back(einds2[ii2]);

          ii = prevInd(ii, n + 1);

          inds1[ii].push_back(einds2[ii2]);
        }
      }
    }

#if 0
    auto printInds = [](const Inds &pinds) {
      for (auto &ind : pinds)
        std::cerr << " " << ind;
      std::cerr << "\n";
    };

    std::cerr << "Face: " << face1->getInd() << ":";
    printInds(vertices);

    for (uint i = 0; i <= n; ++i) {
      std::cerr << "Inds " << i << ":";
      printInds(inds1[i]);
    }
#endif

    auto faceDistance = [&](CGeomFace3D *dface) {
      auto p = dface->edgeStart(edgeData1->edge);

      return p.distanceTo(dface->calcModelCenter());
    };

    std::map<double, std::vector<CGeomFace3D *>> faces1;

    face1->setVertices(inds1[0]);

    faces1[faceDistance(face1)].push_back(face1);

    for (uint i = 1; i <= n; ++i) {
      auto *face2 = face1->dup();

      face2->setVertices(inds1[i]);

      object_->addFace(face2);

      faces1[faceDistance(face2)].push_back(face2);
    }

    uint ii1 = 0;

    for (const auto &pf1 : faces1)
      for (auto *face2 : pf1.second)
        faceIds[ii1++].push_back(face2->getInd());
  }

  for (auto &pe : edgePointMap)
    delete pe.second;

  return faceIds;
}

bool
CGeomEdge3D::
isEdgeInds(uint ind1, uint ind2) const
{
  return ((ind1 == vertices_[0] && ind2 == vertices_[1]) ||
          (ind1 == vertices_[1] && ind2 == vertices_[0]));
}

CLine3D
CGeomEdge3D::
modelLine() const
{
  return CLine3D(modelStart(), modelEnd());
}

CVector3D
CGeomEdge3D::
vector() const
{
  return modelLine().vector();
}

CPoint3D
CGeomEdge3D::
modelStart() const
{
  return object_->getVertexP(vertices_[0])->getModel();
}

CPoint3D
CGeomEdge3D::
modelEnd() const
{
  return object_->getVertexP(vertices_[1])->getModel();
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
calcProjectedCenter() const
{
  auto *v1 = object_->getVertexP(vertices_[0]);
  auto *v2 = object_->getVertexP(vertices_[1]);

  return (v1->getProjected() + v2->getProjected())/2.0;
}

CPoint3D
CGeomEdge3D::
calcModelCenter() const
{
  return modelLinePoint(0.5);
}

CPoint3D
CGeomEdge3D::
modelLinePoint(double f) const
{
  auto v1 = object_->getVertexP(vertices_[0])->getModel();
  auto v2 = object_->getVertexP(vertices_[1])->getModel();

  return v1 + f*(v2 - v1);
}
