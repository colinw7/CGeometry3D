#include <CGeometry3D.h>
#include <CGeomEdge3D.h>
#include <CGeomZBuffer.h>
#include <CGeomLight3D.h>
#include <CGeomUtil3D.h>
#include <CMathRound.h>
#include <CPlane3D.h>

CGeomFace3D::
CGeomFace3D(CGeomObject3D *pobject) :
 pobject_(pobject)
{
  init();
}

CGeomFace3D::
CGeomFace3D(CGeomObject3D *pobject, const VertexList &vertices) :
 pobject_(pobject), vertices_(vertices)
{
  init();
}

CGeomFace3D::
CGeomFace3D(const CGeomFace3D &face) :
 pobject_      (face.pobject_),
 groupId_      (face.groupId_),
 flags_        (face.flags_),
 vertices_     (face.vertices_),
 normals_      (face.normals_),
 texturePoints_(face.texturePoints_),
 normal_       (face.normal_)
{
  init();

  //---

  // copy sub faces and sub lines
  for (auto *subFace : face.subFaces_) {
    auto *face1 = subFace->dup();

    addSubFace(face1);
  }

  for (auto *subLine : face.subLines_) {
    auto *line = subLine->dup();

    addSubLine(line);
  }

  //---

  // copy materials
  if (face.frontMaterial_)
    setFrontMaterial(*face.frontMaterial_);

  if (face.backMaterial_)
    setBackMaterial(*face.backMaterial_);

  materialP_ = face.materialP_;

  //---

  // copy textures (TODO: share)
  if (face.diffuseTexture_)
    diffuseTexture_ = face.diffuseTexture_->dup();

  if (face.specularTexture_)
    specularTexture_ = face.specularTexture_->dup();

  if (face.normalTexture_)
    normalTexture_ = face.normalTexture_->dup();

  if (face.emissiveTexture_)
    emissiveTexture_ = face.emissiveTexture_->dup();

  //---

  // copy mask
  if (face.mask_)
    mask_ = face.mask_->dup();
}

void
CGeomFace3D::
init()
{
  setVisible(true);
}

CGeomFace3D *
CGeomFace3D::
dup() const
{
  return new CGeomFace3D(*this);
}

//---

CGeomMaterial *
CGeomFace3D::
initFrontMaterial() const
{
  auto *th = const_cast<CGeomFace3D *>(this);

  if (! th->frontMaterial_)
    th->frontMaterial_ = CGeometry3DInst->createMaterial();

  return th->frontMaterial_;
}

CGeomMaterial *
CGeomFace3D::
initBackMaterial() const
{
  auto *th = const_cast<CGeomFace3D *>(this);

  if (! th->backMaterial_)
    th->backMaterial_ = CGeometry3DInst->createMaterial();

  return th->backMaterial_;
}

//---

CGeomScene3D *
CGeomFace3D::
getScene() const
{
  if (pobject_)
    return pobject_->getScene();

  return nullptr;
}

void
CGeomFace3D::
setObject(CGeomObject3D *object)
{
  pobject_ = object;

  for (auto *subFace : subFaces_)
    subFace->setObject(object);

  for (auto *subLine : subLines_)
    subLine->setObject(object);
}

void
CGeomFace3D::
setTexture(CImagePtr image)
{
  setDiffuseTexture(image);
}

void
CGeomFace3D::
setTexture(CGeomTexture *texture)
{
  setDiffuseTexture(texture);
}

void
CGeomFace3D::
setTextureMapping(const std::vector<CPoint2D> &points)
{
  if (diffuseTexture_)
    diffuseTexture_->setMapping(points);
}

void
CGeomFace3D::
setTexturePoints(const std::vector<CPoint2D> &points)
{
  texturePoints_ = points;
}

CPoint2D
CGeomFace3D::
getTexturePoint(const CGeomVertex3D &v, int iv) const
{
  auto nt = texturePoints_.size();

  if (iv < 0 || iv >= int(nt))
    return v.getTextureMap();

  return texturePoints_[iv];
}

void
CGeomFace3D::
setDiffuseTexture(CImagePtr image)
{
  setDiffuseTexture(CGeometry3DInst->createTexture(image));
}

void
CGeomFace3D::
setDiffuseTexture(CGeomTexture *texture)
{
  diffuseTexture_ = texture;
}

void
CGeomFace3D::
setSpecularTexture(CImagePtr image)
{
  setSpecularTexture(CGeometry3DInst->createTexture(image));
}

void
CGeomFace3D::
setSpecularTexture(CGeomTexture *texture)
{
  specularTexture_ = texture;
}

void
CGeomFace3D::
setNormalTexture(CGeomTexture *texture)
{
  normalTexture_ = texture;
}

void
CGeomFace3D::
setEmissiveTexture(CGeomTexture *texture)
{
  emissiveTexture_ = texture;
}

//---

void
CGeomFace3D::
setMask(CImagePtr image)
{
  delete mask_;

  mask_ = CGeometry3DInst->createMask(image);
}

void
CGeomFace3D::
setMaskMapping(const std::vector<CPoint2D> &points)
{
  if (mask_)
    mask_->setMapping(points);
}

//---

void
CGeomFace3D::
setLighted(bool b)
{
  if (b)
    flags_ |= LIGHTED;
  else
    flags_ &= uint(~LIGHTED);
}

void
CGeomFace3D::
setTwoSided(bool b)
{
  if (b)
    flags_ |= TWO_SIDED;
  else
    flags_ &= uint(~TWO_SIDED);
}

void
CGeomFace3D::
setSelected(bool b)
{
  if (b)
    flags_ |= SELECTED;
  else
    flags_ &= uint(~SELECTED);
}

void
CGeomFace3D::
setVisible(bool b)
{
  if (b)
    flags_ |= VISIBLE;
  else
    flags_ &= uint(~VISIBLE);
}

//---

void
CGeomFace3D::
setVertexNormals(const std::vector<CVector3D> &normals, bool propagate)
{
  auto nn = normals.size();
  assert(nn == vertices_.size());

  normals_ = normals;

  if (propagate) {
    for (uint i = 0; i < nn; ++i) {
      auto &v = pobject_->getVertex(vertices_[i]);

      v.setNormal(normals[i]);
    }
  }
}

//---

void
CGeomFace3D::
addVertex(uint ind)
{
  vertices_.push_back(ind);

  pobject_->addVertexFace(ind, ind_);
}

uint
CGeomFace3D::
addSubFace(const std::vector<uint> &vertices)
{
  auto *face = CGeometry3DInst->createFace3D(pobject_, vertices);

  return addSubFace(face);
}

uint
CGeomFace3D::
addSubFace(CGeomFace3D *face)
{
  face->setObject(getObject());

  subFaces_.push_back(face);

  auto ind = uint(subFaces_.size() - 1);

  face->setInd(ind);

  return ind;
}

uint
CGeomFace3D::
addSubLine(uint start, uint end)
{
  auto *line = CGeometry3DInst->createLine3D(pobject_, start, end);

  return addSubLine(line);
}

uint
CGeomFace3D::
addSubLine(CGeomLine3D *line)
{
  line->setObject(getObject());

  subLines_.push_back(line);

  auto ind = uint(subLines_.size() - 1);

  line->setInd(ind);

  return ind;
}

void
CGeomFace3D::
setSubFaceColor(const CRGBA &rgba)
{
  for (auto *subFace : subFaces_)
    subFace->setColor(rgba);
}

void
CGeomFace3D::
setSubFaceColor(uint ind, const CRGBA &rgba)
{
  subFaces_[ind]->setColor(rgba);
}

void
CGeomFace3D::
setSubFaceMaterialP(CGeomMaterial *material)
{
  for (auto *subFace : subFaces_)
    subFace->setMaterialP(material);
}

void
CGeomFace3D::
setSubLineColor(uint ind, const CRGBA &rgba)
{
  subLines_[ind]->setColor(rgba);
}

//---

bool
CGeomFace3D::
edgesValid() const
{
  return pobject_->edgesValid();
}

const CGeomFace3D::EdgeList &
CGeomFace3D::
getEdges() const
{
  auto *th = const_cast<CGeomFace3D *>(this);

  // TODO: cache ?
  th->edges_ = pobject_->getFaceEdges(th);

  return th->edges_;
}

CVector3D
CGeomFace3D::
edgeVector(const CGeomEdge3D *edge) const
{
  CVector3D n;
  calcModelNormal(n);

  auto v = n.crossProduct(edge->vector());

  auto p1 = edge->calcCenter() + v*0.001;
  auto p2 = edge->calcCenter() - v*0.001;

  auto c = calcCenter();

  if (c.distanceTo(p2) < c.distanceTo(p1))
    v = -v;

  return v;
}

//---

void
CGeomFace3D::
getModelBBox(CBBox3D &bbox) const
{
  for (const auto &vertex : vertices_) {
    const auto &p = pobject_->getVertex(vertex).getModel();

    bbox += p;
  }
}

double
CGeomFace3D::
distanceTo(const CPoint3D &p) const
{
  std::vector<CPoint3D> points;

  for (const auto &vertex : vertices_) {
    const auto &p1 = pobject_->getVertex(vertex).getModel();

    points.push_back(p1);
  }

  double dist;

  if      (points.size() == 1)
    dist = p.distanceTo(points[0]);
  else if (points.size() == 1) {
    CLine3D line(points[0], points[1]);

    if (! line.pointDistance(p, &dist))
      dist = 1E50;
  }
  else {
    CPlane3D plane(points[0], points[1], points[2]);

    if (! CMathGeom3D::PointPlaneDistance(p, plane, &dist))
      dist = 1E50;
  }

  return dist;
}

//---

void
CGeomFace3D::
drawSolid(CGeom3DRenderer *)
{
  auto num_points = uint(vertices_.size());
  if (num_points < 3) return;

  //------

  //CRGBA rgba;

  // if (! getAdjustedColor(rgba))
  //   return;

  //renderer->setForeground(rgba);

  //------

  //fill(renderer);
}

void
CGeomFace3D::
drawSolid(CGeomZBuffer *zbuffer)
{
  auto num_points = uint(vertices_.size());
  if (num_points < 3) return;

  //------

  CRGBA rgba;

  if (! getAdjustedColor(rgba))
    return;

  zbuffer->setForeground(rgba);

  //------

  fill(zbuffer);
}

void
CGeomFace3D::
drawLines(CGeom3DRenderer *renderer)
{
  if (vertices_.empty())
    return;

  //------

  CRGBA rgba;

  if (! getAdjustedColor(rgba))
    return;

  renderer->setForeground(rgba);

  //------

  auto ppoint1 = pobject_->getVertex(vertices_.back()).getPixel();

  for (const auto &vertex : vertices_) {
    const auto &ppoint2 = pobject_->getVertex(vertex).getPixel();

    renderer->drawLine(CIPoint2D(int(ppoint1.x), int(ppoint1.y)),
                       CIPoint2D(int(ppoint2.x), int(ppoint2.y)));

    ppoint1 = ppoint2;
  }
}

void
CGeomFace3D::
drawLines(CGeomZBuffer *zbuffer)
{
  if (vertices_.empty())
    return;

  //------

  CRGBA rgba;

  if (! getAdjustedColor(rgba))
    return;

  zbuffer->setForeground(rgba);

  //------

  auto ppoint1 = pobject_->getVertex(vertices_.back()).getPixel();

  for (const auto &vertex : vertices_) {
    const auto &ppoint2 = pobject_->getVertex(vertex).getPixel();

    zbuffer->drawOverlayZLine(int(ppoint1.x), int(ppoint1.y), int(ppoint2.x), int(ppoint2.y));

    ppoint1 = ppoint2;
  }
}

void
CGeomFace3D::
fill(CGeomZBuffer *zbuffer)
{
  double color_factor;

  if (! getColorFactor(&color_factor))
    color_factor = 1;

  auto w = zbuffer->getWidth ();
  auto h = zbuffer->getHeight();

  auto num_points = uint(vertices_.size());

  // get y limits
  const auto &pixel = pobject_->getVertex(vertices_[0]).getPixel();

  int ypmin = CMathRound::RoundUp  (pixel.y);
  int ypmax = CMathRound::RoundDown(pixel.y);

  for (uint i1 = 1; i1 < num_points; ++i1) {
    const auto &pixel1 = pobject_->getVertex(vertices_[i1]).getPixel();

    ypmin = std::min(ypmin, CMathRound::RoundUp  (pixel1.y));
    ypmax = std::max(ypmax, CMathRound::RoundDown(pixel1.y));
  }

  int    i1min, i2min, i1max, i2max;
  double zmin, zmax;

  ypmin = std::max(ypmin, 0);
  ypmax = std::min(ypmax, int(h - 1));

  for (int yp = ypmin; yp <= ypmax; ++yp) {
    bool set = false;

    int xpmin, xpmax;

    int xp;

    for (uint i1 = num_points - 1, i2 = 0; i2 < num_points; i1 = i2, ++i2) {
      const auto &pixel1 = pobject_->getVertex(vertices_[i1]).getPixel();
      const auto &pixel2 = pobject_->getVertex(vertices_[i2]).getPixel();

      // skip line if not in y range
      if ((pixel1.y < yp && pixel2.y < yp) ||
          (pixel1.y > yp && pixel2.y > yp) ||
          fabs(pixel2.y - pixel1.y) < 1E-6)
        continue;

      // get x intersect
      double fx = (pixel2.x - pixel1.x)/(pixel2.y - pixel1.y);

      xp = CMathRound::Round((yp - pixel1.y)*fx + pixel1.x);

      // update min and/or max
      if (! set) {
        xpmin = xp;
        i1min = int(i1);
        i2min = int(i2);

        xpmax = xp;
        i1max = int(i1);
        i2max = int(i2);

        set = true;
      }
      else {
        if (xp < xpmin) {
          xpmin = xp;
          i1min = int(i1);
          i2min = int(i2);
        }

        if (xp > xpmax) {
          xpmax = xp;
          i1max = int(i1);
          i2max = int(i2);
        }
      }
    }

    //-------

    if (! set)
      continue;

    //-------

    CPoint2D tmin, tmax;
    CPoint2D mmin, mmax;

    double d, i;

    // get min z intersect (viewed)
    const auto &pixel1 = pobject_->getVertex(vertices_[uint(i1min)]).getPixel();
    const auto &pixel2 = pobject_->getVertex(vertices_[uint(i2min)]).getPixel();

    if (fabs(pixel2.x - pixel1.x) > fabs(pixel2.y - pixel1.y)) {
      d = xpmin - pixel1.x;
      i = 1.0/(pixel2.x - pixel1.x);
    }
    else {
      d = yp - pixel1.y;
      i = 1.0/(pixel2.y - pixel1.y);
    }

    zmin = (pixel2.z - pixel1.z)*i*d + pixel1.z;

    if (diffuseTexture_)
      tmin = interpTexturePoint(i1min, i2min, d, i);

    if (mask_)
      mmin = interpMaskPoint(i1min, i2min, d, i);

    // get max z intersect (viewed)
    const auto &pixel3 = pobject_->getVertex(vertices_[uint(i1max)]).getPixel();
    const auto &pixel4 = pobject_->getVertex(vertices_[uint(i2max)]).getPixel();

    if (fabs(pixel4.x - pixel3.x) > fabs(pixel4.y - pixel3.y)) {
      d = xpmax - pixel3.x;
      i = 1.0/(pixel4.x - pixel3.x);
    }
    else {
      d = yp - pixel3.y;
      i = 1.0/(pixel4.y - pixel3.y);
    }

    zmax = (pixel4.z - pixel3.z)*i*d + pixel3.z;

    if (diffuseTexture_)
      tmax = interpTexturePoint(i1max, i2max, d, i);

    if (mask_)
      mmax = interpMaskPoint(i1max, i2max, d, i);

    //-------

    double zz, dz = 0;

    CPoint2D t, dt;
    CPoint2D m, dm;

    if (xpmin < xpmax) {
      dz = (zmax - zmin)/(xpmax - xpmin);

      if (diffuseTexture_)
        dt = (tmax - tmin)/(xpmax - xpmin);

      if (mask_)
        dm = (mmax - mmin)/(xpmax - xpmin);
    }

    zz = zmin;
    t  = tmin;
    m  = mmin;

    for (int xp1 = xpmin; xp1 <= xpmax; ++xp1, zz += dz) {
      if (xp1 < 0 || xp1 >= int(w)) {
        if (diffuseTexture_)
          t += dt;

        if (mask_)
          m += dm;
      }
      else {
        if (diffuseTexture_) {
          CRGBA rgba = diffuseTexture_->getImageRGBA(uint(t.x), uint(t.y));

          t += dt;

          zbuffer->setForeground(rgba*color_factor);
        }

        if (mask_) {
          bool set1 = mask_->getImageSet(uint(m.x), uint(m.y));

          m += dm;

          if (! set1)
            continue;
        }

        zbuffer->drawFacePoint(this, xp1, yp, zz);
      }
    }
  }
}

bool
CGeomFace3D::
getAdjustedColor(CRGBA &rgba)
{
  CPoint3D mid_point;
  getViewedMidPoint(mid_point);

  CVector3D normal;
  calcViewedNormal(normal);

  //------

  if (flags_ & LIGHTED) {
    if (! pobject_ ||
        ! pobject_->lightPoint(mid_point, normal, getFrontMaterial(), rgba)) {
      CVector3D dir(mid_point, CPoint3D(0, 0, 1));

      double factor1 = normal.dotProduct(dir.normalized());

      if (factor1 < 0)
        factor1 = -factor1;
//      return false;

      rgba = getFrontColor();

      rgba.scaleRGB(factor1);
    }
  }
  else {
    CVector3D dir(mid_point, CPoint3D(0, 0, 1));

    double factor1 = normal.dotProduct(dir.normalized());

    if (factor1 < 0) {
      factor1 = -factor1;
//    return false;
    }

    rgba = getFrontColor();

    rgba.scaleRGB(factor1);
  }

  return true;
}

bool
CGeomFace3D::
getColorFactor(double *factor)
{
  CPoint3D mid_point;
  getViewedMidPoint(mid_point);

  CVector3D normal;
  calcViewedNormal(normal);

  CVector3D dir(mid_point, CPoint3D(0, 0, 1));

  *factor = normal.dotProduct(dir.normalized());

  if (*factor < 0) {
    *factor = -(*factor);
//  return false;
  }

  return true;
}

void
CGeomFace3D::
getModelMidPoint(CPoint3D &mid_point) const
{
  mid_point = pobject_->verticesModelMidPoint(vertices_);
}

void
CGeomFace3D::
getViewedMidPoint(CPoint3D &mid_point) const
{
  mid_point = pobject_->verticesViewedMidPoint(vertices_);
}

void
CGeomFace3D::
calcModelNormal(CVector3D &normal) const
{
  normal = pobject_->verticesModelNormal(vertices_);
}

void
CGeomFace3D::
calcViewedNormal(CVector3D &normal) const
{
  normal = pobject_->verticesViewedNormal(vertices_);
}

CPoint2D
CGeomFace3D::
interpTexturePoint(int i1, int i2, double d, double i)
{
  const auto &point1 = diffuseTexture_->getMappingPoint(uint(i1));
  const auto &point2 = diffuseTexture_->getMappingPoint(uint(i2));

  double fx = (point2.x - point1.x)*i;
  double fy = (point2.y - point1.y)*i;

  double x = d*fx + point1.x;
  double y = d*fy + point1.y;

  return CPoint2D(x, y);
}

CPoint2D
CGeomFace3D::
interpMaskPoint(int i1, int i2, double d, double i)
{
  const auto &point1 = mask_->getMappingPoint(uint(i1));
  const auto &point2 = mask_->getMappingPoint(uint(i2));

  double fx = (point2.x - point1.x)*i;
  double fy = (point2.y - point1.y)*i;

  double x = d*fx + point1.x;
  double y = d*fy + point1.y;

  return CPoint2D(x, y);
}

CPolygonOrientation
CGeomFace3D::
orientation() const
{
  return orientationI(0);
}

CPolygonOrientation
CGeomFace3D::
orientationI(uint i1) const
{
  auto nv = vertices_.size();

  auto i2 = i1 + 1; if (i2 >= nv) i2 = 0;
  auto i3 = i2 + 1; if (i3 >= nv) i3 = 0;

  const auto &point1 = pobject_->getVertex(vertices_[i1]).getProjected();
  const auto &point2 = pobject_->getVertex(vertices_[i2]).getProjected();
  const auto &point3 = pobject_->getVertex(vertices_[i3]).getProjected();

  CTriangle3D triangle(point1, point2, point3);

  return triangle.orientationXY();
}

CPolygonOrientation
CGeomFace3D::
modelOrientation() const
{
  return modelOrientationI(0);
}

CPolygonOrientation
CGeomFace3D::
modelOrientationI(uint i1) const
{
  CVector3D n;
  calcModelNormal(n);

  auto q = CQuaternion::rotationArc(n, CVector3D(0, 0, -1));

  CMatrix3D m;
  q.toRotationMatrix(m);

  auto nv = vertices_.size();

  auto i2 = i1 + 1; if (i2 >= nv) i2 = 0;
  auto i3 = i2 + 1; if (i3 >= nv) i3 = 0;

  const auto &p1 = pobject_->getVertex(vertices_[i1]).getModel();
  const auto &p2 = pobject_->getVertex(vertices_[i2]).getModel();
  const auto &p3 = pobject_->getVertex(vertices_[i3]).getModel();

  auto pp1 = m*p1;
  auto pp2 = m*p2;
  auto pp3 = m*p3;

  CTriangle3D triangle(pp1, pp2, pp3);

  return triangle.orientationXY();
}

bool
CGeomFace3D::
checkModelOrientation() const
{
  auto nv = vertices_.size();

  auto orient = modelOrientationI(0);

  for (uint i = 1; i < nv; ++i) {
    auto orient1 = modelOrientationI(i);

    if (orient1 != orient)
      return false;
  }

  return true;
}

void
CGeomFace3D::
moveBy(const CVector3D &v)
{
  for (auto &vertex : vertices_) {
    auto &v1 = pobject_->getVertex(vertex);

    v1.setModel(v1.getModel() + v);
  }
}

void
CGeomFace3D::
scale(double sx, double sy, double sz)
{
  CVector3D s(sx, sy, sz);

  auto c = calcCenter();

  for (auto &vertex : vertices_) {
    auto &v1 = pobject_->getVertex(vertex);

    auto &p = v1.getModel();

    v1.setModel(CPoint3D((p.x - c.x)*s.x() + c.x,
                         (p.y - c.y)*s.y() + c.y,
                         (p.z - c.z)*s.z() + c.z));
  }
}

void
CGeomFace3D::
rotateModelX(double dx)
{
  CMatrix3D m;
  m.setRotation(CMathGen::X_AXIS_3D, dx);

  auto c = calcCenter();

  for (auto &vertex : vertices_) {
    auto &v1 = pobject_->getVertex(vertex);

    auto p = v1.getModel() - c;

    auto p1 = m.multiplyPoint(p) + c;

    v1.setModel(p1);
  }
}

void
CGeomFace3D::
rotateModelY(double dy)
{
  CMatrix3D m;
  m.setRotation(CMathGen::Y_AXIS_3D, dy);

  auto c = calcCenter();

  for (auto &vertex : vertices_) {
    auto &v1 = pobject_->getVertex(vertex);

    auto p = v1.getModel() - c;

    auto p1 = m.multiplyPoint(p) + c;

    v1.setModel(p1);
  }
}

void
CGeomFace3D::
rotateModelZ(double dz)
{
  CMatrix3D m;
  m.setRotation(CMathGen::Z_AXIS_3D, dz);

  auto c = calcCenter();

  for (auto &vertex : vertices_) {
    auto &v1 = pobject_->getVertex(vertex);

    auto p = v1.getModel() - c;

    auto p1 = m.multiplyPoint(p) + c;

    v1.setModel(p1);
  }
}

// convert face to triangles (adds new faces as needed)
void
CGeomFace3D::
triangulate()
{
  auto nv = vertices_.size();
  if (nv <= 3) return;

  bool hasNormals       = hasVertexNormals();
  bool hasTexturePoints = hasVertexTexturePoints();

  uint i1 = 0;

  // assume triangle fan ?
  for (uint i2 = 2; i2 < nv - 1; ++i2) {
    uint i3 = i2 + 1;

    auto *v1 = pobject_->getVertexP(vertices_[i1])->dup();
    auto *v2 = pobject_->getVertexP(vertices_[i2])->dup();
    auto *v3 = pobject_->getVertexP(vertices_[i3])->dup();

    auto iv1 = pobject_->addVertex(v1);
    auto iv2 = pobject_->addVertex(v2);
    auto iv3 = pobject_->addVertex(v3);

    auto faceNum1 = pobject_->addITriangle(iv1, iv2, iv3);

    auto *face1 = pobject_->getFaceP(faceNum1);

    face1->flags_ = flags_;

    if (hasNormals) {
      face1->normals_.push_back(normals_[i1]);
      face1->normals_.push_back(normals_[i2]);
      face1->normals_.push_back(normals_[i3]);
    }

    if (hasTexturePoints) {
      face1->texturePoints_.push_back(texturePoints_[i1]);
      face1->texturePoints_.push_back(texturePoints_[i2]);
      face1->texturePoints_.push_back(texturePoints_[i3]);
    }

    face1->materialP_ = materialP_;

    // TODO: assert if non-shared material or texture ?
  }

  while (nv > 3) {
    vertices_.pop_back();

    if (hasNormals)
      normals_.pop_back();

    if (hasTexturePoints)
      texturePoints_.pop_back();

    --nv;
  }
}

// divide face a center point
void
CGeomFace3D::
divideCenter()
{
  // calc center
  auto c = calcCenter();

  pobject_->divideFace(this, c);
}

CPoint3D
CGeomFace3D::
calcCenter() const
{
  CPoint3D c;

  for (auto &vertex : vertices_) {
    auto &v = pobject_->getVertex(vertex);

    c += v.getModel();
  }

  c /= double(vertices_.size());

  return c;
}

// duplicate face and move along normal
CGeomFace3D *
CGeomFace3D::
extrude(double d)
{
  CVector3D normal;

  if (getNormalSet())
    normal = getNormal();
  else
    calcModelNormal(normal);

  // add new vertices
  std::vector<uint> inds;

  for (auto &vertex : vertices_) {
    auto &v = pobject_->getVertex(vertex);

    auto ind = pobject_->addVertex(v.getModel() + d*normal);

    pobject_->setVertexNormal(ind, normal);

    inds.push_back(ind);
  }

  //(void) pobject_->addFace(inds);
  auto *newFace = dup();
  newFace->setVertices(inds);
  pobject_->addFace(newFace);

  auto nv = vertices_.size();

  uint i1 = uint(nv - 1);

  for (uint i2 = 0; i2 < nv; i1 = i2++) {
    std::vector<uint> inds1;

    inds1.push_back(vertices_[i1]);
    inds1.push_back(vertices_[i2]);
    inds1.push_back(inds     [i2]);
    inds1.push_back(inds     [i1]);

    //(void) pobject_->addFace(inds1);
    auto *face2 = dup();
    face2->setVertices(inds1);
    pobject_->addFace(face2);
  }

  return newFace;
}

// move face on normal
void
CGeomFace3D::
extrudeMove(double d)
{
  CVector3D normal;

  if (getNormalSet())
    normal = getNormal();
  else
    calcModelNormal(normal);

  for (auto &vertex : vertices_) {
    auto &v1 = pobject_->getVertex(vertex);

    v1.setModel(v1.getModel() + d*normal);
  }
}

CGeomFace3D *
CGeomFace3D::
loopCut()
{
  CGeomFace3D *newFace = nullptr;

  auto nv = vertices_.size();

  if (nv == 4) {
    // split one edge 1 and edge 3
    auto e1v1 = pobject_->getVertex(vertices_[0]);
    auto e1v2 = pobject_->getVertex(vertices_[1]);

    auto e2v1 = pobject_->getVertex(vertices_[2]);
    auto e2v2 = pobject_->getVertex(vertices_[3]);

    auto p1 = (e1v1.getModel() + e1v2.getModel())/2.0;
    auto p2 = (e2v1.getModel() + e2v2.getModel())/2.0;

    auto v1 = pobject_->addVertex(p1);
    auto v2 = pobject_->addVertex(p2);

    // build new vertices of left/right of split
    std::vector<uint> vertices1, vertices2;

    vertices1.push_back(vertices_[0]);
    vertices1.push_back(v1);
    vertices1.push_back(v2);
    vertices1.push_back(vertices_[3]);

    vertices2.push_back(vertices_[1]);
    vertices2.push_back(vertices_[2]);
    vertices2.push_back(v2);
    vertices2.push_back(v1);

    setVertices(vertices1);

    //(void) pobject_->addFace(inds);
    newFace = dup();
    newFace->setVertices(vertices2);
    pobject_->addFace(newFace);
  }

  return newFace;
}

bool
CGeomFace3D::
removeVertex(uint ind)
{
  assert(hasVertex(ind));

  VertexList    vertices;
  Normals       normals;
  TexturePoints texturePoints;

  bool hasNormals       = hasVertexNormals();
  bool hasTexturePoints = hasVertexTexturePoints();

  int iv = 0;

  for (auto vind : vertices_) {
    if (vind != ind) {
      vertices.push_back(vind);

      if (hasNormals)
        normals.push_back(normals_[iv]);

      if (hasTexturePoints)
        texturePoints.push_back(texturePoints_[iv]);
    }

    ++iv;
  }

  std::swap(vertices_     , vertices);
  std::swap(normals_      , normals);
  std::swap(texturePoints_, texturePoints);

  return true;
}

bool
CGeomFace3D::
replaceVertex(uint oldInd, uint newInd)
{
  assert(hasVertex(oldInd));

  int iv = 0;

  for (auto vind : vertices_) {
    if (vind == oldInd)
      vertices_[iv] = newInd;

    ++iv;
  }

  return true;
}

bool
CGeomFace3D::
hasVertex(uint ind) const
{
  for (auto v : vertices_) {
    if (v == ind)
      return true;
  }

  return false;
}

//---

CGeomFace3D::FaceList
CGeomFace3D::
bevel(double d)
{
  return bevelInset(d, d);
}

CGeomFace3D::FaceList
CGeomFace3D::
inset(double d)
{
  return bevelInset(d, 0);
}

// TODO: set normals
CGeomFace3D::FaceList
CGeomFace3D::
bevelInset(double dx, double dy)
{
  // get perp distance to face point to center
  auto d1 = std::sqrt(2.0)*dx;

  auto c = calcCenter();

  // get normal to move new point down
  CVector3D n;
  calcModelNormal(n);

  std::vector<uint> vinds1;

  for (const auto &vind : vertices_) {
    auto *v = pobject_->getVertexP(vind);

    auto p = v->getModel();

    // move existing vertex down
    auto p1 = p - n*dy;

    v->setModel(p1);

    // add new vertex by moving towards center
    auto v1 = CVector3D(p, c).normalized();

    auto p2 = p + v1*d1;

    auto vind1 = pobject_->addVertex(p2);

    vinds1.push_back(vind1);
  }

  // update face to use new vertices
  auto oldVertices = vertices_;

  setVertices(vinds1);

  //---

  // create new faces for tapers
  FaceList faces;

  auto nv = vertices_.size();

  for (uint iv = 0; iv < nv; ++iv) {
    auto iv1 = iv + 1; if (iv1 == nv) iv1 = 0;

    std::vector<uint> vinds2;

    vinds2.push_back(oldVertices[iv ]);
    vinds2.push_back(oldVertices[iv1]);
    vinds2.push_back(vinds1     [iv1]);
    vinds2.push_back(vinds1     [iv ]);

    auto faceId = pobject_->addFace(vinds2);

    auto *taperFace = pobject_->getFaceP(faceId);

    faces.push_back(taperFace);
  }

  return faces;
}
