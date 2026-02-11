#include <CGeometry3D.h>
#include <CGeomZBuffer.h>
#include <CGeomLight3D.h>
#include <CGeomUtil3D.h>
#include <CMathRound.h>

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
  getMidPoint(mid_point);

  CVector3D normal;
  calcNormal(normal);

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
  getMidPoint(mid_point);

  CVector3D normal;
  calcNormal(normal);

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
getMidPoint(CPoint3D &mid_point) const
{
  mid_point = pobject_->verticesMidPoint(vertices_);
}

void
CGeomFace3D::
calcNormal(CVector3D &normal) const
{
  normal = pobject_->verticesNormal(vertices_);
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
  const auto &point1 = pobject_->getVertex(vertices_[0]).getProjected();
  const auto &point2 = pobject_->getVertex(vertices_[1]).getProjected();
  const auto &point3 = pobject_->getVertex(vertices_[2]).getProjected();

  CTriangle3D triangle(point1, point2, point3);

  return triangle.orientationXY();
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
divideCenter()
{
  // calc center
  CPoint3D c;

  for (auto &vertex : vertices_) {
    auto &v = pobject_->getVertex(vertex);

    c += v.getModel();
  }

  c /= double(vertices_.size());

  pobject_->divideFace(this, c);
}

CGeomFace3D *
CGeomFace3D::
extrude(double d)
{
  CVector3D normal;
  calcNormal(normal);

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

void
CGeomFace3D::
extrudeMove(double d)
{
  CVector3D normal;
  calcNormal(normal);

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
