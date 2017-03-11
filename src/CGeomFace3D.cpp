#include <CGeometry3D.h>
#include <CGeomZBuffer.h>
#include <CGeomLight3D.h>
#include <CGeomUtil3D.h>

CGeomFace3D::
CGeomFace3D(CGeomObject3D *pobject) :
 pobject_(pobject)
{
}

CGeomFace3D::
CGeomFace3D(CGeomObject3D *pobject, const VertexList &vertices) :
 pobject_(pobject), vertices_(vertices)
{
}

CGeomFace3D::
CGeomFace3D(const CGeomFace3D &face) :
 pobject_       (face.pobject_),
 vertices_      (face.vertices_),
 front_material_(face.front_material_),
 back_material_ (face.back_material_),
 normal_        (face.normal_),
 flags_         (face.flags_)
{
  SubFaceList::const_iterator pf1 = face.sub_faces_.begin();
  SubFaceList::const_iterator pf2 = face.sub_faces_.end  ();

  for ( ; pf1 != pf2; ++pf1) {
    CGeomFace3D *face = (*pf1)->dup();

    sub_faces_.push_back(face);

    uint ind = sub_faces_.size() - 1;

    face->setInd(ind);
  }

  SubLineList::const_iterator pl1 = face.sub_lines_.begin();
  SubLineList::const_iterator pl2 = face.sub_lines_.end  ();

  for ( ; pl1 != pl2; ++pl1) {
    CGeomLine3D *line = (*pl1)->dup();

    sub_lines_.push_back(line);

    uint ind = sub_lines_.size() - 1;

    line->setInd(ind);
  }

  if (face.texture_)
    texture_ = face.texture_->dup();
  else
    texture_ = 0;

  if (face.mask_)
    mask_ = face.mask_->dup();
  else
    mask_ = 0;
}

CGeomFace3D *
CGeomFace3D::
dup() const
{
  return new CGeomFace3D(*this);
}

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

  SubFaceList::const_iterator pf1 = sub_faces_.begin();
  SubFaceList::const_iterator pf2 = sub_faces_.end  ();

  for ( ; pf1 != pf2; ++pf1)
    (*pf1)->setObject(object);

  SubLineList::const_iterator pl1 = sub_lines_.begin();
  SubLineList::const_iterator pl2 = sub_lines_.end  ();

  for ( ; pl1 != pl2; ++pl1)
    (*pl1)->setObject(object);
}

void
CGeomFace3D::
setTexture(CImagePtr image)
{
  texture_ = CGeometryInst->createTexture(image);
}

void
CGeomFace3D::
setTextureMapping(const std::vector<CPoint2D> &points)
{
  if (texture_)
    texture_->setMapping(points);
}

void
CGeomFace3D::
setMask(CImagePtr image)
{
  delete mask_;

  mask_ = CGeometryInst->createMask(image);
}

void
CGeomFace3D::
setMaskMapping(const std::vector<CPoint2D> &points)
{
  if (mask_)
    mask_->setMapping(points);
}

void
CGeomFace3D::
setLighted(bool lighted)
{
  if (lighted)
    flags_ |= LIGHTED;
  else
    flags_ &= ~LIGHTED;
}

void
CGeomFace3D::
setTwoSided(bool two_sided)
{
  if (two_sided)
    flags_ |= TWO_SIDED;
  else
    flags_ &= ~TWO_SIDED;
}

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
  CGeomFace3D *face = CGeometryInst->createFace3D(pobject_, vertices);

  sub_faces_.push_back(face);

  uint ind = sub_faces_.size() - 1;

  face->setInd(ind);

  return ind;
}

uint
CGeomFace3D::
addSubLine(uint start, uint end)
{
  CGeomLine3D *line = CGeometryInst->createLine3D(pobject_, start, end);

  sub_lines_.push_back(line);

  uint ind = sub_lines_.size() - 1;

  line->setInd(ind);

  return ind;
}

void
CGeomFace3D::
setSubFaceColor(const CRGBA &rgba)
{
  SubFaceList::iterator p1 = sub_faces_.begin();
  SubFaceList::iterator p2 = sub_faces_.end  ();

  for ( ; p1 != p2; ++p1)
    (*p1)->setColor(rgba);
}

void
CGeomFace3D::
setSubFaceColor(uint ind, const CRGBA &rgba)
{
  sub_faces_[ind]->setColor(rgba);
}

void
CGeomFace3D::
setSubLineColor(uint ind, const CRGBA &rgba)
{
  sub_lines_[ind]->setColor(rgba);
}

void
CGeomFace3D::
drawSolid(CGeom3DRenderer *)
{
  uint num_points = vertices_.size();

  if (num_points < 3)
    return;

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
  uint num_points = vertices_.size();

  if (num_points < 3)
    return;

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

  CPoint3D ppoint1 = pobject_->getVertex(vertices_.back()).getPixel();

  VertexList::iterator p1 = vertices_.begin();
  VertexList::iterator p2 = vertices_.end  ();

  for ( ; p1 != p2; ++p1) {
    const CPoint3D &ppoint2 = pobject_->getVertex(*p1).getPixel();

    renderer->drawLine(CIPoint2D(ppoint1.x, ppoint1.y), CIPoint2D(ppoint2.x, ppoint2.y));

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

  CPoint3D ppoint1 = pobject_->getVertex(vertices_.back()).getPixel();

  VertexList::iterator p1 = vertices_.begin();
  VertexList::iterator p2 = vertices_.end  ();

  for ( ; p1 != p2; ++p1) {
    const CPoint3D &ppoint2 = pobject_->getVertex(*p1).getPixel();

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

  int w = zbuffer->getWidth ();
  int h = zbuffer->getHeight();

  uint num_points = vertices_.size();

  // get y limits
  const CPoint3D &pixel = pobject_->getVertex(vertices_[0]).getPixel();

  int ypmin = CMathGen::RoundUp  (pixel.y);
  int ypmax = CMathGen::RoundDown(pixel.y);

  for (uint i1 = 1; i1 < num_points; ++i1) {
    const CPoint3D &pixel = pobject_->getVertex(vertices_[i1]).getPixel();

    ypmin = std::min(ypmin, CMathGen::RoundUp  (pixel.y));
    ypmax = std::max(ypmax, CMathGen::RoundDown(pixel.y));
  }

  int    i1min, i2min, i1max, i2max;
  double zmin, zmax;

  ypmin = std::max(ypmin, 0);
  ypmax = std::min(ypmax, h - 1);

  for (int yp = ypmin; yp <= ypmax; ++yp) {
    bool set = false;

    int xpmin, xpmax;

    int xp;

    for (uint i1 = num_points - 1, i2 = 0; i2 < num_points; i1 = i2, ++i2) {
      const CPoint3D &pixel1 = pobject_->getVertex(vertices_[i1]).getPixel();
      const CPoint3D &pixel2 = pobject_->getVertex(vertices_[i2]).getPixel();

      // skip line if not in y range
      if ((pixel1.y < yp && pixel2.y < yp) ||
          (pixel1.y > yp && pixel2.y > yp) ||
          fabs(pixel2.y - pixel1.y) < 1E-6)
        continue;

      // get x intersect
      double fx = (pixel2.x - pixel1.x)/(pixel2.y - pixel1.y);

      xp = CMathGen::Round((yp - pixel1.y)*fx + pixel1.x);

      // update min and/or max
      if (! set) {
        xpmin = xp;
        i1min = i1;
        i2min = i2;

        xpmax = xp;
        i1max = i1;
        i2max = i2;

        set = true;
      }
      else {
        if (xp < xpmin) {
          xpmin = xp;
          i1min = i1;
          i2min = i2;
        }

        if (xp > xpmax) {
          xpmax = xp;
          i1max = i1;
          i2max = i2;
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
    const CPoint3D &pixel1 = pobject_->getVertex(vertices_[i1min]).getPixel();
    const CPoint3D &pixel2 = pobject_->getVertex(vertices_[i2min]).getPixel();

    if (fabs(pixel2.x - pixel1.x) > fabs(pixel2.y - pixel1.y)) {
      d = xpmin - pixel1.x;
      i = 1.0/(pixel2.x - pixel1.x);
    }
    else {
      d = yp - pixel1.y;
      i = 1.0/(pixel2.y - pixel1.y);
    }

    zmin = (pixel2.z - pixel1.z)*i*d + pixel1.z;

    if (texture_)
      tmin = interpTexturePoint(i1min, i2min, d, i);

    if (mask_)
      mmin = interpMaskPoint(i1min, i2min, d, i);

    // get max z intersect (viewed)
    const CPoint3D &pixel3 = pobject_->getVertex(vertices_[i1max]).getPixel();
    const CPoint3D &pixel4 = pobject_->getVertex(vertices_[i2max]).getPixel();

    if (fabs(pixel4.x - pixel3.x) > fabs(pixel4.y - pixel3.y)) {
      d = xpmax - pixel3.x;
      i = 1.0/(pixel4.x - pixel3.x);
    }
    else {
      d = yp - pixel3.y;
      i = 1.0/(pixel4.y - pixel3.y);
    }

    zmax = (pixel4.z - pixel3.z)*i*d + pixel3.z;

    if (texture_)
      tmax = interpTexturePoint(i1max, i2max, d, i);

    if (mask_)
      mmax = interpMaskPoint(i1max, i2max, d, i);

    //-------

    double zz, dz = 0;

    CPoint2D t, dt;
    CPoint2D m, dm;

    if (xpmin < xpmax) {
      dz = (zmax - zmin)/(xpmax - xpmin);

      if (texture_)
        dt = (tmax - tmin)/(xpmax - xpmin);

      if (mask_)
        dm = (mmax - mmin)/(xpmax - xpmin);
    }

    zz = zmin;
    t  = tmin;
    m  = mmin;

    for (int xp = xpmin; xp <= xpmax; ++xp, zz += dz) {
      if (xp < 0 || xp >= w) {
        if (texture_)
          t += dt;

        if (mask_)
          m += dm;
      }
      else {
        if (texture_) {
          CRGBA rgba = texture_->getImageRGBA(int(t.x), int(t.y));

          t += dt;

          zbuffer->setForeground(rgba*color_factor);
        }

        if (mask_) {
          bool set = mask_->getImageSet(int(m.x), int(m.y));

          m += dm;

          if (! set)
            continue;
        }

        zbuffer->drawFacePoint(this, xp, yp, zz);
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
        ! pobject_->lightPoint(mid_point, normal, front_material_, rgba)) {
      CVector3D dir(mid_point, CPoint3D(0,0,1));

      double factor1 = normal.dotProduct(dir.normalized());

      if (factor1 < 0)
        factor1 = -factor1;
//      return false;

      rgba = front_material_.getColor();

      rgba.scaleRGB(factor1);
    }
  }
  else {
    CVector3D dir(mid_point, CPoint3D(0,0,1));

    double factor1 = normal.dotProduct(dir.normalized());

    if (factor1 < 0) {
      factor1 = -factor1;
//    return false;
    }

    rgba = front_material_.getColor();

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

  CVector3D dir(mid_point, CPoint3D(0,0,1));

  *factor = normal.dotProduct(dir.normalized());

  if (*factor < 0) {
    *factor = -(*factor);
//  return false;
  }

  return true;
}

void
CGeomFace3D::
getMidPoint(CPoint3D &mid_point)
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
  const CPoint2D &point1 = texture_->getMappingPoint(i1);
  const CPoint2D &point2 = texture_->getMappingPoint(i2);

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
  const CPoint2D &point1 = mask_->getMappingPoint(i1);
  const CPoint2D &point2 = mask_->getMappingPoint(i2);

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
  const CPoint3D &point1 = pobject_->getVertex(vertices_[0]).getProjected();
  const CPoint3D &point2 = pobject_->getVertex(vertices_[1]).getProjected();
  const CPoint3D &point3 = pobject_->getVertex(vertices_[2]).getProjected();

  CTriangle3D triangle(point1, point2, point3);

  return triangle.orientationXY();
}
