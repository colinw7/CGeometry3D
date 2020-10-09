#include <CGeomFill3D.h>
#include <CGeomVertex3D.h>
#include <CGeomLight3D.h>
#include <CGeomZBuffer.h>
#include <CGeomObject3D.h>
#include <CMathRound.h>

CGeomFill3D::
CGeomFill3D(uint width, uint height)
{
  resize(width, height);
}

CGeomFill3D::
~CGeomFill3D()
{
  resize(0, 0);
}

void
CGeomFill3D::
resize(uint width, uint height)
{
  if (width == width_ && height == height_)
    return;

  for (uint y = 0; y < height_; ++y)
    delete [] lines_[y].points;

  delete [] lines_;

  //-----

  width_  = width;
  height_ = height;

  if (height_ > 0) {
    lines_ = new Line [height_];

    for (uint y = 0; y < height_; ++y) {
      Line *line = &lines_[y];

      line->width  = width_;
      line->points = new Point [width_];
    }
  }
  else
    lines_ = 0;
}

void
CGeomFill3D::
clear()
{
  for (uint y = 0; y < height_; ++y) {
    Line *line = &lines_[y];

    if (! line->set) continue;

    line->clear();
  }
}

// TODO: mask
void
CGeomFill3D::
fill(const VertexAdapter &vadapter)
{
  int w = getWidth ();
  int h = getHeight();

  uint num_vertices = vadapter.getNumVertices();

  if (num_vertices < 3) return;

  // get y limits
  const CPoint3D &pixel = vadapter.getVertex(0).getPixel();

  int ypmin = CMathRound::RoundUp  (pixel.y);
  int ypmax = CMathRound::RoundDown(pixel.y);

  for (uint i1 = 1; i1 < num_vertices; ++i1) {
    const CPoint3D &pixel1 = vadapter.getVertex(i1).getPixel();

    ypmin = std::min(ypmin, CMathRound::RoundUp  (pixel1.y));
    ypmax = std::max(ypmax, CMathRound::RoundDown(pixel1.y));
  }

  ypmin = std::max(ypmin, 0);
  ypmax = std::min(ypmax, h - 1);

  //------

  int    i1min, i2min, i1max, i2max;
  double zmin, zmax;

  CPoint2D tmin(0,0), tmax(0,0);

  CRGBA rgba_min = getDefaultColor();
  CRGBA rgba_max = rgba_min;

  CVector3D normal_min = getDefaultNormal();
  CVector3D normal_max = normal_min;

  Point point;

#if 0
  CPoint2D mmin, mmax;
#endif

  // rasterize polygon for each value of y in limits
  for (int yp = ypmin; yp <= ypmax; ++yp) {
    bool set = false;

    int xpmin, xpmax;

    int xp;

    // for each polygon line find x range for this y
    for (uint i1 = num_vertices - 1, i2 = 0; i2 < num_vertices; i1 = i2, ++i2) {
      const CPoint3D &pixel1 = vadapter.getVertex(i1).getPixel();
      const CPoint3D &pixel2 = vadapter.getVertex(i2).getPixel();

      // skip line if not in y range or zero length line
      if ((pixel1.y < yp && pixel2.y < yp) ||
          (pixel1.y > yp && pixel2.y > yp) ||
          fabs(pixel2.y - pixel1.y) < 1E-6)
        continue;

      // get x intersect
      double fx = (pixel2.x - pixel1.x)/(pixel2.y - pixel1.y);

      xp = CMathRound::Round((yp - pixel1.y)*fx + pixel1.x);

      // update min and/or max
      if (! set) {
        xpmin = xp; i1min = i1; i2min = i2;
        xpmax = xp; i1max = i1; i2max = i2;

        set = true;
      }
      else {
        if (xp < xpmin) {
          xpmin = xp; i1min = i1; i2min = i2;
        }

        if (xp > xpmax) {
          xpmax = xp; i1max = i1; i2max = i2;
        }
      }
    }

    //-------

    if (! set)
      continue;

    //-------

    double d, i, id;

    // get relative intersect of y value on min line
    const CPoint3D &pixel1 = vadapter.getVertex(i1min).getPixel();
    const CPoint3D &pixel2 = vadapter.getVertex(i2min).getPixel();

    if (fabs(pixel2.x - pixel1.x) > fabs(pixel2.y - pixel1.y)) {
      d = xpmin - pixel1.x;
      i = 1.0/(pixel2.x - pixel1.x);
    }
    else {
      d = yp - pixel1.y;
      i = 1.0/(pixel2.y - pixel1.y);
    }

    id = i*d;

    // interp z at minimim
    zmin = (pixel2.z - pixel1.z)*id + pixel1.z;

    // interp color at minimum (if required)
    if (! getFlat()) {
      const CRGBA &rgba1 = vadapter.getVertex(i1min).getColor();
      const CRGBA &rgba2 = vadapter.getVertex(i2min).getColor();

      rgba_min = (rgba2 - rgba1)*id + rgba1;
    }

    // interp normal at maximum (if required)
    if (getSmooth()) {
      const CVector3D &normal1 = vadapter.getVertex(i1min).getNormal();
      const CVector3D &normal2 = vadapter.getVertex(i2min).getNormal();

      normal_min = (normal2 - normal1)*id + normal1;
    }

    // interp texture map at minimum (if required)
    if (getTexture()) {
      const CPoint3D &point1 = vadapter.getVertex(i1min).getTextureMap();
      const CPoint3D &point2 = vadapter.getVertex(i2min).getTextureMap();

      tmin.x = (point2.x - point1.x)*id + point1.x;
      tmin.y = (point2.y - point1.y)*id + point1.y;
    }

#if 0
    if (mask_)
      mmin = interpMaskPoint(i1min, i2min, d, i);
#endif

    //-----------------

    // get relative intersect of y value on max line
    const CPoint3D &pixel3 = vadapter.getVertex(i1max).getPixel();
    const CPoint3D &pixel4 = vadapter.getVertex(i2max).getPixel();

    if (fabs(pixel4.x - pixel3.x) > fabs(pixel4.y - pixel3.y)) {
      d = xpmax - pixel3.x;
      i = 1.0/(pixel4.x - pixel3.x);
    }
    else {
      d = yp - pixel3.y;
      i = 1.0/(pixel4.y - pixel3.y);
    }

    id = i*d;

    // interp z at maximum
    zmax = (pixel4.z - pixel3.z)*id + pixel3.z;

    // interp color at maximum (if required)
    if (! getFlat()) {
      const CRGBA &rgba1 = vadapter.getVertex(i1max).getColor();
      const CRGBA &rgba2 = vadapter.getVertex(i2max).getColor();

      rgba_max = (rgba2 - rgba1)*id + rgba1;
    }

    // interp normal at maximum (if required)
    if (getSmooth()) {
      const CVector3D &normal1 = vadapter.getVertex(i1max).getNormal();
      const CVector3D &normal2 = vadapter.getVertex(i2max).getNormal();

      normal_max = (normal2 - normal1)*id + normal1;
    }

    // interp texture map at maximum (if required)
    if (getTexture()) {
      const CPoint3D &point1 = vadapter.getVertex(i1max).getTextureMap();
      const CPoint3D &point2 = vadapter.getVertex(i2max).getTextureMap();

      tmax.x = (point2.x - point1.x)*id + point1.x;
      tmax.y = (point2.y - point1.y)*id + point1.y;
    }

#if 0
    if (mask_)
      mmax = interpMaskPoint(i1max, i2max, d, i);
#endif

    //-------

    double    dz = 0;
    CRGBA     drgba;
    CVector3D dnormal;
    CPoint2D  dt;
#if 0
    CPoint2D  dm;
#endif

    double ixpd = 1.0/(xpmax - xpmin);

    if (xpmin < xpmax) {
      dz = (zmax - zmin)*ixpd;

      if (! getFlat())
        drgba = (rgba_max - rgba_min)*ixpd;

      if (getSmooth())
        dnormal = (normal_max - normal_min)*ixpd;

      if (getTexture())
        dt = (tmax - tmin)*ixpd;

#if 0
      if (mask_)
        dm = (mmax - mmin)/(xpmax - xpmin);
#endif
    }

    point.set    = true;
    point.z      = zmin;
    point.rgba   = rgba_min;
    point.normal = normal_min;
    point.tmap   = tmin;
#if 0
    point.m       = mmin;
#endif

    for (int xp1 = xpmin; xp1 <= xpmax; ++xp1, point.z += dz) {
      if (xp1 >= 0 && xp1 < w)
        setPoint(xp1, yp, point);

      if (! getFlat())
        point.rgba += drgba;

      if (getSmooth())
        point.normal += dnormal;

      if (getTexture())
        point.tmap += dt;

#if 0
      if (mask_)
        point.m += dm;
#endif
    }
  }
}

void
CGeomFill3D::
stipple(CImagePtr stipple)
{
  double gray;
  int    x1, y1;

  int w = stipple->getWidth ();
  int h = stipple->getHeight();

  for (uint y = 0; y < height_; ++y) {
    Line *line = &lines_[y];

    if (! line->set) continue;

    y1 = y % h;

    for (uint x = line->start; x <= line->end; ++x) {
      Point *point = &line->points[x];

      if (! point->set || point->stippled) continue;

      x1 = x % w;

      stipple->getGrayPixel(x1, y1, &gray);

      if (gray <= 0.5)
        point->set = false;

      point->stippled = true;
    }
  }
}

void
CGeomFill3D::
setStippled(bool stippled)
{
  for (uint y = 0; y < height_; ++y) {
    Line *line = &lines_[y];

    if (! line->set) continue;

    for (uint x = line->start; x <= line->end; ++x) {
      Point *point = &line->points[x];

      if (! point->set) continue;

      point->stippled = stippled;
    }
  }
}

void
CGeomFill3D::
light(const CMaterial &material, const CRGBA &ambient,
      const std::vector<CGeomLight3D *> &lights)
{
  CPoint3D gpoint;

  for (uint y = 0; y < height_; ++y) {
    Line *line = &lines_[y];

    if (! line->set) continue;

    gpoint.y = y;

    for (uint x = line->start; x <= line->end; ++x) {
      Point *point = &line->points[x];

      if (! point->set) continue;

      gpoint.x = x;
      gpoint.z = point->z;

      CRGBA rgba = material.getEmission();

      rgba += ambient*point->rgba;

      std::vector<CGeomLight3D *>::const_iterator plight1 = lights.begin();
      std::vector<CGeomLight3D *>::const_iterator plight2 = lights.end  ();

      for ( ; plight1 != plight2; ++plight1)
        (*plight1)->lightPoint(rgba, gpoint, point->normal, material);

      point->rgba = rgba;
    }
  }
}

void
CGeomFill3D::
fogExp(double density)
{
  double f;

  for (uint y = 0; y < height_; ++y) {
    Line *line = &lines_[y];

    if (! line->set) continue;

    for (uint x = line->start; x <= line->end; ++x) {
      Point *point = &line->points[x];

      if (! point->set || point->fogged) continue;

      f = exp(density*point->z);

      point->rgba = (point->rgba*f).clamp();

      point->fogged = true;
    }
  }
}

void
CGeomFill3D::
setFogged(bool fogged)
{
  for (uint y = 0; y < height_; ++y) {
    Line *line = &lines_[y];

    if (! line->set) continue;

    for (uint x = line->start; x <= line->end; ++x) {
      Point *point = &line->points[x];

      if (! point->set) continue;

      point->fogged = fogged;
    }
  }
}

void
CGeomFill3D::
render(CGeom3DRenderer *renderer)
{
  for (uint y = 0; y < height_; ++y) {
    Line *line = &lines_[y];

    if (! line->set) continue;

    for (uint x = line->start; x <= line->end; ++x) {
      Point *point = &line->points[x];

      if (! point->set) continue;

      renderer->setForeground(point->rgba);

      renderer->drawPoint(CIPoint2D(x, y));
    }
  }
}

void
CGeomFill3D::
render(CGeomZBuffer *buffer)
{
  for (uint y = 0; y < height_; ++y) {
    Line *line = &lines_[y];

    if (! line->set) continue;

    for (uint x = line->start; x <= line->end; ++x) {
      Point *point = &line->points[x];

      if (! point->set) continue;

      buffer->setForeground(point->rgba);

      buffer->drawFacePoint(0, x, y, point->z);
    }
  }
}

#if 0
void
CGeomFill3D::
zrender(CGeomZBuffer *zbuffer)
{
  for (uint y = 0; y < height_; ++y) {
    Line *line = &lines_[y];

    if (! line->set) continue;

    for (uint x = line->start; x <= line->end; ++x) {
      Point *point = &line->points[x];

      if (! point->set) continue;

      zbuffer->setForeground(point->rgba);

      zbuffer->drawPoint(x, y, z);
    }
  }
}
#endif

//----------

const CGeomVertex3D &
CGeomFill3D::VertexIAdapter::
getVertex(uint i) const
{
  return object_->getVertex(vertices_[i]);
}
