#include <CGeomSphere3D.h>
#include <CGeometry3D.h>

CGeomSphere3D::
CGeomSphere3D(CGeomScene3D *pscene, const std::string &name,
              const CPoint3D &center, double radius) :
 CGeomObject3D(pscene, name), CSphere3D(radius), center_(center),
 num_xy_(NUM_XY), num_patches_(NUM_PATCHES)
{
  addGeometry(this, center, radius, num_xy_, num_patches_);
}

CGeomSphere3D *
CGeomSphere3D::
dup() const
{
  return new CGeomSphere3D(*this);
}

void
CGeomSphere3D::
addGeometry(CGeomObject3D *object, const CPoint3D &center,
            double radius, uint num_xy, uint num_patches)
{
  std::vector<double> x, y;

  x.resize(num_xy);
  y.resize(num_xy);

  double a = -M_PI*0.5;

  double da = M_PI/(num_xy - 1);

  for (uint i = 0; i < num_xy; ++i) {
    x[i] = radius*cos(a);
    y[i] = radius*sin(a);

    a += da;
  }

  object->addBodyRev(&x[0], &y[0], num_xy, num_patches);

  object->moveBy(center);
}

void
CGeomSphere3D::
mapTexture(CGeomTexture *texture)
{
  FaceList &faces = getFaces();

  int twidth, theight;

  texture->getImageSize(&twidth, &theight);

  double dx = (twidth  - 1)/double(num_patches_);
  double dy = (theight - 1)/double(num_xy_ - 1);

  size_t pos = 0;

  double y1 = theight - 1;

  for (uint y = 0; y < num_xy_ - 1; ++y) {
    double y2 = y1 - dy;

    double x1 = 0;

    for (uint x = 0; x < num_patches_; ++x, ++pos) {
      double x2 = x1 + dx;

      CGeomTexture *texture1 = texture->dup();

      faces[pos]->setTexture(texture1);

      std::vector<CPoint2D> points;

      if (y > 0 && y < num_xy_ - 2) {
        points.push_back(CPoint2D(double(int(x2)), double(int(y1))));
        points.push_back(CPoint2D(double(int(x2)), double(int(y2))));
        points.push_back(CPoint2D(double(int(x1)), double(int(y2))));
        points.push_back(CPoint2D(double(int(x1)), double(int(y1))));
      }
      else {
        double x3 = (x1 + x2)/2;

        points.push_back(CPoint2D(double(int(x2)), double(int(y1))));
        points.push_back(CPoint2D(double(int(x3)), double(int(y2))));
        points.push_back(CPoint2D(double(int(x1)), double(int(y1))));
      }

      faces[pos]->setTextureMapping(points);

      x1 = x2;
    }

    y1 = y2;
  }
}

void
CGeomSphere3D::
mapTexture(CImagePtr image)
{
  CGeomObject3D::mapTexture(image);
}

void
CGeomSphere3D::
mapMask(CGeomMask *mask)
{
  FaceList &faces = getFaces();

  uint twidth, theight;

  mask->getImageSize(&twidth, &theight);

  double dx = (twidth  - 1)/double(num_patches_);
  double dy = (theight - 1)/double(num_xy_ - 1);

  size_t pos = 0;

  double y1 = theight - 1;

  for (uint y = 0; y < num_xy_ - 1; ++y) {
    double y2 = y1 - dy;

    double x1 = 0;

    for (uint x = 0; x < num_patches_; ++x, ++pos) {
      double x2 = x1 + dx;

      CGeomMask *mask1 = mask->dup();

      faces[pos]->setMask(mask1);

      std::vector<CPoint2D> points;

      if (y > 0 && y < num_xy_ - 2) {
        points.push_back(CPoint2D(double(int(x2)), double(int(y1))));
        points.push_back(CPoint2D(double(int(x2)), double(int(y2))));
        points.push_back(CPoint2D(double(int(x1)), double(int(y2))));
        points.push_back(CPoint2D(double(int(x1)), double(int(y1))));
      }
      else {
        double x3 = (x1 + x2)/2;

        points.push_back(CPoint2D(double(int(x2)), double(int(y1))));
        points.push_back(CPoint2D(double(int(x3)), double(int(y2))));
        points.push_back(CPoint2D(double(int(x1)), double(int(y1))));
      }

      faces[pos]->setMaskMapping(points);

      x1 = x2;
    }

    y1 = y2;
  }
}

void
CGeomSphere3D::
mapMask(CImagePtr image)
{
  CGeomObject3D::mapMask(image);
}
