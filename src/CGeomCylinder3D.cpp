#include <CGeomCylinder3D.h>
#include <CGeomTexture.h>
#include <CGeomMask.h>

CGeomCylinder3D::
CGeomCylinder3D(CGeomScene3D *pscene, const std::string &name,
                double xc, double yc, double zc, double w, double h) :
 CGeomObject3D(pscene, name)
{
  addGeometry(this, xc, yc, zc, w, h, num_patches_);
}

void
CGeomCylinder3D::
addGeometry(CGeomObject3D *object, double xc, double yc, double zc,
            double w, double h, uint num_patches)
{
  double x[4], y[4];

  x[0] =    0; y[0] = -h/2;
  x[1] = +w/2; y[1] = -h/2;
  x[2] = +w/2; y[2] = +h/2;
  x[3] =    0; y[3] = +h/2;

  object->addBodyRev(x, y, 4, num_patches);

  object->moveBy(CPoint3D(xc, yc, zc));
}

void
CGeomCylinder3D::
mapTexture(CGeomTexture *texture)
{
  FaceList &faces = getFaces();

  int twidth, theight;

  texture->getImageSize(&twidth, &theight);

  double dx = (twidth - 1)/((double) num_patches_);
  double dy = (theight - 1);

  double y1 = 0;
  double y2 = y1 + dy;

  double x1 = 0;

  for (uint i = num_patches_; i < 2*num_patches_; ++i) {
    double x2 = x1 + dx;

    faces[i]->setTexture(texture);

    std::vector<CPoint2D> points;

    points.push_back(CPoint2D((double) ((int) x2), (double) ((int) y2)));
    points.push_back(CPoint2D((double) ((int) x2), (double) ((int) y1)));
    points.push_back(CPoint2D((double) ((int) x1), (double) ((int) y1)));
    points.push_back(CPoint2D((double) ((int) x1), (double) ((int) y2)));

    faces[i]->setTextureMapping(points);

    x1 = x2;
  }
}

void
CGeomCylinder3D::
mapTexture(CImagePtr image)
{
  CGeomObject3D::mapTexture(image);
}

void
CGeomCylinder3D::
mapMask(CGeomMask *mask)
{
  FaceList &faces = getFaces();

  uint twidth, theight;

  mask->getImageSize(&twidth, &theight);

  double dx = (twidth - 1)/((double) num_patches_);
  double dy = (theight - 1);

  double y1 = 0;
  double y2 = y1 + dy;

  double x1 = 0;

  for (uint i = num_patches_; i < 2*num_patches_; ++i) {
    double x2 = x1 + dx;

    faces[i]->setMask(mask);

    std::vector<CPoint2D> points;

    points.push_back(CPoint2D((double) ((int) x2), (double) ((int) y2)));
    points.push_back(CPoint2D((double) ((int) x2), (double) ((int) y1)));
    points.push_back(CPoint2D((double) ((int) x1), (double) ((int) y1)));
    points.push_back(CPoint2D((double) ((int) x1), (double) ((int) y2)));

    faces[i]->setMaskMapping(points);

    x1 = x2;
  }
}

void
CGeomCylinder3D::
mapMask(CImagePtr image)
{
  CGeomObject3D::mapMask(image);
}
