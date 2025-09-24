#include <CGeomCylinder3D.h>
#include <CGeomTexture.h>
#include <CGeomMask.h>
#include <CCylinder3D.h>

CGeomCylinder3D::
CGeomCylinder3D(CGeomScene3D *pscene, const std::string &name,
                double xc, double yc, double zc, double w, double h) :
 CGeomObject3D(pscene, name)
{
  addGeometry(this, xc, yc, zc, w, h, num_patches_);
}

CGeomCylinder3D::
CGeomCylinder3D(CGeomScene3D *pscene, const std::string &name,
                const CPoint3D &center, double w, double h) :
 CGeomObject3D(pscene, name)
{
  addGeometry(this, center, w, h, num_patches_);
}

void
CGeomCylinder3D::
addGeometry(CGeomObject3D *object, const CPoint3D &center,
            double w, double h, uint num_patches)
{
  addGeometry(object, center.x, center.y, center.z, w, h, num_patches);
}

void
CGeomCylinder3D::
addGeometry(CGeomObject3D *object, double xc, double yc, double zc,
            double w, double h, uint num_patches)
{
  int np = 8;

  std::vector<double> x, y;

  x.resize(np + 2);
  y.resize(np + 2);

  auto dy = h/(np - 1);

  double y1 = -h/2.0;

  x[0] = 0.0; y[0] = y1;

  for (int i = 0; i < np; ++i) {
    x[i + 1] = w/2; y[i + 1] = y1;

    y1 += dy;
  }

  x[np + 1] = 0.0; y[np + 1] = h/2.0;

  object->addBodyRev(&x[0], &y[0], np + 2, num_patches);

  object->moveBy(CPoint3D(xc, yc, zc));
}

void
CGeomCylinder3D::
mapTexture(CGeomTexture *texture)
{
  FaceList &faces = getFaces();

  int twidth, theight;

  texture->getImageSize(&twidth, &theight);

  double dx = (twidth - 1)/double(num_patches_);
  double dy = (theight - 1);

  double y1 = 0;
  double y2 = y1 + dy;

  double x1 = 0;

  for (uint i = num_patches_; i < 2*num_patches_; ++i) {
    double x2 = x1 + dx;

    faces[i]->setTexture(texture);

    std::vector<CPoint2D> points;

    points.push_back(CPoint2D(double(int(x2)), double(int(y2))));
    points.push_back(CPoint2D(double(int(x2)), double(int(y1))));
    points.push_back(CPoint2D(double(int(x1)), double(int(y1))));
    points.push_back(CPoint2D(double(int(x1)), double(int(y2))));

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

  double dx = (twidth - 1)/double(num_patches_);
  double dy = (theight - 1);

  double y1 = 0;
  double y2 = y1 + dy;

  double x1 = 0;

  for (uint i = num_patches_; i < 2*num_patches_; ++i) {
    double x2 = x1 + dx;

    faces[i]->setMask(mask);

    std::vector<CPoint2D> points;

    points.push_back(CPoint2D(double(int(x2)), double(int(y2))));
    points.push_back(CPoint2D(double(int(x2)), double(int(y1))));
    points.push_back(CPoint2D(double(int(x1)), double(int(y1))));
    points.push_back(CPoint2D(double(int(x1)), double(int(y2))));

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

void
CGeomCylinder3D::
addNormals(CGeomObject3D *object, double w, double h)
{
  CCylinder3D cylinder(w/2.0, h);

  FaceList &faces = object->getFaces();

  for (auto *face : faces) {
    for (const auto vind : face->getVertices()) {
      auto &vertex = object->getVertex(vind);

      auto p = vertex.getModel();

      auto n = cylinder.pointNormal(CPoint3D(p.x, p.z, p.y));

      vertex.setNormal(n);
    }
  }
}
