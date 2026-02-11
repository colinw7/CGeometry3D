#include <CGeomCylinder3D.h>
#include <CGeomTexture.h>
#include <CGeomMask.h>
#include <CCylinder3D.h>

CGeomCylinder3D::
CGeomCylinder3D(CGeomScene3D *pscene, const std::string &name,
                double xc, double yc, double zc, double w, double h,
                const ConfigData &configData) :
 CGeomObject3D(pscene, name), center_(xc, yc, zc), width_(w), height_(h), configData_(configData)
{
  addGeometry(this, center_, width_, height_, configData_);
}

CGeomCylinder3D::
CGeomCylinder3D(CGeomScene3D *pscene, const std::string &name,
                const CPoint3D &center, double w, double h,
                const ConfigData &configData) :
 CGeomObject3D(pscene, name), center_(center), width_(w), height_(h), configData_(configData)
{
  addGeometry(this, center_, width_, height_, configData_);
}

void
CGeomCylinder3D::
addGeometry(CGeomObject3D *object, const CPoint3D &center,
            double w, double h, const ConfigData &configData)
{
  addGeometry(object, center.x, center.y, center.z, w, h, configData);
}

void
CGeomCylinder3D::
addGeometry(CGeomObject3D *object, double xc, double yc, double zc,
            double w, double h, const ConfigData &configData)
{
  int np = configData.num_xy;

  std::vector<double> x, y;

  if (configData.cap_type != CapType::NONE) {
    // create C shape (0, -h/2, -w/2, h/2)
    x.resize(np + 2);
    y.resize(np + 2);
  }
  else {
    x.resize(np);
    y.resize(np);
  }

  auto dy = h/(np - 1);

  double y1 = -h/2.0;

  uint ii = 0;

  if (configData.cap_type != CapType::NONE) {
    x[ii] = 0.0; y[ii] = y1; ++ii;
  }

  for (int i = 0; i < np; ++i) {
    x[ii] = w/2; y[ii] = y1; ++ii;

    y1 += dy;
  }

  if (configData.cap_type != CapType::NONE) {
    x[ii] = 0.0; y[ii] = h/2.0;
  }

  //---

  // rotate around y to add z (-w/2, w/2)
  BodyRevData bodyRevData;

  bodyRevData.uniquify = true;

  if (configData.cap_type != CapType::NONE) {
    bodyRevData.tagInds[0     ] = 1; // bottom center
    bodyRevData.tagInds[1     ] = 1; // first edge
  //bodyRevData.tagInds[np    ] = 2;
    bodyRevData.tagInds[np + 1] = 2; // top center
  }

  object->addBodyRev(&x[0], &y[0], uint(x.size()), configData.num_patches, bodyRevData);

  object->moveBy(CPoint3D(xc, yc, zc));
}

void
CGeomCylinder3D::
mapTexture(CGeomTexture *texture)
{
  auto &faces = getFaces();

  int twidth, theight;

  texture->getImageSize(&twidth, &theight);

  double dx = (twidth - 1)/double(configData_.num_patches);
  double dy = (theight - 1);

  double y1 = 0;
  double y2 = y1 + dy;

  double x1 = 0;

  for (uint i = configData_.num_patches; i < 2*configData_.num_patches; ++i) {
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
  auto &faces = getFaces();

  uint twidth, theight;

  mask->getImageSize(&twidth, &theight);

  double dx = (twidth - 1)/double(configData_.num_patches);
  double dy = (theight - 1);

  double y1 = 0;
  double y2 = y1 + dy;

  double x1 = 0;

  for (uint i = configData_.num_patches; i < 2*configData_.num_patches; ++i) {
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
  // cylinder (-w/2 -> w/2, -w/2 -> w/2, 0 -> h)
  CCylinder3D cylinder(w/2.0, h);

  // object has y/z flipped
  auto &faces = object->getFaces();

  for (auto *face : faces) {
    for (const auto &vind : face->getVertices()) {
      auto &vertex = object->getVertex(vind);

      auto t = vertex.getTag();

      if      (t == 1)
        vertex.setNormal(CVector3D(0, -1, 0));
      else if (t == 2)
        vertex.setNormal(CVector3D(0,  1, 0));
      else {
        auto p = vertex.getModel();

        auto n = cylinder.pointNormal(CPoint3D(p.x, p.z, p.y));

        vertex.setNormal(CVector3D(n.getX(), n.getZ(), n.getY()));
      }
    }
  }
}
