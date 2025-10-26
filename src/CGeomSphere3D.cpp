#include <CGeomSphere3D.h>
#include <CGeometry3D.h>

CGeomSphere3D::
CGeomSphere3D(CGeomScene3D *pscene, const std::string &name,
              const CPoint3D &center, double radius, const ConfigData &configData) :
 CGeomObject3D(pscene, name), CSphere3D(radius), center_(center), configData_(configData)
{
  addGeometry(this, center, radius, configData_);
}

CGeomSphere3D *
CGeomSphere3D::
dup() const
{
  return new CGeomSphere3D(*this);
}

void
CGeomSphere3D::
addGeometry(CGeomObject3D *object, const CPoint3D &center, double radius,
            const ConfigData &configData)
{
  std::vector<double> x, y;

  // create arc from -PI/2 -> PI/2
  x.resize(configData.num_xy);
  y.resize(configData.num_xy);

  double a  = -M_PI*0.5;
  double da = M_PI/(configData.num_xy - 1);

  for (uint i = 0; i < configData.num_xy; ++i) {
    x[i] = radius*std::cos(a);
    y[i] = radius*std::sin(a);

    a += da;
  }

  // revolve
  BodyRevData bodyRevData;
  bodyRevData.angleStart = configData.angleStart;
  bodyRevData.angleDelta = configData.angleDelta;

  object->addBodyRev(&x[0], &y[0], configData.num_xy, configData.num_patches, bodyRevData);

  object->moveBy(center);
}

void
CGeomSphere3D::
addTextureMap(CGeomObject3D *object, CGeomTexture *texture, const ConfigData &configData)
{
  mapTextureI(object, texture, configData);
}

void
CGeomSphere3D::
addTexturePoints(CGeomObject3D *object, const ConfigData &configData)
{
  addTexturePointsI(object, configData);
}

void
CGeomSphere3D::
addNormals(CGeomObject3D *object, double radius)
{
  addNormalsI(object, radius);
}

void
CGeomSphere3D::
addNormals()
{
  addNormalsI(this, getRadius());
}

void
CGeomSphere3D::
mapTexture(CGeomTexture *texture)
{
  mapTextureI(this, texture, configData_);
}

void
CGeomSphere3D::
addTexturePoints()
{
  addTexturePointsI(this, configData_);
}

void
CGeomSphere3D::
mapTextureI(CGeomObject3D *object, CGeomTexture *texture, const ConfigData &configData)
{
  FaceList &faces = object->getFaces();

  int twidth, theight;

  texture->getImageSize(&twidth, &theight);

  double dx = (twidth  - 1)/double(configData.num_patches);
  double dy = (theight - 1)/double(configData.num_xy - 1);

  size_t pos = 0;

  double y1 = theight - 1;

  for (uint y = 0; y < configData.num_xy - 1; ++y) {
    double y2 = y1 - dy;

    double x1 = 0;

    for (uint x = 0; x < configData.num_patches; ++x, ++pos) {
      double x2 = x1 + dx;

      CGeomTexture *texture1 = texture->dup();

      faces[pos]->setTexture(texture1);

      std::vector<CPoint2D> points;

      if (y > 0 && y < configData.num_xy - 2) {
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
addTexturePointsI(CGeomObject3D *object, const ConfigData &configData)
{
  FaceList &faces = object->getFaces();

  double dx = 1.0/double(configData.num_patches);
  double dy = 1.0/double(configData.num_xy - 1);

  size_t pos = 0;

  double y1 = 1.0;

  for (uint y = 0; y < configData.num_xy - 1; ++y) {
    double y2 = y1 - dy;

    double x1 = 0.0;

    for (uint x = 0; x < configData.num_patches; ++x, ++pos) {
      double x2 = x1 + dx;

      auto *face = faces[pos];

      std::vector<CPoint2D> points;

      if (y > 0 && y < configData.num_xy - 2) {
        points.push_back(CPoint2D(x2, y1));
        points.push_back(CPoint2D(x2, y2));
        points.push_back(CPoint2D(x1, y2));
        points.push_back(CPoint2D(x1, y1));
      }
      else {
        double x3 = (x1 + x2)/2;

        points.push_back(CPoint2D(x2, y1));
        points.push_back(CPoint2D(x3, y2));
        points.push_back(CPoint2D(x1, y1));
      }

      face->setTexturePoints(points);

      x1 = x2;
    }

    y1 = y2;
  }
}

void
CGeomSphere3D::
addNormalsI(CGeomObject3D *object, double radius)
{
  CSphere3D sphere(radius);

  FaceList &faces = object->getFaces();

  for (auto *face : faces) {
    for (const auto vind : face->getVertices()) {
      auto &vertex = object->getVertex(vind);

      auto p = vertex.getModel();

      auto n = sphere.pointNormal(CPoint3D(p.x, p.y, p.z));

      vertex.setNormal(n);
    }
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

  double dx = (twidth  - 1)/double(configData_.num_patches);
  double dy = (theight - 1)/double(configData_.num_xy - 1);

  size_t pos = 0;

  double y1 = theight - 1;

  for (uint y = 0; y < configData_.num_xy - 1; ++y) {
    double y2 = y1 - dy;

    double x1 = 0;

    for (uint x = 0; x < configData_.num_patches; ++x, ++pos) {
      double x2 = x1 + dx;

      CGeomMask *mask1 = mask->dup();

      faces[pos]->setMask(mask1);

      std::vector<CPoint2D> points;

      if (y > 0 && y < configData_.num_xy - 2) {
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
