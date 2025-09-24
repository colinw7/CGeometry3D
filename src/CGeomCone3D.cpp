#include <CGeomCone3D.h>
#include <CCone3D.h>

CGeomCone3D::
CGeomCone3D(CGeomScene3D *pscene, const std::string &name,
            double xc, double yc, double zc, double w, double h) :
 CGeomObject3D(pscene, name)
{
  addGeometry(this, xc, yc, zc, w, h, uint(num_patches_));
}

CGeomCone3D::
CGeomCone3D(CGeomScene3D *pscene, const std::string &name,
            const CPoint3D &center, double w, double h) :
 CGeomObject3D(pscene, name)
{
  addGeometry(this, center, w, h, uint(num_patches_));
}

void
CGeomCone3D::
addGeometry(CGeomObject3D *object, const CPoint3D &center,
            double w, double h, uint num_patches)
{
  addGeometry(object, center.x, center.y, center.z, w, h, num_patches);
}

void
CGeomCone3D::
addGeometry(CGeomObject3D *object, double xc, double yc, double zc,
            double w, double h, uint num_patches)
{
  int np = 7;

  std::vector<double> x, y;

  x.resize(np + 1);
  y.resize(np + 1);

  auto dx = w/(np - 1);
  auto dy = h/(np - 1);

  double x1 = 0.0;
  double y1 = +h/2.0;

  for (int i = 0; i < np; ++i) {
    x[i] = x1; y[i] = y1;

    x1 += dx;
    y1 -= dy;
  }

  x[np] = 0.0; y[np] = -h/2.0;

  object->addBodyRev(&x[0], &y[0], np + 1, num_patches);

  object->moveBy(CPoint3D(xc, yc, zc));
}

void
CGeomCone3D::
addNormals(CGeomObject3D *object, double w, double h)
{
  CCone3D cone(w/2.0, h);

  //cone.translate(CPoint3D(0, 0, -height/2.0));

  FaceList &faces = object->getFaces();

  for (auto *face : faces) {
    for (const auto vind : face->getVertices()) {
      auto &vertex = object->getVertex(vind);

      auto p = vertex.getModel();

      auto n = cone.pointNormal(CPoint3D(p.x, p.z, p.y));

      vertex.setNormal(n);
    }
  }
}
