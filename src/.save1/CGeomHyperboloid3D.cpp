#include <CGeomHyperboloid3D.h>
#include <CHyperboloid3D.h>

CGeomHyperboloid3D::
CGeomHyperboloid3D(CGeomScene3D *pscene, const std::string &name,
                   const CPoint3D &point1, const CPoint3D &point2) :
 CGeomObject3D(pscene, name)
{
  addGeometry(this, point1, point2, num_patches_);
}

void
CGeomHyperboloid3D::
addGeometry(CGeomObject3D *object, const CPoint3D &point1, const CPoint3D &point2,
            int num_patches)
{
  CHyperboloid3D hyperboloid(point1, point2);

  int np = 8;

  double f = 1.0/(np - 1);

  std::vector<double> x, y;

  x.resize(np);
  y.resize(np);

  for (int i = 0; i < np; ++i) {
    x[i] = point1.x + i*(point2.x - point1.x)*f;
    y[i] = point1.y + i*(point2.y - point1.y)*f;
  }

  object->addBodyRev(&x[0], &y[0], np, num_patches);
}

void
CGeomHyperboloid3D::
addNormals(CGeomObject3D *object, const CPoint3D &point1, const CPoint3D &point2)
{
  CHyperboloid3D hyperboloid(point1, point2);

  FaceList &faces = object->getFaces();

  for (auto *face : faces) {
    for (const auto vind : face->getVertices()) {
      auto &vertex = object->getVertex(vind);

      auto p = vertex.getModel();

      auto n = hyperboloid.pointNormal(CPoint3D(p.x, p.z, p.y));

      vertex.setNormal(n);
    }
  }
}
