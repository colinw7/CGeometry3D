#include <CGeomCone3D.h>
#include <CCone3D.h>

CGeomCone3D::
CGeomCone3D(CGeomScene3D *pscene, const std::string &name,
            double xc, double yc, double zc, double w, double h) :
 CGeomObject3D(pscene, name)
{
  addGeometry(this, xc, yc, zc, w, h, num_patches_);
}

CGeomCone3D::
CGeomCone3D(CGeomScene3D *pscene, const std::string &name,
            const CPoint3D &center, double w, double h) :
 CGeomObject3D(pscene, name)
{
  addGeometry(this, center, w, h, num_patches_);
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
  // create triangle edge (0, h/2) - > (-w/2, -h/2)
  int np = 7;

  std::vector<double> x, y;

  x.resize(np + 1);
  y.resize(np + 1);

  auto dx = 0.5*w/(np - 1);
  auto dy = h/(np - 1);

  double x1 = 0.0;
  double y1 = 0.5*h;

  for (int i = 0; i < np; ++i) {
    auto j = np - i;

    x[j] = x1;
    y[j] = y1;

    x1 += dx;
    y1 -= dy;
  }

  x[0] = 0.0; y[0] = -h/2.0;

  //---

  // rotate around y to add z (-w/2, w/2)
  BodyRevData bodyRevData;

  bodyRevData.uniquify = true;

  bodyRevData.tagInds[0 ] = 1;
  bodyRevData.tagInds[1 ] = 1;
  bodyRevData.tagInds[np] = 2;

  object->addBodyRev(&x[0], &y[0], np + 1, num_patches, bodyRevData);

  object->moveBy(CPoint3D(xc, yc, zc));
}

void
CGeomCone3D::
addNormals(CGeomObject3D *object, double w, double h)
{
  // cone (-w/2 -> w/2, -w/2 -> w/2, 0 -> h)
  CCone3D cone(0.5*w, h);

  //cone.translate(CPoint3D(0, 0, -height/2.0));

  // object has y/z flipped
  auto &faces = object->getFaces();

  for (auto *face : faces) {
    for (const auto vind : face->getVertices()) {
      auto &vertex = object->getVertex(vind);

      auto t = vertex.getTag();

      if      (t == 1)
        vertex.setNormal(CVector3D(0, -1, 0));
      else if (t == 2)
        vertex.setNormal(CVector3D(0,  1, 0));
      else {
        auto p = vertex.getModel();

        auto n = cone.pointNormal(CPoint3D(p.x, p.z, p.y));

        vertex.setNormal(CVector3D(n.getX(), n.getZ(), n.getY()));
      }
    }
  }
}
