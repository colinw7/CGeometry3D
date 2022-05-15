#include <CGeomCone3D.h>

CGeomCone3D::
CGeomCone3D(CGeomScene3D *pscene, const std::string &name,
            double xc, double yc, double zc, double w, double h) :
 CGeomObject3D(pscene, name)
{
  addGeometry(this, xc, yc, zc, w, h, uint(num_patches_));
}

void
CGeomCone3D::
addGeometry(CGeomObject3D *object, double xc, double yc, double zc,
            double w, double h, uint num_patches)
{
  double x[3], y[3];

  x[0] =  0; y[0] = +h/2;
  x[1] = +w; y[1] = -h/2;
  x[2] =  0; y[2] = -h/2;

  object->addBodyRev(x, y, 3, num_patches);

  object->moveBy(CPoint3D(xc, yc, zc));
}
