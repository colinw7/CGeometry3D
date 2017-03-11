#ifndef CGEOM_BOX_3D_H
#define CGEOM_BOX_3D_H

#include <CGeomObject3D.h>

class CGeomBox3D : public CGeomObject3D {
 public:
  CGeomBox3D(CGeomScene3D *pscene, const std::string &name,
             double xc, double yc, double zc, double xs, double ys, double zs);

 ~CGeomBox3D() { }

  static void addGeometry(CGeomObject3D *object, double xc, double yc, double zc,
                          double xs, double ys, double zs);
};

#endif
