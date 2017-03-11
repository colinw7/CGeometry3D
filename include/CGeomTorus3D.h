#ifndef CGEOM_TORUS_3D_H
#define CGEOM_TORUS_3D_H

#include <CGeomObject3D.h>

class CGeomTorus3D : public CGeomObject3D {
 public:
  CGeomTorus3D(CGeomScene3D *pscene, const std::string &name,
               double xc, double yc, double zc, double r0, double r1,
               double p1, double p2, int nu, int nv);

 ~CGeomTorus3D() { }

  static void addGeometry(CGeomObject3D *object,
                          double xc, double yc, double zc,
                          double r0, double r1,
                          double p1, double p2, int nu, int nv);

 private:
  static double power(double f, double p);
};

#endif
