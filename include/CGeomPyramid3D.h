#ifndef CGEOM_PYRAMID_3D_H
#define CGEOM_PYRAMID_3D_H

#include <CGeomObject3D.h>

class CGeomPyramid3D : public CGeomObject3D {
 public:
  CGeomPyramid3D(CGeomScene3D *pscene, const std::string &name,
                 double xc, double yc, double zc, double w, double h);
  CGeomPyramid3D(CGeomScene3D *pscene, const std::string &name,
                 const CPoint3D &c, double w, double h);

 ~CGeomPyramid3D() { }

  static void addGeometry(CGeomObject3D *object,
                          const CPoint3D &c, double w, double h);
  static void addGeometry(CGeomObject3D *object,
                          double xc, double yc, double zc, double w, double h);

 private:
  void addGeometry(double xc, double yc, double zc, double w, double h);
};

#endif
