#ifndef CGEOM_CUBE_3D_H
#define CGEOM_CUBE_3D_H

#include <CGeomObject3D.h>

class CGeomCube3D : public CGeomObject3D {
 public:
  CGeomCube3D(CGeomScene3D *pscene, const std::string &name,
              const CPoint3D &c, double r);
  CGeomCube3D(CGeomScene3D *pscene, const std::string &name,
              double xc, double yc, double zc, double r);

 ~CGeomCube3D() { }

  static void addGeometry(CGeomObject3D *object, const CPoint3D &c, double r);
  static void addGeometry(CGeomObject3D *object, double xc, double yc, double zc, double r);
};

#endif
