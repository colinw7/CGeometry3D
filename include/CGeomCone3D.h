#ifndef CGEOM_CONE_3D_H
#define CGEOM_CONE_3D_H

#include <CGeomObject3D.h>

class CGeomCone3D : public CGeomObject3D {
 public:
  enum { NUM_PATCHES = 20 };

 public:
  static void addGeometry(CGeomObject3D *object, double xc, double yc, double zc,
                          double w, double h, uint num_patches=NUM_PATCHES);
  static void addGeometry(CGeomObject3D *object, const CPoint3D &center,
                          double w, double h, uint num_patches=NUM_PATCHES);

  static void addNormals(CGeomObject3D *object, double w, double h);

 public:
  CGeomCone3D(CGeomScene3D *pscene, const std::string &name,
              const CPoint3D &center, double w, double h);
  CGeomCone3D(CGeomScene3D *pscene, const std::string &name,
              double xc, double yc, double zc, double w, double h);

 ~CGeomCone3D() { }

 private:
  int num_patches_ { NUM_PATCHES };
};

#endif
