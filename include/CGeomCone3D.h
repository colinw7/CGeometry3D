#ifndef CGEOM_CONE_3D_H
#define CGEOM_CONE_3D_H

#include <CGeomObject3D.h>

class CGeomCone3D : public CGeomObject3D {
 public:
  enum { NUM_PATCHES = 20 };

  CGeomCone3D(CGeomScene3D *pscene, const std::string &name,
              double xc, double yc, double zc, double w, double h);

 ~CGeomCone3D() { }

  static void addGeometry(CGeomObject3D *object, double xc, double yc, double zc,
                          double w, double h, uint num_patches=NUM_PATCHES);

 private:
  int num_patches_ { NUM_PATCHES };
};

#endif
