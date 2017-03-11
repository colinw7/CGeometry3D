#ifndef CGEOM_CYLINDER_3D_H
#define CGEOM_CYLINDER_3D_H

#include <CGeomObject3D.h>

class CGeomCylinder3D : public CGeomObject3D {
 public:
  enum { NUM_PATCHES = 20 };

  CGeomCylinder3D(CGeomScene3D *pscene, const std::string &name,
                  double xc, double yc, double zc, double w, double h);

 ~CGeomCylinder3D() { }

  static void addGeometry(CGeomObject3D *object, double xc, double yc, double zc,
                          double w, double h, uint num_patches=NUM_PATCHES);

  void mapTexture(CGeomTexture *texture);
  void mapTexture(CImagePtr image);

  void mapMask(CGeomMask *mask);
  void mapMask(CImagePtr image);

 private:
  uint num_patches_ { NUM_PATCHES };
};

#endif
