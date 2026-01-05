#ifndef CGEOM_HYPERBOLOID_3D_H
#define CGEOM_HYPERBOLOID_3D_H

#include <CGeomObject3D.h>

class CGeomHyperboloid3D : public CGeomObject3D {
 public:
  enum { NUM_PATCHES = 20 };

 public:
  static void addGeometry(CGeomObject3D *object, const CPoint3D &point1, const CPoint3D &point2,
                          int num_patches=NUM_PATCHES);

  static void addNormals(CGeomObject3D *object, const CPoint3D &point1, const CPoint3D &point2);

 public:
  CGeomHyperboloid3D(CGeomScene3D *pscene, const std::string &name,
                     const CPoint3D &point1, const CPoint3D &point2);

 ~CGeomHyperboloid3D() { }

 private:
  int num_patches_ { NUM_PATCHES };
};

#endif
