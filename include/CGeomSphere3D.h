#ifndef CGEOM_SPHERE_3D_H
#define CGEOM_SPHERE_3D_H

#include <CSphere3D.h>
#include <CGeomObject3D.h>

class CGeomSphere3D : public CGeomObject3D, public CSphere3D {
 public:
  enum { NUM_XY      = 40 };
  enum { NUM_PATCHES = 40 };

  CGeomSphere3D(CGeomScene3D *pscene, const std::string &name,
                const CPoint3D &center, double radius);

 ~CGeomSphere3D() { }

  CGeomSphere3D *dup() const override;

  static void addGeometry(CGeomObject3D *object, const CPoint3D &center,
                          double radius, uint num_xy=NUM_XY,
                          uint num_patches=NUM_PATCHES);

  void mapTexture(CGeomTexture *texture) override;
  void mapTexture(CImagePtr image) override;

  void mapMask(CGeomMask *mask) override;
  void mapMask(CImagePtr image) override;

 private:
  CPoint3D center_;
  uint     num_xy_ { 0 };
  uint     num_patches_ { 0 };
};

#endif
