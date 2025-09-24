#ifndef CGEOM_SPHERE_3D_H
#define CGEOM_SPHERE_3D_H

#include <CSphere3D.h>
#include <CGeomObject3D.h>

class CGeomSphere3D : public CGeomObject3D, public CSphere3D {
 public:
  enum { NUM_XY      = 40 };
  enum { NUM_PATCHES = 40 };

 public:
  static void addGeometry(CGeomObject3D *object, const CPoint3D &center,
                          double radius, uint num_xy=NUM_XY,
                          uint num_patches=NUM_PATCHES);

  static void addTextureMap(CGeomObject3D *object, CGeomTexture *texture,
                            uint num_xy=NUM_XY, uint num_patches=NUM_PATCHES);

  static void addTexturePoints(CGeomObject3D *object,
                               uint num_xy=NUM_XY, uint num_patches=NUM_PATCHES);

  static void addNormals(CGeomObject3D *object, double radius);

 public:
  CGeomSphere3D(CGeomScene3D *pscene, const std::string &name,
                const CPoint3D &center, double radius);

 ~CGeomSphere3D() { }

  CGeomSphere3D *dup() const override;

  //---

  void mapTexture(CGeomTexture *texture) override;
  void mapTexture(CImagePtr image) override;

  void addTexturePoints();

  void mapMask(CGeomMask *mask) override;
  void mapMask(CImagePtr image) override;

  void addNormals();

 private:
  static void mapTextureI(CGeomObject3D *object, CGeomTexture *texture,
                          uint num_xy, uint num_patches);

  static void addTexturePointsI(CGeomObject3D *object, uint num_xy, uint num_patches);

  static void addNormalsI(CGeomObject3D *object, double radius);

 private:
  CPoint3D center_;
  uint     num_xy_      { NUM_XY };
  uint     num_patches_ { NUM_PATCHES };
};

#endif
