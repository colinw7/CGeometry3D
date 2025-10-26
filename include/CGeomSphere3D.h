#ifndef CGEOM_SPHERE_3D_H
#define CGEOM_SPHERE_3D_H

#include <CSphere3D.h>
#include <CGeomObject3D.h>

class CGeomSphere3D : public CGeomObject3D, public CSphere3D {
 public:
  enum { NUM_XY      = 40 };
  enum { NUM_PATCHES = 40 };

 public:
  struct ConfigData {
    ConfigData() { }

    uint   num_xy      { NUM_XY };
    uint   num_patches { NUM_PATCHES };
    double angleStart  { 0.0 };
    double angleDelta  { 2.0*M_PI };
  };

  static void addGeometry(CGeomObject3D *object, const CPoint3D &center,
                          double radius, const ConfigData &data=ConfigData());

  static void addTextureMap(CGeomObject3D *object, CGeomTexture *texture,
                            const ConfigData &data=ConfigData());

  static void addTexturePoints(CGeomObject3D *object, const ConfigData &data=ConfigData());

  static void addNormals(CGeomObject3D *object, double radius);

 public:
  CGeomSphere3D(CGeomScene3D *pscene, const std::string &name,
                const CPoint3D &center, double radius, const ConfigData &data=ConfigData());

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
                          const ConfigData &data);

  static void addTexturePointsI(CGeomObject3D *object, const ConfigData &data);

  static void addNormalsI(CGeomObject3D *object, double radius);

 private:
  CPoint3D   center_;
  ConfigData configData_;
};

#endif
