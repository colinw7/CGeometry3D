#ifndef CGEOM_CYLINDER_3D_H
#define CGEOM_CYLINDER_3D_H

#include <CGeomObject3D.h>

class CGeomCylinder3D : public CGeomObject3D {
 public:
  enum { NUM_PATCHES = 20 };
  enum { NUM_XY      = 8 };

  enum CapType {
    NONE,
    NGON,
    FAN
  };

 public:
  struct ConfigData {
    ConfigData() { }

    CapType cap_type    { CapType::FAN };
    uint    num_patches { NUM_PATCHES };
    uint    num_xy      { NUM_XY };
  };

 public:
  static void addGeometry(CGeomObject3D *object, double xc, double yc, double zc,
                          double w, double h, const ConfigData &data=ConfigData());
  static void addGeometry(CGeomObject3D *object, const CPoint3D &center,
                          double w, double h, const ConfigData &data=ConfigData());

  static void addNormals(CGeomObject3D *object, double w, double h);

 public:
  CGeomCylinder3D(CGeomScene3D *pscene, const std::string &name,
                  double xc, double yc, double zc, double w, double h,
                  const ConfigData &data=ConfigData());
  CGeomCylinder3D(CGeomScene3D *pscene, const std::string &name,
                  const CPoint3D &center, double w, double h,
                  const ConfigData &data=ConfigData());

 ~CGeomCylinder3D() { }

  void mapTexture(CGeomTexture *texture) override;
  void mapTexture(CImagePtr image) override;

  void mapMask(CGeomMask *mask) override;
  void mapMask(CImagePtr image) override;

 private:
  CPoint3D   center_ { 0, 0, 0 };
  double     width_  { 1 };
  double     height_ { 1 };
  ConfigData configData_;
};

#endif
