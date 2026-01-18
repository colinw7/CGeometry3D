#ifndef CGEOM_PLANE_3D_H
#define CGEOM_PLANE_3D_H

#include <CGeomObject3D.h>

class CGeomPlane3D : public CGeomObject3D {
 public:
  struct ConfigData {
    ConfigData() { }
  };

  static void addGeometry(CGeomObject3D *object, const CPoint3D &center,
                          double w, double h, const ConfigData &data=ConfigData());

 public:
  CGeomPlane3D(CGeomScene3D *pscene, const std::string &name,
                const CPoint3D &center, double w, double h, const ConfigData &data=ConfigData());

 ~CGeomPlane3D() { }

  CGeomPlane3D *dup() const override;

 private:
  CPoint3D   center_;
  double     w_ { 1.0 };
  double     h_ { 1.0 };
  ConfigData configData_;
};

#endif
