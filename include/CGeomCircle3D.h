#ifndef CGEOM_CIRCLE_3D_H
#define CGEOM_CIRCLE_3D_H

#include <CGeomObject3D.h>

class CGeomCircle3D : public CGeomObject3D {
 public:
  enum { NUM_XY = 40 };

 public:
  struct ConfigData {
    ConfigData() { }

    uint   num_xy     { NUM_XY };
    double angleStart { 0.0 };
    double angleDelta { 2.0*M_PI };
  };

  static void addGeometry(CGeomObject3D *object, const CPoint3D &center,
                          double radius, const ConfigData &data=ConfigData());

 public:
  CGeomCircle3D(CGeomScene3D *pscene, const std::string &name,
                const CPoint3D &center, double radius, const ConfigData &data=ConfigData());

 ~CGeomCircle3D() { }

  CGeomCircle3D *dup() const override;

 private:
  CPoint3D   center_;
  ConfigData configData_;
};

#endif
