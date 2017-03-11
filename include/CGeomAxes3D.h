#ifndef CGEOM_AXES_3D_H
#define CGEOM_AXES_3D_H

#include <CGeomObject3D.h>

class CGeomAxes3D : public CGeomObject3D {
 public:
  CGeomAxes3D(CGeomScene3D *pscene, const std::string &name);

 ~CGeomAxes3D() { }
};

#endif
