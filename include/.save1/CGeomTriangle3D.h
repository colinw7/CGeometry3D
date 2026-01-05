#ifndef CGEOM_TRIANGLE_3D_H
#define CGEOM_TRIANGLE_3D_H

#include <CTriangle3D.h>
#include <CGeomObject3D.h>

class CGeomTriangle3D : public CGeomObject3D, public CTriangle3D {
 public:
  CGeomTriangle3D(CGeomScene3D *pscene, const std::string &name,
                  const CPoint3D &point1, const CPoint3D &point2,
                  const CPoint3D &point3);

 ~CGeomTriangle3D();

  CGeomTriangle3D *dup() const override;
};

#endif
