#ifndef CGEOM_UTIL_3D_H
#define CGEOM_UTIL_3D_H

#include <CPoint3D.h>
#include <CVector3D.h>
#include <CTriangle3D.h>
#include <CGeomVertex3D.h>
#include <vector>
#include <list>

class CGeomUtil3D {
 public:
  static CPoint3D  getMidPoint(const std::vector<CGeomVertex3D *> &vertices);
  static CVector3D getNormal  (const std::vector<CGeomVertex3D *> &vertices);

  static void triangulate(const std::list<CPoint3D> points,
                          std::vector<CTriangle3D> &triangle_list);
};

#endif
