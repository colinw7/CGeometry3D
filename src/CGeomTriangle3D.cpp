#include <CGeomTriangle3D.h>
#include <CGeomTexture.h>
#include <CGeomMask.h>

#define NUM_XY      40
#define NUM_PATCHES 40

CGeomTriangle3D::
CGeomTriangle3D(CGeomScene3D *pscene, const std::string &name,
                const CPoint3D &point1, const CPoint3D &point2, const CPoint3D &point3) :
 CGeomObject3D(pscene, name), CTriangle3D(point1, point2, point3)
{
}

CGeomTriangle3D::
~CGeomTriangle3D()
{
}

CGeomTriangle3D *
CGeomTriangle3D::
dup() const
{
  return new CGeomTriangle3D(*this);
}
