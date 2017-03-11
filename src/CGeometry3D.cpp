#include <CGeometry3D.h>

CGeometry3D *
CGeometry3D::
getInstance()
{
  static CGeometry3D *instance;

  if (! instance)
    instance = new CGeometry3D;

  return instance;
}
