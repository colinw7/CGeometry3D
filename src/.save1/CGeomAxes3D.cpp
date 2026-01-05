#include <CGeomAxes3D.h>

CGeomAxes3D::
CGeomAxes3D(CGeomScene3D *pscene, const std::string &name) :
 CGeomObject3D(pscene, name)
{
  uint vertices[6];

  vertices[0] = addVertex(CPoint3D( 0,  0,  1));
  vertices[1] = addVertex(CPoint3D( 0,  0, -1));
  vertices[2] = addVertex(CPoint3D( 0,  1,  0));
  vertices[3] = addVertex(CPoint3D( 0, -1,  0));
  vertices[4] = addVertex(CPoint3D( 1,  0,  0));
  vertices[5] = addVertex(CPoint3D(-1,  0,  0));

  for (int i = 0; i < 3; ++i) {
    auto v1 = vertices[2*i    ];
    auto v2 = vertices[2*i + 1];

    addLine(v1, v2);
  }
}
