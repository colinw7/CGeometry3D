#include <CGeomMesh.h>
#include <CGeomFace3D.h>
#include <CGeomEdge3D.h>

CGeomMesh::
CGeomMesh()
{
}

CGeomMesh::
~CGeomMesh()
{
  clear(/*destroy*/ true);
}

void
CGeomMesh::
clear(bool destroy)
{
  if (destroy) {
    for (auto *face : faces)
      delete face;

    for (auto *line : lines)
      delete line;

    for (auto *vertex : vertices)
      delete vertex;
  }

  for (auto &pe1 : vertexVertexEdgeMap)
    for (auto &pe2 : pe1.second)
      delete pe2.second;

  faces   .clear();
  lines   .clear();
  vertices.clear();

  vertexVertexEdgeMap.clear();

  vertexFaceList  .clear();
  vertexFaceNormal.clear();

  texturePoints.clear();

  normals.clear();

  edgesValid = false;
}
