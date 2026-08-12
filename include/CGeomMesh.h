#ifndef CGeomMesh_H
#define CGeomMesh_H

#include <CVector3D.h>

#include <map>
#include <vector>

class CGeomFace3D;
class CGeomLine3D;
class CGeomVertex3D;
class CGeomEdge3D;

struct CGeomMesh {
  CGeomMesh();
 ~CGeomMesh();

  void clear(bool destroy);

  using FaceList         = std::vector<CGeomFace3D *>;
  using FaceIList        = std::vector<uint>;
  using LineList         = std::vector<CGeomLine3D *>;
  using VertexList       = std::vector<CGeomVertex3D *>;
  using VertexFaceList   = std::map<uint, FaceIList>;
  using VertexFaceNormal = std::map<uint, CVector3D>;
  using TexturePoints    = std::vector<CPoint3D>;
  using Normals          = std::vector<CVector3D>;
  using EdgeFaces        = std::map<CGeomEdge3D *, FaceList>;
  using EdgeList         = std::vector<CGeomEdge3D *>;

  // geometry
  FaceList         faces;
  LineList         lines;
  VertexList       vertices;
  VertexFaceList   vertexFaceList;
  VertexFaceNormal vertexFaceNormal;
  TexturePoints    texturePoints;
  Normals          normals;

  // edges (dynamic)
  bool      edgesValid { false };
  EdgeFaces edgeFaces;
  EdgeList  edges;

  using VertexEdgeMap       = std::map<uint, CGeomEdge3D *>;
  using VertexVertexEdgeMap = std::map<uint, VertexEdgeMap>;

  VertexVertexEdgeMap vertexVertexEdgeMap;
  uint                edgeInd { 0 };
};

#endif
