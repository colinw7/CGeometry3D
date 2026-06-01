#ifndef FaceData_H
#define FaceData_H

#include <CRGBA.h>
#include <CPoint3D.h>
#include <vector>

class CGeomFace3D;
class CGeomLine3D;

class CQGLTexture;

namespace CQTclParticle3D {

struct FaceData {
  CGeomFace3D*     face            { nullptr };
  CGeomLine3D*     line            { nullptr };
  bool             selected        { false };
  bool             visible         { true };
  bool             wireframe       { false };
  int              lineWidth       { -1 };
  bool             lines           { false };
  int              pos             { 0 };
  int              len             { 0 };
  CPoint3D         position        { 0, 0, 0 };
  CQGLTexture*     diffuseTexture  { nullptr };
  CQGLTexture*     normalTexture   { nullptr };
  CQGLTexture*     specularTexture { nullptr };
  CQGLTexture*     emissiveTexture { nullptr };
  CRGBA            ambient         { 0, 0, 0 };
  CRGBA            diffuse         { 1, 1, 1 };
  CRGBA            specular        { 0, 0, 0 };
  CRGBA            emission        { 0, 0, 0, 0 };
  double           shininess       { 1.0 };
  double           alpha           { 1.0 };
  std::vector<int> vertices;
};

struct FaceDataList {
  void clear() { faceDatas.clear(); pos = 0; }

  void add(const FaceDataList &faceDataList) {
    for (const auto &faceData : faceDataList.faceDatas)
      faceDatas.push_back(faceData);
  }

  int                   pos { 0 };
  std::vector<FaceData> faceDatas;
};

}

#endif
