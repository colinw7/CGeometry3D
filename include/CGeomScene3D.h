#ifndef CGEOM_SCENE_3D_H
#define CGEOM_SCENE_3D_H

#include <CGeomObject3D.h>
#include <CGeomLight3D.h>
#include <CGeomAxes3D.h>
#include <CGeomZBuffer.h>
#include <CGeomCamera3D.h>

class CGeomScene3D {
 public:
  enum DrawType {
    WIRE_FRAME,
    SOLID
  };

 public:
  using ObjectList = std::vector<CGeomObject3D *>;

 public:
  CGeomScene3D();
 ~CGeomScene3D() { }

  void setRenderer(CGeom3DRenderer *renderer);

  CGeomZBuffer *getZBuffer() const { return zbuffer_.get(); }

  CGeomCamera3D *getCamera() const { return camera_.get(); }

  bool isUseZBuffer() const { return useZBuffer_; }
  void setUseZBuffer(bool b) { useZBuffer_ = b; }

  DrawType getDrawType() const { return drawType_; }
  DrawType setDrawType(DrawType drawType) { std::swap(drawType_, drawType); return drawType; }

  void addPrimitive(CGeomObject3D *object);

  uint getNumPrimitives() const { return uint(primitives_.size()); }

  CGeomObject3D &getPrimitive (uint i) const { return *primitives_[i]; }
  CGeomObject3D *getPrimitiveP(uint i) const { return  primitives_[i]; }

  CGeomObject3D &getPrimitive (const std::string &name) const;
  CGeomObject3D *getPrimitiveP(const std::string &name) const;

  void addObject   (CGeomObject3D *object);
  void removeObject(CGeomObject3D *object, bool force=false);

  uint getNumObjects() const { return uint(objects_.size()); }

  const ObjectList &getObjects() const { return objects_; }

  uint getNumSelectedObjects() const;

  ObjectList getSelectedObjects() const;

  CGeomObject3D &getObject (uint i) const { return *objects_[i]; }
  CGeomObject3D *getObjectP(uint i) const { return  objects_[i]; }

  CGeomObject3D &getObject (const std::string &name) const;
  CGeomObject3D *getObjectP(const std::string &name) const;

  void addLight(CGeomLight3D *light);

  uint getNumLights() const;

  CGeomLight3D *getLight(uint i);

  void getBBox(CBBox3D &bbox) const;

  CPoint3D getCenter() const;

  void setCenter(const CPoint3D &point);

  CVector3D getSize() const;

  void initCamera();

  void cameraMoveX(double dx);
  void cameraMoveY(double dy);
  void cameraMoveZ(double dz);

  void cameraRotateX(double dx);
  void cameraRotateY(double dy);
  void cameraRotateZ(double dz);

  void lightsMoveX(double dx);
  void lightsMoveY(double dy);
  void lightsMoveZ(double dz);

  void lightsRotateX(double dx);
  void lightsRotateY(double dy);
  void lightsRotateZ(double dz);

  void objectsMove(const CPoint3D &delta);

  void objectsMoveX(double dx);
  void objectsMoveY(double dy);
  void objectsMoveZ(double dz);

  void objectsRotate(const CPoint3D &angles);

  void objectsRotateX(double dx);
  void objectsRotateY(double dy);
  void objectsRotateZ(double dz);

  void objectsResizeX(double dx);
  void objectsResizeY(double dy);
  void objectsResizeZ(double dz);

  void drawInit();
  void drawExec();
  void drawTerm();

  void drawWireframe();
  void drawWireframeZ();
  void drawSolid();
  void drawSolidZ();

  void modelToPixel();

  void objectsScale(double factor);

  CGeomFace3D *getFaceAt(int x, int y);

  double getZAt(int x, int y);

  bool lightPoint(const CPoint3D &point, const CVector3D &normal,
                  const CMaterial &material, CRGBA &rgba) const;

 private:
  CGeomScene3D(const CGeomScene3D &rhs);
  CGeomScene3D &operator=(const CGeomScene3D &rhs);

 private:
  using ObjectMap = std::map<std::string, CGeomObject3D *>;
  using ZBufferP  = std::unique_ptr<CGeomZBuffer>;
  using CameraP   = std::unique_ptr<CGeomCamera3D>;
  using AxesP     = std::unique_ptr<CGeomAxes3D>;

  ObjectMap        primitive_map_;
  ObjectList       primitives_;
  ObjectMap        object_map_;
  ObjectList       objects_;
  CGeom3DRenderer* renderer_ { nullptr };
  ZBufferP         zbuffer_;
  bool             useZBuffer_ { true };
  CameraP          camera_;
  CGeomLight3DMgr  light_mgr_;
  DrawType         drawType_ { WIRE_FRAME };
  CBBox3D          bbox_;
  AxesP            axes_;
};

#endif
