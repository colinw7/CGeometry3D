#ifndef CGEOM_SCENE_3D_H
#define CGEOM_SCENE_3D_H

#include <CGeomObject3D.h>
#include <CGeomLight3D.h>
#include <CGeomAxes3D.h>
#include <CGeomZBuffer.h>
#include <CGeomCamera3D.h>
#include <CAutoPtr.h>

class CGeomScene3D {
 public:
  enum DrawType {
    WIRE_FRAME,
    SOLID
  };

 public:
  typedef std::vector<CGeomObject3D *> ObjectList;

 public:
  CGeomScene3D();
 ~CGeomScene3D() { }

  void setRenderer(CGeom3DRenderer *renderer);

  CGeomZBuffer *getZBuffer() const { return zbuffer_; }

  CGeomCamera3D *getCamera() const { return camera_; }

  ACCESSOR(UseZBuffer, bool, use_zbuffer)

  DrawType getDrawType() const { return draw_type_; }

  DrawType setDrawType(DrawType draw_type) {
    std::swap(draw_type_, draw_type);

    return draw_type;
  }

  void addPrimitive(CGeomObject3D *object);

  uint getNumPrimitives() const { return primitives_.size(); }

  CGeomObject3D &getPrimitive (uint i) const { return *primitives_[i]; }
  CGeomObject3D *getPrimitiveP(uint i) const { return  primitives_[i]; }

  CGeomObject3D &getPrimitive (const std::string &name) const;
  CGeomObject3D *getPrimitiveP(const std::string &name) const;

  void addObject   (CGeomObject3D *object);
  void removeObject(CGeomObject3D *object);

  uint getNumObjects() const { return objects_.size(); }

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
  typedef std::map<std::string, CGeomObject3D *> ObjectMap;

  ObjectMap                primitive_map_;
  ObjectList               primitives_;
  ObjectMap                object_map_;
  ObjectList               objects_;
  CGeom3DRenderer         *renderer_ { nullptr };
  CAutoPtr<CGeomZBuffer>   zbuffer_;
  bool                     use_zbuffer_ { true };
  CAutoPtr<CGeomCamera3D>  camera_;
  CGeomLight3DMgr          light_mgr_;
  DrawType                 draw_type_ { WIRE_FRAME };
  CBBox3D                  bbox_;
  CAutoPtr<CGeomAxes3D>    axes_;
};

#endif
