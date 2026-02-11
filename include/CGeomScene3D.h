#ifndef CGEOM_SCENE_3D_H
#define CGEOM_SCENE_3D_H

#include <CGeomObject3D.h>
#include <CGeomLight3D.h>
#include <CGeomAxes3D.h>
#include <CGeomZBuffer.h>
#include <CGeomCamera3D.h>
#include <CGeomTexture.h>

class CGeomScene3D {
 public:
  enum DrawType {
    WIRE_FRAME,
    SOLID
  };

 public:
  using ObjectList = std::vector<CGeomObject3D *>;
  using Textures   = CGeomTextureMgr::Textures;

 public:
  CGeomScene3D();
 ~CGeomScene3D();

  void setRenderer(CGeom3DRenderer *renderer);

  CGeomZBuffer *getZBuffer() const { return zbuffer_.get(); }

  CGeomCamera3D *getCamera() const { return camera_.get(); }

  bool isUseZBuffer() const { return useZBuffer_; }
  void setUseZBuffer(bool b) { useZBuffer_ = b; }

  DrawType getDrawType() const { return drawType_; }
  DrawType setDrawType(DrawType drawType) { std::swap(drawType_, drawType); return drawType; }

  //----

  // primitives

  void addPrimitive(CGeomObject3D *object);

  uint getNumPrimitives() const { return uint(primitives_.size()); }

  CGeomObject3D &getPrimitive (uint i) const { return *primitives_[i]; }
  CGeomObject3D *getPrimitiveP(uint i) const { return  primitives_[i]; }

  CGeomObject3D &getPrimitive (const std::string &name) const;
  CGeomObject3D *getPrimitiveP(const std::string &name) const;

  //----

  // objects

  void addObject   (CGeomObject3D *object);
  void removeObject(CGeomObject3D *object, bool force=false);

  uint getNumObjects() const { return uint(objects_.size()); }

  const ObjectList &getObjects() const { return objects_; }

  CGeomObject3D &getObject (uint i) const { return *objects_[i]; }
  CGeomObject3D *getObjectP(uint i) const { return  objects_[i]; }

  CGeomObject3D &getObject (const std::string &name) const;
  CGeomObject3D *getObjectP(const std::string &name) const;

  CGeomObject3D *getObjectByInd(uint ind) const;

  uint getNumSelectedObjects() const;

  ObjectList getSelectedObjects() const;

  //----

  // lights

  void addLight(CGeomLight3D *light);

  uint getNumLights() const;

  CGeomLight3D *getLight(uint i);

  void lightsMoveX(double dx);
  void lightsMoveY(double dy);
  void lightsMoveZ(double dz);

  void lightsRotateX(double dx);
  void lightsRotateY(double dy);
  void lightsRotateZ(double dz);

  //----

  // bbox
  void getBBox(CBBox3D &bbox) const;

  CPoint3D getCenter() const;

  void setCenter(const CPoint3D &point);

  CVector3D getSize() const;

  //---

  // camera
  void initCamera();

  void cameraMoveX(double dx);
  void cameraMoveY(double dy);
  void cameraMoveZ(double dz);

  void cameraRotateX(double dx);
  void cameraRotateY(double dy);
  void cameraRotateZ(double dz);

  //---

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

  void objectsScale(double factor);

  //---

  // draw

  void drawInit();
  void drawExec();
  void drawTerm();

  void drawWireframe();
  void drawWireframeZ();
  void drawSolid();
  void drawSolidZ();

  //---

  void modelToPixel();

  CGeomFace3D *getFaceAt(int x, int y);

  double getZAt(int x, int y);

  bool lightPoint(const CPoint3D &point, const CVector3D &normal,
                  const CGeomMaterial &material, CRGBA &rgba) const;

  //---

  // materials

  void addMaterial(CGeomMaterial *material) {
    materialMgr_.addMaterial(material);
  }

  CGeomMaterial *getMaterial(const std::string &name) const {
    return materialMgr_.getMaterial(name);
  }

  std::vector<CGeomMaterial *> getMaterials() const {
    return materialMgr_.getMaterials();
  }

  CGeomMaterial *getMaterialById(uint id) const {
    return materialMgr_.getMaterialById(id);
  }
  //---

  // textures
  const Textures &textures() const {
    return textureMgr_.textures();
  }

  void addTexture(CGeomTexture *texture) {
    textureMgr_.addTexture(texture);
  }

  CGeomTexture *getTextureByName(const std::string &name) const {
    return textureMgr_.getTextureByName(name);
  }

  CGeomTexture *getTextureById(int id) const {
    return textureMgr_.getTextureById(id);
  }

 private:
  CGeomScene3D(const CGeomScene3D &rhs);
  CGeomScene3D &operator=(const CGeomScene3D &rhs);

 private:
  using ObjectMap = std::map<std::string, CGeomObject3D *>;
  using ZBufferP  = std::unique_ptr<CGeomZBuffer>;
  using CameraP   = std::unique_ptr<CGeomCamera3D>;
  using AxesP     = std::unique_ptr<CGeomAxes3D>;
  using MaterialP = std::unique_ptr<CGeomMaterial>;

  ObjectMap        primitiveMap_;
  ObjectList       primitives_;
  ObjectMap        objectMap_;
  ObjectList       objects_;
  CGeom3DRenderer* renderer_ { nullptr };
  ZBufferP         zbuffer_;
  bool             useZBuffer_ { true };
  CameraP          camera_;
  CGeomLight3DMgr  lightMgr_;
  CGeomMaterialMgr materialMgr_;
  CGeomTextureMgr  textureMgr_;
  DrawType         drawType_ { WIRE_FRAME };
  CBBox3D          bbox_;
  AxesP            axes_;
};

//---

class CGeomScene3DVisitor {
 public:
  CGeomScene3DVisitor(CGeomScene3D *scene);

  virtual ~CGeomScene3DVisitor() { }

  //---

  bool isOnlyVisible() const { return onlyVisible_; }
  void setOnlyVisible(bool b) { onlyVisible_ = b; }

  //---

  void visit();

  virtual bool beginVisitObject(CGeomObject3D *) { return true; }
  virtual void endVisitObject  (CGeomObject3D *) { }

  virtual bool beginVisitFace(CGeomFace3D *) { return true; }
  virtual void endVisitFace  (CGeomFace3D *) { }

  virtual void visitFaceVertex(CGeomVertex3D *) { }

  virtual void visitLine(CGeomLine3D *) { }

  virtual bool beginVisitSubFace(CGeomFace3D *) { return true; }
  virtual void endVisitSubFace  (CGeomFace3D *) { }

  virtual void visitSubFaceVertex(CGeomVertex3D *) { }

  virtual void visitSubLine(CGeomLine3D *) { }

 protected:
  CGeomScene3D* scene_       { nullptr };
  bool          onlyVisible_ { true };

  CGeomObject3D* object_  { nullptr };
  CGeomFace3D*   face_    { nullptr };
  CGeomLine3D*   line_    { nullptr };
  CGeomFace3D*   subFace_ { nullptr };
  CGeomLine3D*   subLine_ { nullptr };
};

#endif
