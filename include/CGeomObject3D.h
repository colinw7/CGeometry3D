#ifndef CGEOM_OBJECT_3D_H
#define CGEOM_OBJECT_3D_H

#include <CGeomFace3D.h>
#include <CGeomPoint3D.h>
#include <CCoordFrame3D.h>
#include <CBBox3D.h>
#include <CVector4D.h>
#include <CTranslate3D.h>
#include <CScale3D.h>
#include <CRotate3D.h>
#include <map>

class CGeomScene3D;
class CGeomCamera;
class CGeomZBuffer;
class CGeom3DRenderer;
class CGeomAnimationData;
class CGeomNodeData;

// If override need to implement constructor, copy constructor, destructor, dup

class CGeomObject3D {
 public:
  using FaceList         = std::vector<CGeomFace3D *>;
  using FaceIList        = std::vector<uint>;
  using LineList         = std::vector<CGeomLine3D *>;
  using VertexList       = std::vector<CGeomVertex3D *>;
  using VertexIList      = std::vector<uint>;
  using VertexFaceList   = std::map<uint, FaceIList>;
  using VertexFaceNormal = std::map<uint, CVector3D>;

  enum Transform {
    NONE,
    ROTATION,
    TRANSLATION,
    SCALE
  };

  class Group {
   public:
    Group(const std::string &name, uint id) :
     name_(name), id_(id) {
    }

    uint id() const { return id_; }

    void addFace(uint faceNum) { faceList_.push_back(faceNum); }

   private:
    std::string name_;
    uint        id_ { 0 };
    FaceIList   faceList_;
  };

  using Groups = std::map<std::string,Group>;

  //---

  using NodeDatas = std::map<int, CGeomNodeData>;
  using NodeIds   = std::vector<int>;

 public:
  CGeomObject3D(CGeomScene3D *pscene, const std::string &name);

  CGeomObject3D(const CGeomObject3D &object);

  CGeomObject3D &operator=(const CGeomObject3D &rhs) = delete;

  virtual ~CGeomObject3D();

  virtual CGeomObject3D *dup() const;

  //---

  void setScene(CGeomScene3D *scene) { pscene_ = scene; }
  CGeomScene3D *getScene() const { return pscene_; }

  //---

  const uint &getInd() const { return ind_; }
  void setInd(const uint &i) { ind_ = i; }

  const std::string &getName() const { return name_; }
  void setName(const std::string &name) { name_ = name; }

  const std::string &getId() const { return id_; }
  void setId(const std::string &id) { id_ = id; }

  const std::string &getMeshName() const { return meshName_; }
  void setMeshName(const std::string &name) { meshName_ = name; }

  //---

  bool getSelected() const { return selected_; }
  void setSelected(bool b) { selected_ = b; }

  bool getHierSelected() const;

  bool getVisible() const { return visible_; }
  void setVisible(bool b) { visible_ = b; }

  //---

  bool isDrawPosition() const { return drawPosition_; }
  void setDrawPosition(bool b) { drawPosition_ = b; }

  //---

  CGeomObject3D *parent() const { return parent_; }
  const std::vector<CGeomObject3D *> &children() const { return children_; }

  void addChild(CGeomObject3D *child);

  void resetHier();
  void resetChildren();

  //---

  CPoint3D getPosition() const { return coordFrame_.getOriginPoint(); }

  const CGeomPoint3D &getPositionPoint() const { return position_; }

  CGeomPoint3D &editPositionPoint() { return position_; }

  void setAbsPosition(const CPoint3D &point) {
    position_.setModel  (point);
    position_.setCurrent(point);
    position_.setViewed (point);
  }

  void setPosition(const CPoint3D &point) {
    coordFrame_.setOrigin(point);

    updatePosition();
  }

  void setPositionPoint(const CGeomPoint3D &point) {
    position_ = point;

    setPosition(point.getCurrent());
  }

  void updatePosition() {
    position_.setModel  (coordFrame_.getOriginPoint());
    position_.setCurrent(coordFrame_.getOriginPoint());
  }

  //---

  CPoint3D getModelCenter() const;

  void setModelCenter(const CPoint3D &point);

  CVector3D getModelSize() const;

  //---

  void setTexture(CGeomTexture *texture);
  void setTexture(CImagePtr image);

  virtual void mapTexture(CGeomTexture *texture);
  virtual void mapTexture(CImagePtr image);

  //---

  void setMask(CGeomMask *mask);
  void setMask(CImagePtr image);

  virtual void mapMask(CGeomMask *mask);
  virtual void mapMask(CImagePtr image);

  //---

  void setFaceFlags(uint flags);

  void unsetFaceFlags(uint flags);

  //---

  bool findVertex(const CPoint3D &point, uint *ind);

  uint addVertex(const CPoint3D &point);
  uint addVertex(CGeomVertex3D *vertex);

  void addVertexFace(uint vertex_ind, uint face_num);

  uint dupVertex(uint vind);

  //---

  uint getNumTextuePoints() const { return uint(texturePoints_.size()); }

  uint addTexturePoint(const CPoint2D &point);
  uint addTexturePoint(const CPoint3D &point);
  const CPoint3D &texturePoint(uint i) const;

  //---

  uint getNumNormals() const { return uint(normals_.size()); }

  uint addNormal(const CVector3D &point);
  const CVector3D &normal(uint i) const;

  //---

  const CMatrix3D &viewMatrix() const { return viewMatrix_; }
  void setViewMatrix(const CMatrix3D &v) { viewMatrix_ = v; }

  //---

  uint addLine(uint start, uint end);
  uint addLine(CGeomLine3D *line);

  //---

  std::vector<uint> addITriangles(uint *inds, uint num_inds);

  uint addITriangle(uint i1, uint i2, uint i3);

  //---

  uint addIPolygon(uint *inds, uint num_inds);

  //---

  uint addFace(const VertexIList &vertices);
  uint addFace(CGeomFace3D *face);

  //---

  uint addFaceSubFace(uint face_num, const std::vector<uint> &vertices);
  uint addFaceSubLine(uint face_num, uint start, uint end);

  //---

  const VertexList &getVertices() const { return vertices_; }

  uint getNumVertices() const { return uint(vertices_.size()); }

  const CGeomVertex3D &getVertex(uint i) const { return *vertices_[i]; }

  const CGeomVertex3D *getVertexP(uint i) const { return vertices_[i]; }

  CGeomVertex3D &getVertex(uint i) { return *vertices_[i]; }

  //---

  const LineList &getLines() const { return lines_; }

  uint getNumLines() const { return uint(lines_.size()); }

  CGeomLine3D &getLine(uint i) const { return *lines_[i]; }

  //---

  const FaceList &getFaces() const { return faces_; }

  FaceList &getFaces() { return faces_; }

  uint getNumFaces() const { return uint(faces_.size()); }

  const CGeomFace3D &getFace(uint i) const { return *faces_[i]; }
  CGeomFace3D *getFaceP(uint i) const { return faces_[i]; }

  CGeomFace3D &getFace(uint i) { return *faces_[i]; }

  //---

  Group &getGroup(const std::string &name);

  //---

  // textures
  CGeomTexture *getDiffuseTexture() const { return diffuseTexture_; }
  void setDiffuseTexture(CGeomTexture *texture) { diffuseTexture_ = texture; }

  CGeomTexture *getSpecularTexture() const { return specularTexture_; }
  void setSpecularTexture(CGeomTexture *texture) { specularTexture_ = texture; }

  CGeomTexture *getNormalTexture() const { return normalTexture_; }
  void setNormalTexture(CGeomTexture *texture) { normalTexture_ = texture; }

  CGeomTexture *getEmissiveTexture() const { return emissiveTexture_; }
  void setEmissiveTexture(CGeomTexture *texture) { emissiveTexture_ = texture; }

  //---

  // individual face appearance
  CRGBA getFaceColor() const;
  void setFaceColor(const CRGBA &rgba);

  CRGBA getFaceColor(uint face_num) const;
  void setFaceColor(uint face_num, const CRGBA &rgba);

  void setFaceDiffuse(const CRGBA &rgba);
  void setFaceSpecular(const CRGBA &rgba);
  void setFaceEmission(const CRGBA &rgba);

  void setFaceMaterial(uint face_num, const CGeomMaterial &material);

  void setFaceTexture(uint face_num, CGeomTexture *texture);

  void setFaceDiffuseTexture (uint face_num, CGeomTexture *texture);
  void setFaceSpecularTexture(uint face_num, CGeomTexture *texture);
  void setFaceNormalTexture  (uint face_num, CGeomTexture *texture);
  void setFaceEmissiveTexture(uint face_num, CGeomTexture *texture);

  //---

  // individual sub->face appearance
  void setSubFaceColor(const CRGBA &rgba);
  void setSubFaceColor(uint face_num, const CRGBA &rgba);
  void setSubFaceColor(uint face_num, uint sub_face_num, const CRGBA &rgba);

  //---

  // individual line appearance
  void setLineColor(const CRGBA &rgba);
  void setLineColor(uint line_num, const CRGBA &rgba);

  //---

  void setSubLineColor(uint face_num, uint sub_line_num, const CRGBA &rgba);

  //---

  // individual vertex appearance
  void setVertexColor(uint i, const CRGBA &rgba);
  void setVertexPixel(uint i, const CPoint3D &pixel);

  void setVertexNormal(uint i, const CVector3D &n);

  void setVertexTextureMap(uint i, const CPoint2D &p);

  //---

  // material

  // set material for all faces
  void setFrontMaterial(const CGeomMaterial &material);
  void setBackMaterial (const CGeomMaterial &material);

  CGeomMaterial *getMaterialP() { return materialP_; }
  void setMaterialP(CGeomMaterial *material) { materialP_ = material; }

  //---

  CGeomObject3D *getRootObject() const;

  CMatrix3D getMeshGlobalTransform() const;
  CMatrix3D getMeshLocalTransform() const;

  //---

  // skeleton
  bool hasNode(int i) const;
  void addNode(int i, const CGeomNodeData &data);

  const NodeDatas &getNodes() const { return nodes_; }
  const NodeIds &getNodeIds() const { return nodeIds_; }

  int mapNodeId(int id) const;
  int mapNodeIndex(int id) const;

  const CGeomNodeData &getNode(int i) const;
  CGeomNodeData &editNode(int i);

  CGeomNodeData *getNodeByInd(int ind) const;

  CGeomObject3D *getMeshObject() const;

  int getMeshNode() const;
  void setMeshNode(int ind);

  int getRootNode() const;
  void setRootNode(int ind);

  void setNodeLocalTransforms(int i, const CTranslate3D &translation,
                              const CRotate3D &rotation, const CScale3D &scale);
  void setNodeLocalTransform (int i, const CMatrix3D &m);
  void setNodeGlobalTransform(int i, const CMatrix3D &m);

  CMatrix3D getNodeHierTransform(const CGeomNodeData &node) const;

  CMatrix3D getNodeAnimHierTransform(CGeomNodeData &node, const std::string &animName,
                                     double t) const;

  // animation
  const std::string &animName() const { return animName_; }
  void setAnimName(const std::string &s) { animName_ = s; }

  void setNodeAnimationData(int i, const std::string &name, const CGeomAnimationData &data);

  void setNodeAnimationTransformData(int i, const std::string &name, const Transform &transform,
                                     const CGeomAnimationData &data);

  CGeomAnimationData &getNodeAnimationData(int i, const std::string &name);

  bool updateNodesAnimationData(const std::string &name, double t);

  bool updateNodeAnimationData(int i, const std::string &name, double t);

  bool updateHierNodeAnimationData(CGeomNodeData &node, const std::string &name, double t);

  bool updateNodeAnimationData(CGeomNodeData &node, const std::string &name, double t);

  bool updateAnimationData(CGeomNodeData &node, CGeomAnimationData &animationData, double t) const;

  void getAnimationNames(std::vector<std::string> &names) const;

  bool getAnimationTranslationRange(const std::string &name, double &min, double &max) const;

  //---

  struct BodyRevData {
    BodyRevData() { }

    double               angleStart { 0.0 };
    double               angleDelta { 2.0*M_PI };
    bool                 uniquify   { false };
    std::map<uint, uint> tagInds;
  };

  void addBodyRev(double *x, double *y, uint num_xy, uint num_patches,
                  const BodyRevData &data=BodyRevData());

  //---

  void moveTo(const CPoint3D &position);
  void moveBy(const CPoint3D &offset);

  //---

  void setBasis(const CVector3D &right, const CVector3D &up, const CVector3D &dir);
  void getBasis(CVector3D &right, CVector3D &up, CVector3D &dir);

  //---

  void transform(const CMatrix3D &matrix);

  CMatrix3D getTransform() const;
  void setTransform(const CMatrix3D &m);

  CMatrix3D getTranslate() const;
  void setTranslate(const CMatrix3D &m);

  CMatrix3D getRotate() const;
  void setRotate(const CMatrix3D &m);

  CMatrix3D getScale() const;
  void setScale(const CMatrix3D &m);

  CMatrix3D getHierTransform() const;

  //---

  void getModelBBox(CBBox3D &bbox, bool hier=true) const;

  //---

  void reset();

  //---

  CPoint3D verticesMidPoint(const VertexIList &vertices) const;

  CVector3D verticesNormal(const VertexIList &vertices) const;

  //---

  CVector3D getVertexFaceNormal(uint i) const;

  //---

  CPoint3D transformTo(const CPoint3D &p) const;
  CVector3D transformTo(const CVector3D &v) const;

  //---

  virtual void drawSolid(const CGeomCamera3D &camera, CGeomZBuffer *zbuffer);
  virtual void drawSolid(const CGeomCamera3D &camera, CGeom3DRenderer *renderer);

  //---

  virtual void drawWireframe(const CGeomCamera3D &camera, CGeomZBuffer *zbuffer);
  virtual void drawWireframe(const CGeomCamera3D &camera, CGeom3DRenderer *renderer);

  //---

  // model->view->pixel transformations
  virtual void modelToPixel(const CGeomCamera3D &camera);

  void toCurrent(const CGeomCamera3D &camera);

  void toView(const CGeomCamera3D &camera);
  void toView(CGeom3DRenderer *renderer);

  void createViewMatrix(CGeom3DRenderer *renderer, CMatrix3D &matrix);

  void project(const CGeomCamera3D &camera);

  void toPixel(const CGeomCamera3D &camera);

  //---

  // draw (TODO: use standalone render code)
  void drawSolidFaces(CGeom3DRenderer *renderer);
  void drawSolidFaces(CGeomZBuffer *zbuffer);

  void drawLineFaces(CGeom3DRenderer *renderer);
  void drawLineFaces(CGeomZBuffer *zbuffer);

  void drawSubLines(CGeomZBuffer *zbuffer);

  void drawPosition(CGeomZBuffer *zbuffer);

  void drawBBox(const CGeomCamera3D &camera, CGeomZBuffer *zbuffer);
  void drawBBox(const CGeomCamera3D &camera, CGeom3DRenderer *renderer);

  //---

  uint getVertexInd(const CGeomVertex3D &vertex) const;

  //---

  // transform
  virtual void moveX(double dx);
  virtual void moveY(double dy);
  virtual void moveZ(double dz);

  virtual void moveModel(const CPoint3D &d);
  virtual void moveModelX(double dx);
  virtual void moveModelY(double dy);
  virtual void moveModelZ(double dz);

  virtual void rotate(const CPoint3D &angle);
  virtual void rotateX(double dx);
  virtual void rotateY(double dy);
  virtual void rotateZ(double dz);

  virtual void rotateModelX(double dx);
  virtual void rotateModelY(double dy);
  virtual void rotateModelZ(double dz);

  virtual void resizeModel(double factor);
  virtual void resizeModelX(double dx);
  virtual void resizeModelY(double dy);
  virtual void resizeModelZ(double dz);

  //---

  void swapXY();
  void swapYZ();
  void swapZX();

  void invertX();
  void invertY();
  void invertZ();

  //---

  // spin animation
  void resetSpin() { da_ = CVector3D(0, 0, 0); }

  void spinX(double da) { da_.setX(da_.getX() + da); }
  void spinY(double da) { da_.setY(da_.getY() + da); }
  void spinZ(double da) { da_.setZ(da_.getZ() + da); }

  //---

  void update() {
    rotateZ(da_.getZ());
    rotateY(da_.getY());
    rotateX(da_.getX());
  }

  //---

  bool lightPoint(const CPoint3D &point, const CVector3D &normal,
                  const CGeomMaterial &material, CRGBA &rgba) const;

  //---

  FaceList getMaterialFaces(CGeomMaterial *material) const;

  //---

  void divideFace(CGeomFace3D *face, const CPoint3D &c);

 private:
  void validatePObject();

 protected:
  using TexturePoints = std::vector<CPoint3D>;
  using Normals       = std::vector<CVector3D>;

  // scene
  CGeomScene3D* pscene_ { nullptr };

  // state
  uint        ind_ { 0 };
  std::string name_;
  std::string id_;
  std::string meshName_;
  bool        selected_ { false };
  bool        visible_ { true };
  bool        drawPosition_ { true };

  // position
  CCoordFrame3D coordFrame_;
  CGeomPoint3D  position_ { CPoint3D(0, 0, 0) };

  // textures
  CGeomTexture* diffuseTexture_  { nullptr };
  CGeomTexture* specularTexture_ { nullptr };
  CGeomTexture* normalTexture_   { nullptr };
  CGeomTexture* emissiveTexture_ { nullptr };

  // material
  CGeomMaterial* materialP_ { nullptr };

  // geometry
  FaceList         faces_;
  LineList         lines_;
  VertexList       vertices_;
  VertexFaceList   vertexFaceList_;
  VertexFaceNormal vertexFaceNormal_;
  Groups           groups_;
  TexturePoints    texturePoints_;
  Normals          normals_;

  // skeleton
  NodeDatas nodes_;
  NodeIds   nodeIds_;
  int       meshNode_ { -1 };
  int       rootNode_ { -1 };

  CMatrix3D viewMatrix_;

  struct TransformData {
    bool global { true };
    CMatrix3D transform { CMatrix3D::identity() };

    CMatrix3D translate { CMatrix3D::identity() };
    CMatrix3D rotate    { CMatrix3D::identity() };
    CMatrix3D scale     { CMatrix3D::identity() };
  };

  TransformData transformData_;

  // animation
  std::string animName_;

  // motion
  CVector3D dv_ { 0, 0, 0 };
  CVector3D da_ { 0, 0, 0 };

  // hierarchy (parent, children)
  CGeomObject3D*               parent_ { nullptr };
  std::vector<CGeomObject3D *> children_;
};

//------

class CGeomObjectInst3D {
 public:
  CGeomObjectInst3D(CGeomObject3D &object) :
   object_(object) {
  }

  const CGeomObject3D &object() const { return object_; }

 private:
  CGeomObject3D& object_;
  CCoordFrame3D  coordFrame_;
};

#endif
