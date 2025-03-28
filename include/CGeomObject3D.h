#ifndef CGEOM_OBJECT_3D_H
#define CGEOM_OBJECT_3D_H

#include <CGeomFace3D.h>
#include <CGeomPoint3D.h>
#include <CCoordFrame3D.h>
#include <CBBox3D.h>
#include <CMatrix3D.h>
#include <map>

class CGeomScene3D;
class CGeomCamera;
class CGeomZBuffer;
class CGeom3DRenderer;

class CGeomObject3D {
 public:
  using FaceList         = std::vector<CGeomFace3D *>;
  using FaceIList        = std::vector<uint>;
  using LineList         = std::vector<CGeomLine3D *>;
  using VertexList       = std::vector<CGeomVertex3D *>;
  using VertexIList      = std::vector<uint>;
  using VertexFaceList   = std::map<uint,FaceIList>;
  using VertexFaceNormal = std::map<uint,CVector3D>;

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

 public:
  CGeomObject3D(CGeomScene3D *pscene, const std::string &name) :
   pscene_(pscene), name_(name) {
  }

  CGeomObject3D(const CGeomObject3D &object);

  virtual ~CGeomObject3D();

  virtual CGeomObject3D *dup() const;

  //------

  void setScene(CGeomScene3D *scene) { pscene_ = scene; }

  CGeomScene3D *getScene() const { return pscene_; }

  const std::string &getName() const { return name_; }
  void setName(const std::string &name) { name_ = name; }

  ACCESSOR(Selected, bool, selected)
  ACCESSOR(Visible , bool, visible )

  //---

  CPoint3D getPosition() const { return coord_frame_.getOriginPoint(); }

  const CGeomPoint3D &getPositionPoint() const { return position_; }

  CGeomPoint3D &editPositionPoint() { return position_; }

  void setAbsPosition(const CPoint3D &point) {
    position_.setModel  (point);
    position_.setCurrent(point);
    position_.setViewed (point);
  }

  void setPosition(const CPoint3D &point) {
    coord_frame_.setOrigin(point);

    updatePosition();
  }

  void setPositionPoint(const CGeomPoint3D &point) {
    position_ = point;

    setPosition(point.getCurrent());
  }

  void updatePosition() {
    position_.setModel  (coord_frame_.getOriginPoint());
    position_.setCurrent(coord_frame_.getOriginPoint());
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

  void addVertexFace(uint vertex_ind, uint face_num);

  //---

  uint getNumTextuePoints() const { return uint(texturePoints_.size()); }

  uint addTexturePoint(const CPoint3D &point);
  const CPoint3D &texturePoint(uint i) const;

  //---

  uint getNumNormals() const { return uint(normals_.size()); }

  uint addNormal(const CVector3D &point);
  const CVector3D &normal(uint i) const;

  //---

  uint addLine(uint start, uint end);

  //---

  std::vector<uint> addITriangles(uint *inds, uint num_inds);

  uint addITriangle(uint i1, uint i2, uint i3);

  //---

  uint addIPolygon(uint *inds, uint num_inds);

  //---

  uint addFace(const VertexIList &vertices);

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

  CGeomFace3D &getFace(uint i) { return *faces_[i]; }

  //---

  Group &getGroup(const std::string &name) {
    auto p = groups_.find(name);

    if (p == groups_.end()) {
      uint id = uint(groups_.size() + 1);

      p = groups_.insert(p, Groups::value_type(name, Group(name, id)));
    }

    return (*p).second;
  }

  //---

  CGeomTexture *getDiffuseTexture() const { return diffuseTexture_; }
  void setDiffuseTexture(CGeomTexture *texture) { diffuseTexture_ = texture; }

  CGeomTexture *getSpecularTexture() const { return specularTexture_; }
  void setSpecularTexture(CGeomTexture *texture) { specularTexture_ = texture; }

  CGeomTexture *getNormalTexture() const { return normalTexture_; }
  void setNormalTexture(CGeomTexture *texture) { normalTexture_ = texture; }

  //---

  void setFaceColor(const CRGBA &rgba);
  void setFaceColor(uint face_num, const CRGBA &rgba);

  void setFaceDiffuse(const CRGBA &rgba);
  void setFaceSpecular(const CRGBA &rgba);

  void setFaceMaterial(uint face_num, const CMaterial &material);

  void setFaceTexture(uint face_num, CGeomTexture *texture);

  void setFaceDiffuseTexture (uint face_num, CGeomTexture *texture);
  void setFaceSpecularTexture(uint face_num, CGeomTexture *texture);
  void setFaceNormalTexture  (uint face_num, CGeomTexture *texture);

  //---

  void setSubFaceColor(const CRGBA &rgba);
  void setSubFaceColor(uint face_num, const CRGBA &rgba);
  void setSubFaceColor(uint face_num, uint sub_face_num, const CRGBA &rgba);

  //---

  void setLineColor(const CRGBA &rgba);
  void setLineColor(uint line_num, const CRGBA &rgba);

  //---

  void setSubLineColor(uint face_num, uint sub_line_num, const CRGBA &rgba);

  //---

  void setVertexColor(uint i, const CRGBA &rgba);
  void setVertexPixel(uint i, const CPoint3D &pixel);

  void setVertexNormal(uint i, const CVector3D &n);

  void setVertexTextureMap(uint i, const CPoint2D &p);

  //---

  void setFrontMaterial(const CMaterial &material);
  void setBackMaterial (const CMaterial &material);

  //---

  void addBodyRev(double *x, double *y, uint num_xy, uint num_patches);

  //---

  void moveTo(const CPoint3D &position);
  void moveBy(const CPoint3D &offset);

  //---

  void setBasis(const CVector3D &right, const CVector3D &up, const CVector3D &dir);

  void getBasis(CVector3D &right, CVector3D &up, CVector3D &dir);

  //---

  void transform(const CMatrix3D &matrix);

  //---

  void getModelBBox(CBBox3D &bbox) const;

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

  virtual void modelToPixel(const CGeomCamera3D &camera);

  void toCurrent(const CGeomCamera3D &camera);

  void toView(const CGeomCamera3D &camera);
  void toView(CGeom3DRenderer *renderer);

  void createViewMatrix(CGeom3DRenderer *renderer, CMatrix3D &matrix);

  void project(const CGeomCamera3D &camera);

  void toPixel(const CGeomCamera3D &camera);

  //---

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
                  const CMaterial &material, CRGBA &rgba) const;

 private:
  void validatePObject();

 private:
  CGeomObject3D &operator=(const CGeomObject3D &rhs);

 protected:
  using TexturePoints = std::vector<CPoint3D>;
  using Normals       = std::vector<CVector3D>;

  CGeomScene3D* pscene_ { nullptr };

  std::string name_;
  bool        selected_ { false };
  bool        visible_ { true };
  bool        draw_position_ { true };

  CCoordFrame3D    coord_frame_;
  CGeomPoint3D     position_ { CPoint3D(0, 0, 0) };

  CGeomTexture* diffuseTexture_  { nullptr };
  CGeomTexture* specularTexture_ { nullptr };
  CGeomTexture* normalTexture_   { nullptr };

  FaceList         faces_;
  LineList         lines_;
  VertexList       vertices_;
  VertexFaceList   vertexFaceList_;
  VertexFaceNormal vertexFaceNormal_;
  Groups           groups_;
  TexturePoints    texturePoints_;
  Normals          normals_;

  CMatrix3D view_matrix_;

  CVector3D dv_ { 0, 0, 0 };
  CVector3D da_ { 0, 0, 0 };
};

//------

class CGeomObjectInst3D {
 public:
  CGeomObjectInst3D(CGeomObject3D &object) :
   object_(object), coord_frame_() {
  }

  const CGeomObject3D &object() const { return object_; }

 private:
  CGeomObject3D &object_;
  CCoordFrame3D  coord_frame_;
};

#endif
