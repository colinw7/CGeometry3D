#ifndef CGEOM_FACE_3D_H
#define CGEOM_FACE_3D_H

#include <CGeomVertex3D.h>
#include <CGeomLine3D.h>
#include <CGeomMaterial.h>

#include <CImagePtr.h>
#include <CMatrix3D.h>
#include <CBBox3D.h>
#include <CPoint2D.h>
#include <CPolygonOrientation.h>

class CGeomScene3D;
class CGeomObject3D;
class CGeomEdge3D;
class CGeomZBuffer;
class CGeomTexture;
class CGeomMask;
class CGeom3DRenderer;

class CGeomFace3D {
 public:
  using VertexList    = std::vector<uint>;
  using Normals       = std::vector<CVector3D>;
  using TexturePoints = std::vector<CPoint2D>;
  using FaceList      = std::vector<CGeomFace3D *>;
  using SubFaceList   = std::vector<CGeomFace3D *>;
  using SubLineList   = std::vector<CGeomLine3D *>;
  using EdgeList      = std::vector<CGeomEdge3D *>;

  using OptColor = std::optional<CRGBA>;
  using OptReal  = std::optional<double>;

  enum {
    LIGHTED   = (1L<<0),
    TWO_SIDED = (1L<<1),
    SELECTED  = (1L<<2),
    VISIBLE   = (1L<<3)
  };

  struct VertexData {
    VertexList    vertices;
    bool          hasNormals       { false };
    bool          hasTexturePoints { false };
    Normals       normals;
    TexturePoints texturePoints;
  };

 public:
  explicit CGeomFace3D(CGeomObject3D *pobject=nullptr);
  CGeomFace3D(CGeomObject3D *pobject, const VertexList &vertices);

  CGeomFace3D(const CGeomFace3D &face);

  CGeomFace3D &operator=(const CGeomFace3D &rhs) = delete;

  virtual ~CGeomFace3D() { }

  virtual CGeomFace3D *dup() const;

  //---

  CGeomFace3D *duplicate() const;

  //---

  CGeomObject3D *getObject() const { return pobject_; }
  void setObject(CGeomObject3D *object);

  //---

  const uint &getInd() const { return ind_; }
  void setInd(const uint &i) { ind_ = i; }

  //---

  bool getLighted() const { return (flags_ & LIGHTED); }
  void setLighted(bool b);

  bool getTwoSided() const { return (flags_ & TWO_SIDED); }
  void setTwoSided(bool b);

  bool getSelected() const { return (flags_ & SELECTED); }
  void setSelected(bool b);

  bool getVisible() const { return (flags_ & VISIBLE); }
  void setVisible(bool b);

  //---

  void setFlags  (uint flags) { flags_ |=  flags; }
  void unsetFlags(uint flags) { flags_ &= ~flags; }

  //---

  const std::string &name() const { return name_; }
  void setName(const std::string &s) { name_ = s; }

  //---

  CGeomScene3D *getScene() const;

  //---

  CGeomTexture *getTexture() const { return getDiffuseTexture(); }

  void setTexture(CGeomTexture *texture);
  void setTexture(CImagePtr image);

  CGeomTexture *getDiffuseTexture() const { return diffuseTexture_; }
  void setDiffuseTexture(CGeomTexture *texture);
  void setDiffuseTexture(CImagePtr image);

  CGeomTexture *getSpecularTexture() const { return specularTexture_; }
  void setSpecularTexture(CGeomTexture *texture);
  void setSpecularTexture(CImagePtr image);

  CGeomTexture *getNormalTexture() const { return normalTexture_; }
  void setNormalTexture(CGeomTexture *texture);

  CGeomTexture *getEmissiveTexture() const { return emissiveTexture_; }
  void setEmissiveTexture(CGeomTexture *texture);

  void setTextureMapping(const std::vector<CPoint2D> &points);

  bool hasVertexTexturePoints() const { return (texturePoints_.size() == vertices_.size()); }
  const std::vector<CPoint2D> &getTexturePoints() const { return texturePoints_; }
  void setTexturePoints(const TexturePoints &points);

  CPoint2D getTexturePoint(const CGeomVertex3D &v, int iv) const;

  //---

  CGeomMask *getMask() const { return mask_; }

  void setMask(CGeomMask *mask) { mask_ = mask; }

  void setMask(CImagePtr image);

  void setMaskMapping(const std::vector<CPoint2D> &points);

  //---

  void setNormal(const CVector3D &normal) { normal_ = normal; }

  bool getNormalSet() const { return !!normal_; }

  const CVector3D &getNormal() const { return normal_.value(); }

  void setVertexNormals(const std::vector<CVector3D> &normals, bool propagate=true);
  bool hasVertexNormals() const { return normals_.size() == vertices_.size(); }
  const Normals &getVertexNormals() const { return normals_; }
  CVector3D getVertexNormal(uint i) const { return normals_[i]; }

  //---

  void setGroup(uint id) { groupId_ = id; }

  //---

  void addVertex(uint ind);

  void addVertices(int ind, ...) {
    va_list vargs;

    va_start(vargs, ind);

    while (ind >= 0) {
      addVertex(ind);

      ind = va_arg(vargs, int);
    }

    va_end(vargs);
  }

  //---

  uint addSubFace(const VertexList &vertices);
  uint addSubFace(CGeomFace3D *face);

  uint addSubLine(uint start, uint end);
  uint addSubLine(CGeomLine3D *line);

  //---

  CRGBA getColor() const { return getFrontColor(); }
  void setColor(const CRGBA &rgba) { setFrontColor(rgba); }

  const OptColor &color() const { return initFrontMaterial()->diffuse(); }

  CRGBA getFrontColor() const { return initFrontMaterial()->getDiffuse(); }
  void setFrontColor(const CRGBA &rgba) { initFrontMaterial()->setDiffuse(rgba); }

  CRGBA getBackColor() const { return initBackMaterial()->getDiffuse(CRGBA(1, 1, 1, 1)); }
  void setBackColor(const CRGBA &rgba) { initBackMaterial()->setDiffuse(rgba); }

  //---

  void setSubFaceColor(const CRGBA &rgba);

  void setSubFaceColor(uint ind, const CRGBA &rgba);
  void setSubLineColor(uint ind, const CRGBA &rgba);

  void setSubFaceMaterialP(CGeomMaterial *);

  //---

  const OptReal &shininess() const { return initFrontMaterial()->shininess(); }

  const OptColor &emission() const { return initFrontMaterial()->emission(); }

  //---

  void setAmbient(const CRGBA &rgba) { setFrontAmbient(rgba); }
  void setFrontAmbient(const CRGBA &rgba) { initFrontMaterial()->setAmbient(rgba); }
  void setBackAmbient(const CRGBA &rgba) { initBackMaterial()->setAmbient(rgba); }

  //---

  void setDiffuse(const CRGBA &rgba) { setFrontDiffuse(rgba); }
  void setFrontDiffuse(const CRGBA &rgba) { initFrontMaterial()->setDiffuse(rgba); }
  void setBackDiffuse(const CRGBA &rgba) { initBackMaterial()->setDiffuse(rgba); }

  //---

  void setSpecular(const CRGBA &rgba) { setFrontSpecular(rgba); }
  void setFrontSpecular(const CRGBA &rgba) { initFrontMaterial()->setSpecular(rgba); }
  void setBackSpecular(const CRGBA &rgba) { initBackMaterial()->setSpecular(rgba); }

  //---

  CRGBA getEmission() const { return getFrontEmission(); }
  CRGBA getFrontEmission() const { return initFrontMaterial()->getEmission(CRGBA(0, 0, 0, 1)); }

  void setEmission(const CRGBA &rgba) { setFrontEmission(rgba); }
  void setFrontEmission(const CRGBA &rgba) { initFrontMaterial()->setEmission(rgba); }
  void setBackEmission(const CRGBA &rgba) { initBackMaterial()->setEmission(rgba); }

  //---

  void setShininess(double shininess) { setFrontShininess(shininess); }
  void setFrontShininess(double shininess) { initFrontMaterial()->setShininess(shininess); }
  void setBackShininess (double shininess) { initBackMaterial()->setShininess(shininess); }

  //---

  const CGeomMaterial &getMaterial() const { return getFrontMaterial(); }
  void setMaterial(const CGeomMaterial &material) { setFrontMaterial(material); }

  const CGeomMaterial &getFrontMaterial() const { return *initFrontMaterial(); }
  void setFrontMaterial(const CGeomMaterial &material) { *initFrontMaterial() = material; }

  const CGeomMaterial &getBackMaterial () const { return *initBackMaterial() ; }
  void setBackMaterial(const CGeomMaterial &material) { *initBackMaterial() = material; }

  CGeomMaterial *getMaterialP() { return materialP_; }
  void setMaterialP(CGeomMaterial *material) { materialP_ = material; }

  //---

  void getVertexData(VertexData &vertexData) const;

  const VertexList &getVertices() const { return vertices_; }

  virtual void setVertices(const VertexList &vertices) { vertices_ = vertices; }

  uint getNumVertices() const { return uint(vertices_.size()); }

  uint getVertex(uint i) const { return vertices_[i]; }

  //---

  const SubFaceList &subFaces() const { return subFaces_; }

  const SubLineList &subLines() const { return subLines_; }

  //---

  bool edgesValid() const;

  const EdgeList &getEdges() const;

  CVector3D edgeVector(const CGeomEdge3D *edge) const;

  CPoint3D edgeStart(const CGeomEdge3D *edge) const;
  CPoint3D edgeEnd  (const CGeomEdge3D *edge) const;

  //---

  void getModelBBox(CBBox3D &bbox) const;

  double distanceTo(const CPoint3D &p) const;
  double distanceToCenter(const CPoint3D &p) const;

  //---

  void drawSolid(CGeom3DRenderer *renderer);
  void drawSolid(CGeomZBuffer *zbuffer);

  void drawLines(CGeom3DRenderer *renderer);
  void drawLines(CGeomZBuffer *zbuffer);

  //---

  void fill(CGeomZBuffer *zbuffer);

  //---

  bool getAdjustedColor(CRGBA &rgba);
  bool getColorFactor(double *factor);

  //---

  void getModelMidPoint(CPoint3D &mid_point) const;
  void getViewedMidPoint(CPoint3D &mid_point) const;

  void calcModelNormal(CVector3D &normal) const;
  void calcViewedNormal(CVector3D &normal) const;

  //---

  CPoint2D interpTexturePoint(int i1, int i2, double d, double i);

  CPoint2D interpMaskPoint(int i1, int i2, double d, double i);

  //---

  CPolygonOrientation getProjectedOrientation() const;
  CPolygonOrientation getProjectedOrientationI(uint i1) const;

  CPolygonOrientation getModelOrientation() const;
  CPolygonOrientation getModelOrientationI(uint i1) const;

  bool checkModelOrientation() const;

  //---

  void moveBy(const CVector3D &v);

  void scale(double sx, double sy, double sz);
  void scale(const CVector3D &s);
  void centerScale(const CPoint3D &c, const CVector3D &s);

  void rotateModelX(double a);
  void rotateModelY(double a);
  void rotateModelZ(double a);

  //---

  void triangulate();

  void divideCenter();

  CPoint3D calcModelCenter    () const;
  CPoint3D calcProjectedCenter() const;

  //---

  struct ExtrudeData {
    CGeomFace3D*               topFace { nullptr };
    std::vector<CGeomFace3D *> sideFaces;
  };

  ExtrudeData extrude(double d);

  void extrudeMove(double d);

  //---

  CGeomFace3D *loopCut();

  bool removeVertex(uint ind);
  bool replaceVertex(uint oldInd, uint newInd);

  bool hasVertex(uint ind) const;

  //---

  FaceList bevel(double d);
  FaceList inset(double d);

  FaceList subdivide(uint n);

  //---

  bool isProjectedInside(const CPoint2D &p) const;

  bool fixNormal();

 private:
  void init();

  CGeomMaterial *initFrontMaterial() const;
  CGeomMaterial *initBackMaterial() const;

  FaceList bevelInset(double dx, double dy);

 protected:
  using OptVector = std::optional<CVector3D>;

  CGeomObject3D* pobject_ { nullptr }; // parent object

  uint ind_ { 0 }; // unique id

  uint groupId_ { 0 }; // parent group id

  uint flags_ { LIGHTED }; // state flags

  std::string name_; // name

  // geometry

  VertexList    vertices_;      // vertices (indices)
  Normals       normals_;       // individual vertex normals
  TexturePoints texturePoints_; // individual vertex texture points

  SubFaceList subFaces_; // sub faces
  SubLineList subLines_; // sub lines

  OptVector normal_; // calculated normal

  // materials (TODO: simplify)
  CGeomMaterial* frontMaterial_ { nullptr };
  CGeomMaterial* backMaterial_  { nullptr };
  CGeomMaterial* materialP_     { nullptr };

  // textures (TODO: use material)
  CGeomTexture* diffuseTexture_  { nullptr };
  CGeomTexture* specularTexture_ { nullptr };
  CGeomTexture* normalTexture_   { nullptr };
  CGeomTexture* emissiveTexture_ { nullptr };

  CGeomMask* mask_ { nullptr };

  mutable EdgeList edges_;
};

#endif
