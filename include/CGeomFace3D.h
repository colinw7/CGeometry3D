#ifndef CGEOM_FACE_3D_H
#define CGEOM_FACE_3D_H

#include <CImagePtr.h>
#include <CGeomVertex3D.h>
#include <CGeomLine3D.h>
#include <CMaterial.h>
#include <CMatrix3D.h>
#include <CPoint2D.h>
#include <CPolygonOrientation.h>

class CGeomScene3D;
class CGeomObject3D;
class CGeomZBuffer;
class CGeomTexture;
class CGeomMask;
class CGeom3DRenderer;

class CGeomFace3D {
 public:
  using VertexList    = std::vector<uint>;
  using TexturePoints = std::vector<CPoint2D>;
  using SubFaceList   = std::vector<CGeomFace3D *>;
  using SubLineList   = std::vector<CGeomLine3D *>;

  enum {
    LIGHTED   = (1<<0),
    TWO_SIDED = (1<<1)
  };

 public:
  CGeomFace3D(CGeomObject3D *pobject);
  CGeomFace3D(CGeomObject3D *pobject, const VertexList &vertices);

  CGeomFace3D(const CGeomFace3D &face);

  virtual ~CGeomFace3D() { }

  virtual CGeomFace3D *dup() const;

  //---

  CGeomObject3D *getObject() const { return pobject_; }

  void setObject(CGeomObject3D *object);

  //---

  ACCESSOR(Ind, uint, ind)

  //---

  bool getLighted () const { return (flags_ & LIGHTED  ); }
  bool getTwoSided() const { return (flags_ & TWO_SIDED); }

  void setLighted(bool lighted);
  void setTwoSided(bool twoSided);

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

  CGeomTexture *getNormalTexture () const { return normalTexture_; }
  void setNormalTexture(CGeomTexture *texture);

  CGeomTexture *getEmissiveTexture () const { return emissiveTexture_; }
  void setEmissiveTexture(CGeomTexture *texture);

  void setTextureMapping(const std::vector<CPoint2D> &points);

  void setTexturePoints(const std::vector<CPoint2D> &points);
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

  //---

  void setFlags(uint flags) {
    flags_ |= flags;
  }

  void unsetFlags(uint flags) {
    flags_ &= ~flags;
  }

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

  uint addSubFace(const std::vector<uint> &vertices);
  uint addSubLine(uint start, uint end);

  //---

  void setColor(const CRGBA &rgba) { frontMaterial_->setColor(rgba); }
  CRGBA getColor() const { return frontMaterial_->getColor(); }

  const CMaterial::OptColor &color() const { return frontMaterial_->color(); }

  const CMaterial::OptReal &shininess() const { return frontMaterial_->shininess(); }

  const CMaterial::OptColor &emission() const { return frontMaterial_->emission(); }

  //---

  void setSubFaceColor(const CRGBA &rgba);

  void setSubFaceColor(uint ind, const CRGBA &rgba);
  void setSubLineColor(uint ind, const CRGBA &rgba);

  //---

  const CMaterial &getMaterial() const { return *frontMaterial_; }
  void setMaterial(const CMaterial &material) { *frontMaterial_ = material; }

  const CMaterial &getFrontMaterial() const { return *frontMaterial_; }
  void setFrontMaterial(const CMaterial &material) { *frontMaterial_ = material; }

  const CMaterial &getBackMaterial () const { return *backMaterial_ ; }
  void setBackMaterial(const CMaterial &material) { *backMaterial_ = material; }

  //---

  CRGBA getFrontColor() const { return frontMaterial_->getColor(); }
  void setFrontColor(const CRGBA &rgba) { frontMaterial_->setColor(rgba); }

  CRGBA getBackColor() const { return backMaterial_->getColor(); }
  void setBackColor(const CRGBA &rgba) { backMaterial_->setColor(rgba); }

  //---

  void setAmbient(const CRGBA &rgba) { setFrontAmbient(rgba); }
  void setFrontAmbient(const CRGBA &rgba) { frontMaterial_->setAmbient(rgba); }
  void setBackAmbient(const CRGBA &rgba) { backMaterial_->setAmbient(rgba); }

  //---

  void setDiffuse(const CRGBA &rgba) { setFrontDiffuse(rgba); }
  void setFrontDiffuse(const CRGBA &rgba) { frontMaterial_->setDiffuse(rgba); }
  void setBackDiffuse(const CRGBA &rgba) { backMaterial_->setDiffuse(rgba); }

  //---

  void setSpecular(const CRGBA &rgba) { setFrontSpecular(rgba); }
  void setFrontSpecular(const CRGBA &rgba) { frontMaterial_->setSpecular(rgba); }
  void setBackSpecular(const CRGBA &rgba) { backMaterial_->setSpecular(rgba); }

  //---

  CRGBA getEmission() const { return getFrontEmission(); }
  CRGBA getFrontEmission() const { return frontMaterial_->getEmission(); }

  void setEmission(const CRGBA &rgba) { setFrontEmission(rgba); }
  void setFrontEmission(const CRGBA &rgba) { frontMaterial_->setEmission(rgba); }
  void setBackEmission(const CRGBA &rgba) { backMaterial_->setEmission(rgba); }

  //---

  void setShininess(double shininess) { setFrontShininess(shininess); }
  void setFrontShininess(double shininess) { frontMaterial_->setShininess(shininess); }
  void setBackShininess (double shininess) { backMaterial_->setShininess(shininess); }

  //---

  const VertexList &getVertices() const { return vertices_; }

  uint getNumVertices() const { return uint(vertices_.size()); }

  uint getVertex(uint i) const { return vertices_[i]; }

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

  void getMidPoint(CPoint3D &mid_point);

  void calcNormal(CVector3D &normal) const;

  //---

  CPoint2D interpTexturePoint(int i1, int i2, double d, double i);

  CPoint2D interpMaskPoint(int i1, int i2, double d, double i);

  //---

  CPolygonOrientation orientation() const;

 private:
  CGeomFace3D &operator=(const CGeomFace3D &rhs);

  void init();

 protected:
  using OptVector = std::optional<CVector3D>;

  CGeomObject3D* pobject_         { nullptr };
  uint           ind_             { 0 };
  VertexList     vertices_;
  TexturePoints  texturePoints_;
  SubFaceList    sub_faces_;
  SubLineList    sub_lines_;
  CMaterial*     frontMaterial_   { nullptr };
  CMaterial*     backMaterial_    { nullptr };
  CGeomTexture*  diffuseTexture_  { nullptr };
  CGeomTexture*  specularTexture_ { nullptr };
  CGeomTexture*  normalTexture_   { nullptr };
  CGeomTexture*  emissiveTexture_ { nullptr };
  CGeomMask*     mask_            { nullptr };
  OptVector      normal_;
  uint           flags_           { LIGHTED };
  uint           groupId_         { 0 };
};

#endif
