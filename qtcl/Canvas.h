#ifndef Canvas_H
#define Canvas_H

#include <App.h>
#include <FaceData.h>

#include <CImagePtr.h>
#include <CMatrix3DH.h>
#include <CBBox3D.h>
#include <CPoint2D.h>
#include <CPlane3D.h>

#include <QGLWidget>
#include <QOpenGLExtraFunctions>

#include <optional>

class CQGLBuffer;
class CQGLTexture;
class CGeomTexture;
class CQRubberBand;

namespace CQTclModel3DView {

class App;
class Camera;
class ShaderProgram;
class Texture;
class Font;
class Text;

class Canvas : public QGLWidget, public QOpenGLExtraFunctions {
  Q_OBJECT

 public:
  using SelectType = App::SelectType;
  using EditType   = App::EditType;

  using Texts = std::vector<Text *>;

  struct ParticleData {
    CQGLBuffer*           buffer { nullptr };
    std::vector<FaceData> faceDatas;
  };

  struct ViewportData {
    std::string           id;
    CBBox2D               rect       { 0, 0, 1, 1 };
    QColor                bgColor    { 140, 140, 150 };
    Camera*               camera     { nullptr };
    CBBox3D               bbox;
    bool                  updateBBox { true };
    std::vector<CPlane3D> clips;
  };

  struct ShaderData {
    std::string id;

    std::string vs;
    std::string gs;
    std::string fs;

    ShaderProgram* program { nullptr };

    ParticleData particleData;

    bool point { false };

    double lineWidth { 0 };
  };

  using Particles       = std::vector<CPSysParticle *>;
  using ShaderParticles = std::map<ShaderData *, Particles>;

 public:
  Canvas(App *app);

  //---

  App *app() const { return app_; }

  int ind() const { return ind_; }
  void setInd(int i) { ind_ = i; }

  //---

  Camera *camera() const;

  bool isPerspective() const { return perspective_; }
  void setPerspective(bool b) { perspective_ = b; }

  //---

  ShaderProgram *sceneShaderProgram();
  ShaderProgram *selectionShaderProgram();
  ShaderProgram *particleShaderProgram();

  //---

  int pixelWidth() const { return pixelWidth_; }
  void setPixelWidth(int i) { pixelWidth_ = i; }

  int pixelHeight() const { return pixelHeight_; }
  void setPixelHeight(int i) { pixelHeight_ = i; }

  double aspect() const { return aspect_; }
  void setAspect(double r) { aspect_ = r; }

  //---

  const QColor &bgColor() const;
  void setBgColor(const QColor &c);

  CBBox2D viewportRegion() const;

  //---

  bool isDepthTest() const { return depthTest_; }
  void setDepthTest(bool b) { depthTest_ = b; }

  bool isCullFace() const { return cullFace_; }
  void setCullFace(bool b);
  void enableCullFace();

  bool isFrontFace() const { return frontFace_; }
  void setFrontFace(bool b);
  void enableFrontFace();

  bool isPolygonLine() const { return polygonLine_; }
  void setPolygonLine(bool b) { polygonLine_ = b; }

  double pointSize() const { return pointSize_; }
  void setPointSize(double r) { pointSize_ = r; update(); }

  double lineWidth() const { return lineWidth_; }
  void setLineWidth(double r) { lineWidth_ = r; update(); }

  bool isShowOrient() const { return showOrient_; }
  void setShowOrient(bool b) { showOrient_ = b; update(); }

  int isStoreProjected() const { return storeProjected_; }
  void setStoreProjected(int i) { storeProjected_ = i; }

  //---

  const CRGBA &ambientColor() const { return ambientColor_; }
  void setAmbientColor(const CRGBA &v) { ambientColor_ = v; }

  double ambientStrength() const { return ambientStrength_; }
  void setAmbientStrength(double r) { ambientStrength_ = r; }

  double diffuseStrength() const { return diffuseStrength_; }
  void setDiffuseStrength(double r) { diffuseStrength_ = r; }

  const CRGBA &emissiveColor() const { return emissiveColor_; }
  void setEmissiveColor(const CRGBA &v) { emissiveColor_ = v; }

  double emissiveStrength() const { return emissiveStrength_; }
  void setEmissiveStrength(double r) { emissiveStrength_ = r; }

  const CRGBA &specularColor() const { return specularColor_; }
  void setSpecularColor(const CRGBA &v) { specularColor_ = v; }

  double specularStrength() const { return specularStrength_; }
  void setSpecularStrength(double r) { specularStrength_ = r; }

  bool isFixedDiffuse() const { return fixedDiffuse_; }
  void setFixedDiffuse(bool b) { fixedDiffuse_ = b; update(); }

  bool isFlatShaded() const { return flatShaded_; }
  void setFlatShaded(bool b) { flatShaded_ = b; update(); }

  //---

  bool isWireframe() const { return wireframe_; }
  void setWireframe(bool b) { wireframe_ = b; }

  bool isSolid() const { return solid_; }
  void setSolid(bool b) { solid_ = b; }

  bool isTextured() const { return textured_; }
  void setTextured(bool b) { textured_ = b; }

  //---

  const CRGBA &selectColor() const { return selectColor_; }
  void setSelectColor(const CRGBA &c) { selectColor_ = c; update(); }

  const CRGBA &wireframeColor() const { return wireframeColor_; }
  void setWireframeColor(const CRGBA &c) { wireframeColor_ = c; update(); }

  //---

  const EditType &editType() const { return editType_; }
  void setEditType(const EditType &v) {
    editType_ = v; updateStatus(); Q_EMIT editTypeChanged(); }

  const SelectType &selectType() const { return selectType_; }
  void setSelectType(const SelectType &v) {
    selectType_ = v; updateStatus(); Q_EMIT selectTypeChanged(); }

  //---

  const CBBox3D &bbox() const;
  void setBBox(const CBBox3D &b);

  void setBBox(ViewportData *viewportData, const CBBox3D &b);

  void updateCamera();
  void updateCamera(Camera *camera);

  //---

  void initializeGL() override;

  void resizeGL(int, int) override;

  void paintGL() override;

  //---

  void enableClips(bool b);

  void addScene ();
  void addObject(CGeomObject3D *object, CBBox3D &bbox, bool updateBBox);

  void drawScene();
  void drawObject(CGeomObject3D *object);

  void drawSelection();

  //---

  void mousePressEvent  (QMouseEvent *e) override;
  void mouseMoveEvent   (QMouseEvent *e) override;
  void mouseReleaseEvent(QMouseEvent *e) override;

  void wheelEvent(QWheelEvent *e) override;

  void keyPressEvent(QKeyEvent *event) override;

  //---

  QSize sizeHint() const override { return QSize(1536, 1536); }

  //---

  void addParticles(CBBox3D &bbox, bool updateBBox);

  void drawParticles();

  void drawParticle(CPSysParticle *particle, int i, ShaderProgram *program,
                    CQGLTexture* &texture, ShaderData *shaderData);

  void getShaderParticles(ShaderParticles &shaderParticles) const;

  ShaderProgram *getShaderDataProgram(ShaderData *shaderData);

  //---

  CMatrix3DH calcWorldMatrix() const;

  //---

  Font *font() const { return font_; }

  const Texts &texts() const { return texts_; }

  void clearTexts();

  void addText(Text *text);

  Text *getTextById(uint id) const;

  void updateTexts();

  void drawTexts();

  //---

  void selectFaceAt  (const CPoint2D &p, bool flip=false);
  void selectFaceIn  (const CPoint2D &p1, const CPoint2D &p2, bool flip=false);
  void selectEdgeAt  (const CPoint2D &p, bool flip=false);
  void selectVertexAt(const CPoint2D &p, bool flip=false);
  void selectVertexIn(const CPoint2D &p1, const CPoint2D &p2, bool flip=false);

  void deselectAll();

  //---

  std::vector<CGeomObject3D *> getDrawObjects() const;
  void addDrawObjects(CGeomObject3D *object, std::vector<CGeomObject3D *> &objects) const;

  void projectFaceVertices(CGeomFace3D *face);
  void projectVertex(CGeomVertex3D *vertex);

  //---

  CPoint2D pixelToView(const CPoint2D &p) const;

  //---

  void updateStatus();

  //---

  std::string addViewport(const CBBox2D &rect);

  ViewportData *getCurrentViewportData() const;

  ViewportData *getViewportData(const std::string &id) const;

  const uint &currentViewport() const { return currentViewport_; }
  void setCurrentViewport(const uint &v) { currentViewport_ = v; }

  //---

  std::string addShaderData(const std::string &vs, const std::string &fs);
  std::string addShaderData(const std::string &vs, const std::string &gs, const std::string &fs);

  ShaderData *getShaderData(const std::string &id) const;

 private:
  ShaderProgram *getShader(const QString &vertex, const QString &fragment);
  ShaderProgram *getShader(const QString &vertex, const QString &geoemtry, const QString &fragment);

  void enableDepthTest();
  void enablePolygonLine();

  CQGLTexture *getGLTexture(CGeomTexture *texture, bool /*add*/);

  void initGLTexture(Texture *texture);

  CQGLTexture *makeTexture(const CImagePtr &image) const;

  void deselectAllI();

  void emitSelectionChanged();

 Q_SIGNALS:
  void glStateChanged();

  void selectTypeChanged();
  void editTypeChanged();

  void textureAdded();

  void selectionChanged();

 public Q_SLOTS:
  void updateScene(bool updateBBox=true);

 private:
  struct MouseData {
    bool            pressed   { false };
    bool            isShift   { false };
    bool            isControl { false };
    Qt::MouseButton button    { Qt::NoButton };
    CPoint2D        press     { 0.0, 0.0 };
    CPoint2D        move      { 0.0, 0.0 };
  };

  //---

  App* app_ { nullptr };

  int ind_ { 0 };

  // camera
  bool perspective_ { true };

  // state
  int pixelWidth_  { 2000 };
  int pixelHeight_ { 1500 };

  double aspect_ { 1.0 };

  // Open GL globals

  bool depthTest_   { true };
  bool cullFace_    { true };
  bool frontFace_   { false };
  bool polygonLine_ { false };

  double pointSize_  { 4.0 };
  double lineWidth_  { 2.0 };
  bool   showOrient_ { false };

  bool storeProjected_ { false };

  // lighting
  CRGBA  ambientColor_     { CRGBA::white() };
  double ambientStrength_  { 0.2 };
  double diffuseStrength_  { 1.0 };
  CRGBA  emissiveColor_    { CRGBA::white() };
  double emissiveStrength_ { 0.0 };
  CRGBA  specularColor_    { CRGBA::white() };
  double specularStrength_ { 0.2 };
  bool   fixedDiffuse_     { false };
  bool   flatShaded_       { true };

  // textures
  bool wireframe_ { false };
  bool solid_     { true };
  bool textured_  { true };

  // globals
  CRGBA selectColor_    { CRGBA::yellow() };
  CRGBA wireframeColor_ { CRGBA::white() };

  // interaction
  MouseData mouseData_;

  //---

  using ViewportDatas = std::vector<ViewportData *>;

  ViewportDatas viewportDatas_;
  uint          currentViewport_ { 0 };

  //---

  using Shaders     = std::map<QString, ShaderProgram *>;
  using ShaderDatas = std::vector<ShaderData *>;

  Shaders     shaders_;
  ShaderDatas shaderDatas_;

  //---

  bool invalid_ { true };

  EditType   editType_   { EditType::SELECT };
  SelectType selectType_ { SelectType::FACE };

  //---

  struct DrawData {
    CMatrix3DH worldMatrix;
    CMatrix3DH viewMatrix;
    CMatrix3DH modelMatrix;
  };

  DrawData drawData_;

  //---

  struct DrawSelectionData {
    CQGLBuffer* buffer { nullptr };

    uint lineIndex   { 0 };
    uint vertexIndex { 0 };
    uint endIndex    { 0 };
  };

  DrawSelectionData drawSelectionData_;

  //---

  ParticleData particleData_;

  //---

  // font/text
  Font* font_ { nullptr };
  Texts texts_;

  //---

  CQRubberBand* rubberBand_ { nullptr };
};

}

#endif
