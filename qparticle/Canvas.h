#ifndef Canvas_H
#define Canvas_H

#include <App.h>
#include <FaceData.h>

#include <CImagePtr.h>
#include <CBBox3D.h>
#include <CPoint2D.h>

#include <QGLWidget>
#include <QOpenGLExtraFunctions>

#include <optional>

class CQGLBuffer;
class CQGLTexture;
class CGeomTexture;
class CQRubberBand;

namespace CQTclParticle3D {

class App;
class Camera;
class ShaderProgram;
class Texture;

class Canvas : public QGLWidget, public QOpenGLExtraFunctions {
  Q_OBJECT

 public:
  using SelectType = App::SelectType;
  using EditType   = App::EditType;

 public:
  Canvas(App *app);

  //---

  App *app() const { return app_; }

  int ind() const { return ind_; }
  void setInd(int i) { ind_ = i; }

  //---

  Camera *camera() const { return camera_; }

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

  const QColor &bgColor() const { return bgColor_; }
  void setBgColor(const QColor &c) { bgColor_ = c; }

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

  const CBBox3D &bbox() const { return bbox_; }
  void setBBox(const CBBox3D &v) { bbox_ = v; updateCamera(); updateBBox_ = false; }

  void updateCamera();

  //---

  void initializeGL() override;

  void resizeGL(int, int) override;

  void paintGL() override;

  //---

  void mousePressEvent  (QMouseEvent *e) override;
  void mouseMoveEvent   (QMouseEvent *e) override;
  void mouseReleaseEvent(QMouseEvent *e) override;

  void wheelEvent(QWheelEvent *e) override;

  void keyPressEvent(QKeyEvent *event) override;

  //---

  QSize sizeHint() const override { return QSize(1536, 1536); }

  //---

  void addScene ();
  void drawScene();

  void drawSelection();

  void addParticles();
  void drawParticles();

  //---

  void selectFaceAt  (const CPoint2D &p, bool flip=false);
  void selectFaceIn  (const CPoint2D &p1, const CPoint2D &p2, bool flip=false);
  void selectEdgeAt  (const CPoint2D &p, bool flip=false);
  void selectVertexAt(const CPoint2D &p, bool flip=false);
  void selectVertexIn(const CPoint2D &p1, const CPoint2D &p2, bool flip=false);

  void deselectAll();

  //---

  CPoint2D pixelToView(const CPoint2D &p) const;

  //---

  void updateStatus();

 private:
  ShaderProgram *getShader(const QString &vertex, const QString &fragment);

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

  using Shaders = std::map<QString, ShaderProgram *>;

  //---

  App* app_ { nullptr };

  int ind_ { 0 };

  // camera
  Camera* camera_ { nullptr };

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
  QColor bgColor_ { 40, 40, 40 };

  CRGBA selectColor_    { CRGBA::yellow() };
  CRGBA wireframeColor_ { CRGBA::white() };

  // interaction
  MouseData mouseData_;

  CBBox3D bbox_;

  Shaders shaders_;

  bool invalid_    { true };
  bool updateBBox_ { true };

  EditType   editType_   { EditType::TCL };
  SelectType selectType_ { SelectType::FACE };

  //---

  struct SelectionData {
    CQGLBuffer* buffer { nullptr };

    uint lineIndex   { 0 };
    uint vertexIndex { 0 };
    uint endIndex    { 0 };
  };

  SelectionData selectionData_;

  //---

  struct ParticleData {
    CQGLBuffer*           buffer { nullptr };
    std::vector<FaceData> faceDatas;
  };

  ParticleData particleData_;

  //---

  CQRubberBand* rubberBand_ { nullptr };
};

}

#endif
