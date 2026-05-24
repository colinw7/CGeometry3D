#include <Canvas.h>
#include <App.h>
#include <Status.h>
#include <ShaderProgram.h>
#include <Camera.h>
#include <Texture.h>
#include <GeomObject.h>
#include <Util.h>

#include <CQGLBuffer.h>
#include <CQGLTexture.h>
#include <CQGLUtil.h>

#include <CGeomScene3D.h>
#include <CGeomFace3D.h>
#include <CGeomEdge3D.h>

#include <CQRubberBand.h>

#include <CTclUtil.h>
#include <CBBox3D.h>

#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>

namespace CQTclModel3DView {

Canvas::
Canvas(App *app) :
 app_(app)
{
  setObjectName("canvas");

  setFocusPolicy(Qt::StrongFocus);

  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  //---

  camera_ = new Camera(app_);

  camera_->setOrigin(CVector3D(0, 0, 0));
  camera_->setDistance(5);

  //---

  rubberBand_ = new CQRubberBand(this);

  //---

  connect(app_, SIGNAL(modelAdded()), this, SLOT(updateScene()));

  connect(camera_, SIGNAL(stateChangedSignal()), this, SLOT(update()));

  //---

  updateStatus();
}

//---

ShaderProgram *
Canvas::
sceneShaderProgram()
{
  return getShader("scene.vs", "scene.fs");
}

ShaderProgram *
Canvas::
selectionShaderProgram()
{
  return getShader("selection.vs", "selection.fs");
}

//---

void
Canvas::
initializeGL()
{
  initializeOpenGLFunctions();
}

void
Canvas::
resizeGL(int width, int height)
{
  setPixelWidth (width);
  setPixelHeight(height);

  glViewport(0, 0, width, height);

  setAspect(double(width)/double(height));

  camera_->setAspect(aspect());
}

void
Canvas::
paintGL()
{
  if (invalid_) {
    addScene();

    invalid_ = false;
  }

  //---

  glClearColor(bgColor_.redF(), bgColor_.greenF(), bgColor_.blueF(), 1.0f);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //---

  // set GL state
  enableDepthTest  ();
  enableCullFace   ();
  enableFrontFace  ();
  enablePolygonLine();

  glPointSize(pointSize());
  glLineWidth(lineWidth());

  //---

  drawScene();
}

//---

void
Canvas::
updateScene(bool updateBBox)
{
  updateBBox_ = updateBBox;

  invalid_ = true;

  update();
}

void
Canvas::
addScene()
{
  if (updateBBox_)
    bbox_ = CBBox3D();

  //---

  auto *scene = app_->scene();

  for (auto *object : scene->getObjects()) {
    auto *object1 = dynamic_cast<GeomObject *>(object);
    assert(object1);

    //---

    auto modelMatrix = CMatrix3DH(object1->getHierTransform());
    auto meshMatrix  = CMatrix3DH(object->getMeshGlobalTransform());

    //---

    auto *buffer = object1->initBuffer(this);

    //---

    auto *objectMaterial = object->getMaterialP();

    auto *diffuseTexture  = object->getDiffuseTexture();
    auto *normalTexture   = object->getNormalTexture();
    auto *specularTexture = object->getSpecularTexture();
    auto *emissiveTexture = object->getEmissiveTexture();

    //---

    CBBox3D bbox1;

    int pos = 0;

    auto addFaceData = [&](CGeomFace3D *face, bool reverse=false) {
      FaceData faceData;

      faceData.face = const_cast<CGeomFace3D *>(face);

      //---

      auto *faceMaterial = faceData.face->getMaterialP();

      if (! faceMaterial && objectMaterial)
        faceMaterial = objectMaterial;

      //---

      faceData.diffuse = face->color().value_or(CRGBA(1, 1, 1));

      if (faceMaterial && faceMaterial->diffuse())
        faceData.diffuse = faceMaterial->diffuse().value();

      //---

      // set face textures
      auto *diffuseTexture1  = face->getDiffuseTexture();
      auto *normalTexture1   = face->getNormalTexture();
      auto *specularTexture1 = face->getSpecularTexture();
      auto *emissiveTexture1 = face->getEmissiveTexture();

      if (! diffuseTexture1 ) diffuseTexture1  = diffuseTexture;
      if (! normalTexture1  ) normalTexture1   = normalTexture;
      if (! specularTexture1) specularTexture1 = specularTexture;
      if (! emissiveTexture1) emissiveTexture1 = emissiveTexture;

      if (faceMaterial) {
        if (faceMaterial->diffuseTexture ()) diffuseTexture1  = faceMaterial->diffuseTexture ();
        if (faceMaterial->normalTexture  ()) normalTexture1   = faceMaterial->normalTexture  ();
        if (faceMaterial->specularTexture()) specularTexture1 = faceMaterial->specularTexture();
        if (faceMaterial->emissiveTexture()) emissiveTexture1 = faceMaterial->emissiveTexture();
      }

      if (diffuseTexture1)
        faceData.diffuseTexture = getGLTexture(diffuseTexture1, /*add*/true);

      if (normalTexture1)
        faceData.normalTexture = getGLTexture(normalTexture1, /*add*/true);

      if (specularTexture1)
        faceData.specularTexture = getGLTexture(specularTexture1, /*add*/true);

      if (emissiveTexture1)
        faceData.emissiveTexture = getGLTexture(emissiveTexture1, /*add*/true);

      //---

      auto vertices = face->getVertices();

      if (reverse)
        std::reverse(vertices.begin(), vertices.end());

      //--

      // get face normal
      CVector3D normal;

      if (face->getNormalSet())
        normal = face->getNormal();
      else {
        for (const auto &v : vertices) {
          auto &vertex = object->getVertex(v);

          vertex.setViewed(vertex.getModel());
        }

        face->calcModelNormal(normal);
      }

      //---

      faceData.pos = pos;
      faceData.len = int(vertices.size());

      int iv = 0;

      for (const auto &v : vertices) {
        faceData.vertices.push_back(v);

        auto *vertex = object->getVertexP(v);

        const auto &model = vertex->getModel();

        auto model1 = meshMatrix *model;
        auto model2 = modelMatrix*model1;

        vertex->setViewed(model2);

        //---

        // update color, normal for custom vertex value

        auto normal1 = normal;
        auto color1  = faceData.diffuse;

        if (! isFlatShaded()) {
          if (vertex->hasNormal())
            normal1 = vertex->getNormal();
        }

        if (vertex->hasColor())
          color1 = vertex->getColor();

        //---

        if (faceData.normalTexture) {
          CPoint2D tpoint;

          if (vertex->hasTextureMap())
            tpoint = vertex->getTextureMap();
          else
            tpoint = face->getTexturePoint(*vertex, iv);

          int tw = faceData.normalTexture->getWidth ();
          int th = faceData.normalTexture->getHeight();

          auto tx = CMathUtil::clamp(tpoint.x, 0.0, 1.0);
          auto ty = CMathUtil::clamp(tpoint.y, 0.0, 1.0);

          // get normal value from texture
          auto rgba = faceData.normalTexture->getImage().pixel(tx*(tw - 1), ty*(th - 1));
          auto tnormal = CVector3D(qRed(rgba)/255.0, qGreen(rgba)/255.0, qBlue(rgba)/255.0);

          // this normal is in tangent space
          normal1 = (tnormal*2.0 - CVector3D(1.0, 1.0, 1.0)).normalized();
        }

        //---

        buffer->addInd(vertex->getInd());

        buffer->addPoint(float(model.x), float(model.y), float(model.z));

        buffer->addNormal(float(normal1.getX()), float(normal1.getY()), float(normal1.getZ()));

        buffer->addColor(color1);

        //---

#if 0
        const auto &jointData = vertex->getJointData();

        if (jointData.nodeDatas[0].node >= 0) {
          for (int i = 0; i < 4; ++i) {
            boneNodeIds[i] = jointData.nodeDatas[i].node;
            boneWeights[i] = jointData.nodeDatas[i].weight;
          }

          buffer->addBoneIds    (boneNodeIds[0], boneNodeIds[1], boneNodeIds[2], boneNodeIds[3]);
          buffer->addBoneWeights(boneWeights[0], boneWeights[1], boneWeights[2], boneWeights[3]);
        }
#endif

        //---

        if (faceData.diffuseTexture) {
          const auto &tpoint = face->getTexturePoint(*vertex, iv);

          buffer->addTexturePoint(float(tpoint.x), float(tpoint.y));
        }
        else
          buffer->addTexturePoint(0.0f, 0.0f);

        //---

        ++iv;

        bbox1 += model2;
      }

      pos += faceData.len;

      object1->addFaceData(faceData);
    };

    //---

    const auto &faces = object->getFaces();

    for (auto *face : faces) {
      addFaceData(face);

      auto *faceMaterial = const_cast<CGeomFace3D *>(face)->getMaterialP();

      if (! faceMaterial && objectMaterial)
        faceMaterial = objectMaterial;

      if (face->getTwoSided() || (faceMaterial && faceMaterial->isTwoSided()))
        addFaceData(face, /*reverse*/true);
    }

    //---

    const auto &lines = object->getLines();

    for (const auto *line : lines) {
      FaceData faceData;

      faceData.line = const_cast<CGeomLine3D *>(line);

      //---

      auto color = line->getColor();

      //---

      auto v1 = line->getStartInd();
      auto v2 = line->getEndInd  ();

      std::vector<uint> vertices;

      vertices.push_back(v1);
      vertices.push_back(v2);

      //--

      faceData.pos = pos;
      faceData.len = int(vertices.size());

      int iv = 0;

      for (const auto &v : vertices) {
        faceData.vertices.push_back(v);

        const auto &vertex = object->getVertex(v);
        const auto &model  = vertex.getModel();

        auto model1 = meshMatrix *model;
        auto model2 = modelMatrix*model1;

        //---

        // update color, normal for custom vertex value

        auto color1 = color;

        if (vertex.hasColor())
          color1 = vertex.getColor();

        //---

        buffer->addInd(vertex.getInd());

        buffer->addPoint(float(model.x), float(model.y), float(model.z));

        buffer->addNormal(0, 0, 1);

        buffer->addColor(color1);

        buffer->addTexturePoint(0.0f, 0.0f);

        //---

        ++iv;

        bbox1 += model2;
      }

      pos += faceData.len;

      object1->addFaceData(faceData);
    }

    //---

    object1->setBBox(bbox1);

    if (updateBBox_)
      bbox_ += bbox1;

    //---

    buffer->load();
  }

  //---

  if (! bbox_.isSet()) {
    bbox_.add(CPoint3D(-1, -1, -1));
    bbox_.add(CPoint3D( 1,  1,  1));
  }

  //---

  auto c = bbox_.getCenter();
  auto d = bbox_.getMaxSize();

  camera_->setOrigin(CVector3D(c));
  camera_->setDistance(std::sqrt(2.0)*d);
}

void
Canvas::
drawScene()
{
  auto *program = this->sceneShaderProgram();

  program->bind();

  //---

  auto worldMatrix = camera_->perspectiveMatrix();
  auto viewMatrix  = camera_->viewMatrix();
  auto viewPos     = camera_->position();

  // camera projection
  program->setUniformValue("projection", CQGLUtil::toQMatrix(worldMatrix));

  // camera/view transformation
  program->setUniformValue("view", CQGLUtil::toQMatrix(viewMatrix));

  // view pos
  program->setUniformValue("viewPos", CQGLUtil::toVector(viewPos));

  //---

#if 0
  // add light data to shader program
  addShaderLights(program);
#else
  program->setUniformValue("ambientColor", CQGLUtil::toVector(ambientColor()));
  program->setUniformValue("ambientStrength", float(ambientStrength()));

  program->setUniformValue("diffuseStrength", float(diffuseStrength()));

  program->setUniformValue("emissionColor"   , CQGLUtil::toVector(emissiveColor()));
  program->setUniformValue("emissiveStrength", float(emissiveStrength()));

  program->setUniformValue("specularColor"   , CQGLUtil::toVector(specularColor()));
  program->setUniformValue("specularStrength", float(specularStrength()));

  program->setUniformValue("fixedDiffuse", isFixedDiffuse());
#endif

  //---

  glPointSize(pointSize());
  glLineWidth(lineWidth());

#if 0
  paintData_.reset();
#endif

  //---

#if 0
  bool isAnim = (app_->animName() != "");
#endif

  //---

  auto selectColor    = this->selectColor();
  auto wireframeColor = this->wireframeColor();

  auto wireframeTransparency = 0.5f;

  program->setUniformValue("selectColor", CQGLUtil::toVector(ColorToVector(selectColor)));
  program->setUniformValue("wireframeColor", CQGLUtil::toVector(ColorToVector(wireframeColor)));

  program->setUniformValue("wireframeTransparency", wireframeTransparency);

  //---

  auto *scene = app_->scene();

  for (auto *object : scene->getObjects()) {
    if (! object->getVisible())
      continue;

    auto *object1 = dynamic_cast<GeomObject *>(object);
    assert(object1);

    if (! object1->buffer())
      continue;

    //---

    // mesh matrix
    auto meshMatrix = object->getMeshGlobalTransform();
    program->setUniformValue("meshMatrix", CQGLUtil::toQMatrix(meshMatrix));

    //---

    // model matrix
    //auto modelMatrix = CMatrix3DH::identity();
    auto modelMatrix = object1->getHierTransform();
    program->setUniformValue("model", CQGLUtil::toQMatrix(modelMatrix));

    //---

#if 0
    // anim
    program->setUniformValue("useBonePoints", isAnim); // per object ?

    if (isAnim) {
      updateNodeMatrices(object);

      program->setUniformValueArray("globalBoneTransform",
        &paintData_.nodeQMatrices[0], PaintData::NUM_NODE_MATRICES);
    }
#endif

    //---

    bool objectSelected = object->getHierSelected();

    object1->buffer()->bind();

    //---

    auto *objectMaterial = object->getMaterialP();

    //---

    auto drawFace = [&](const FaceData &faceData, double transparency) {
      bool faceSelected = (faceData.face ? faceData.face->getSelected() : false);

      bool selected = objectSelected || faceSelected;

      program->setUniformValue("isSelected", selected);

      //---

      bool useDiffuseTexture = (isTextured() ? !!faceData.diffuseTexture : false);

      program->setUniformValue("diffuseTexture.enabled", useDiffuseTexture);

      if (useDiffuseTexture) {
        glActiveTexture(GL_TEXTURE0);
        faceData.diffuseTexture->bind();

        program->setUniformValue("diffuseTexture.texture", 0);
      }

      //---

      bool useNormalTexture = (isTextured() ? !!faceData.normalTexture : false);

      program->setUniformValue("normalTexture.enabled", useNormalTexture);

      if (useNormalTexture) {
        glActiveTexture(GL_TEXTURE1);
        faceData.normalTexture->bind();

        program->setUniformValue("normalTexture.texture", 1);
      }

      //---

      bool useSpecularTexture = (isTextured() ? !!faceData.specularTexture : false);

      program->setUniformValue("specularTexture.enabled", useSpecularTexture);

      if (useSpecularTexture) {
        glActiveTexture(GL_TEXTURE2);
        faceData.specularTexture->bind();

        program->setUniformValue("specularTexture.texture", 2);
      }

      //---

      bool useEmissiveTexture = (isTextured() ? !!faceData.emissiveTexture : false);

      program->setUniformValue("emissiveTexture.enabled", useEmissiveTexture);

      if (useEmissiveTexture) {
        glActiveTexture(GL_TEXTURE3);
        faceData.emissiveTexture->bind();

        program->setUniformValue("emissiveTexture.texture", 3);
      }

      program->setUniformValue("emissionColor", CQGLUtil::toVector(faceData.emission));

      //---

      program->setUniformValue("shininess", float(faceData.shininess));

      program->setUniformValue("transparency", float(1.0 - transparency));

      //---

      program->setUniformValue("isLine", false);

      if (isWireframe() || selected) {
        program->setUniformValue("isWireframe", true);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        glDrawArrays(GL_TRIANGLE_FAN, faceData.pos, faceData.len);
      }

      if (isSolid()) {
        program->setUniformValue("isWireframe", false);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        glDrawArrays(GL_TRIANGLE_FAN, faceData.pos, faceData.len);
      }

#if 0
      if (isPoints()) {
        program->setUniformValue("isWireframe", true);

        glDrawArrays(GL_POINTS, faceData.pos, faceData.len);
      }
#endif

      // set view and project point
      for (int i = 0; i < faceData.len; ++i) {
        CQGLBuffer::PointData data;
        object1->buffer()->getPointData(faceData.pos + i, data);

        auto p1 = viewMatrix*data.point->point();
        auto p2 = worldMatrix*p1;

        auto &vertex = object->getVertex(data.ind.value());

        vertex.setViewed   (p1);
        vertex.setProjected(p2);
      }
    };

    //---

#if 0
    bool anyTransparent = false;
#endif

    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);

    for (const auto &faceData : object1->faceDatas()) {
      if      (faceData.face) {
        auto *face = faceData.face;

        if (! face->getVisible())
          continue;

        auto *faceMaterial = face->getMaterialP();

        if (! faceMaterial && objectMaterial)
          faceMaterial = objectMaterial;

#if 0
        if (faceMaterial && faceMaterial->transparency() > 0.0) {
          anyTransparent = true;
          continue;
        }
#endif

        drawFace(faceData, 0.0);
      }
      else if (faceData.line) {
        auto *line = faceData.line;

        if (! line->getVisible())
          continue;

        program->setUniformValue("isWireframe", true);
        program->setUniformValue("isLine"     , true);

        glDrawArrays(GL_LINES, faceData.pos, faceData.len);
      }
      else
        assert(false);
    }

#if 0
    if (anyTransparent) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glDepthMask(GL_FALSE);

      for (const auto &faceData : object1->faceDatas()) {
        auto *face = faceData.face;

        if (! face->getVisible())
          continue;

        auto *faceMaterial = face->getMaterialP();

        if (! faceMaterial && objectMaterial)
          faceMaterial = objectMaterial;

        if (! faceMaterial || faceMaterial->transparency() <= 0.0)
          continue;

        drawFace(faceData, faceMaterial->transparency());
      }

      glDisable(GL_BLEND);
      glDepthMask(GL_TRUE);
    }
#endif

    //---

    object1->buffer()->unbind();
  }

  //---

  program->release();

  //---

  drawSelection();
}

void
Canvas::
drawSelection()
{
  auto *program = selectionShaderProgram();

  //---

  if (! selectionData_.buffer)
    selectionData_.buffer = program->createBuffer();

  selectionData_.buffer->clearBuffers();

  //---

  auto color = CRGBA::yellow();

  auto addLine = [&](const CPoint3D &p1, const CPoint3D &p2) {
    selectionData_.buffer->addPoint(float(p1.x), float(p1.y), float(p1.z));
    selectionData_.buffer->addColor(color);

    selectionData_.buffer->addPoint(float(p2.x), float(p2.y), float(p2.z));
    selectionData_.buffer->addColor(color);
  };

  auto addPoint = [&](const CPoint3D &p) {
    selectionData_.buffer->addPoint(float(p.x), float(p.y), float(p.z));
    selectionData_.buffer->addColor(color);
  };

  //---

  auto *scene = app_->scene();

  selectionData_.lineIndex = 0;

  for (auto *object : scene->getObjects()) {
    auto *object1 = dynamic_cast<GeomObject *>(object);
    assert(object1);

    const auto &edges = object->getEdges();

    for (auto *e : edges) {
      if (e->getSelected())
        addLine(e->modelStart(), e->modelEnd());
    }
  }

  selectionData_.vertexIndex = selectionData_.buffer->numPoints();

  for (auto *object : scene->getObjects()) {
    auto *object1 = dynamic_cast<GeomObject *>(object);
    assert(object1);

    const auto &vertices = object->getVertices();

    for (auto *v : vertices) {
      if (v && v->getSelected())
        addPoint(v->getModel());
    }
  }

  selectionData_.endIndex = selectionData_.buffer->numPoints();

  //---

  selectionData_.buffer->load();

  //---

  selectionData_.buffer->bind();

  program->bind();

  //---

  // camera projection
  auto projectionMatrix = camera_->worldMatrix();
  program->setUniformValue("projection", CQGLUtil::toQMatrix(projectionMatrix));

  // camera/view transformation
  auto viewMatrix = camera_->viewMatrix();
  program->setUniformValue("view", CQGLUtil::toQMatrix(viewMatrix));

  // model matrix
  auto modelMatrix = CMatrix3DH::identity();
  program->setUniformValue("model", CQGLUtil::toQMatrix(modelMatrix));

  //---

  if (selectionData_.vertexIndex > selectionData_.lineIndex) {
    glLineWidth(8);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glDrawArrays(GL_LINES, selectionData_.lineIndex,
                 selectionData_.vertexIndex - selectionData_.lineIndex);

    glLineWidth(1);
  }

  if (selectionData_.endIndex > selectionData_.vertexIndex) {
    glPointSize(8);

    glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);

    glDrawArrays(GL_POINTS, selectionData_.vertexIndex,
                 selectionData_.endIndex - selectionData_.vertexIndex);

    glPointSize(1);
  }

  //---

  selectionData_.buffer->unbind();

  //---

  program->release();
}

//---

void
Canvas::
mousePressEvent(QMouseEvent *e)
{
  mouseData_.pressed = true;
  mouseData_.button  = e->button();
  mouseData_.press   = CPoint2D(e->x(), e->y());
  mouseData_.move    = mouseData_.press;

  mouseData_.isShift   = (e->modifiers() & Qt::ShiftModifier);
  mouseData_.isControl = (e->modifiers() & Qt::ControlModifier);

  if (mouseData_.button == Qt::LeftButton) {
    rubberBand_->setBounds(QPoint(mouseData_.press.x, mouseData_.press.y),
                           QPoint(mouseData_.move .x, mouseData_.move .y));
    rubberBand_->show();
  }
}

void
Canvas::
mouseMoveEvent(QMouseEvent *e)
{
  mouseData_.move.x = e->x();
  mouseData_.move.y = e->y();

  mouseData_.isShift   = (e->modifiers() & Qt::ShiftModifier);
  mouseData_.isControl = (e->modifiers() & Qt::ControlModifier);

  //---

  auto dx = CMathUtil::sign(mouseData_.move.x - mouseData_.press.x);
  auto dy = CMathUtil::sign(mouseData_.move.y - mouseData_.press.y);

  if      (mouseData_.button == Qt::LeftButton) {
    rubberBand_->setBounds(QPoint(mouseData_.press.x, mouseData_.press.y),
                           QPoint(mouseData_.move .x, mouseData_.move .y));
  }
  else if (mouseData_.button == Qt::MiddleButton) {
    auto da = M_PI/180.0;

    camera_->rotateY(-dx*da);
    camera_->rotateX(-dy*da);
  }
  else if (mouseData_.button == Qt::RightButton) {
    camera_->moveRight(-dx/10.0);
    camera_->moveUp   ( dy/10.0);
  }

  //---

  //mouseData_.press = mouseData_.move;
}

void
Canvas::
mouseReleaseEvent(QMouseEvent *e)
{
  mouseData_.move.x = e->x();
  mouseData_.move.y = e->y();

  if (mouseData_.button == Qt::LeftButton) {
    auto dx = std::abs(mouseData_.press.x - mouseData_.move.x);
    auto dy = std::abs(mouseData_.press.y - mouseData_.move.y);

    if (editType() == EditType::SELECT) {
      bool flip = mouseData_.isControl;

      if (! mouseData_.isShift && ! mouseData_.isControl)
        deselectAllI();

      if (dx < 4 && dy < 4) {
        if      (selectType() == SelectType::FACE)
          selectFaceAt(mouseData_.press, flip);
        else if (selectType() == SelectType::EDGE)
          selectEdgeAt(mouseData_.press, flip);
        else if (selectType() == SelectType::POINT)
          selectVertexAt(mouseData_.press, flip);
      }
      else {
        if      (selectType() == SelectType::FACE)
          selectFaceIn(mouseData_.press, mouseData_.move, flip);
        else if (selectType() == SelectType::EDGE)
          selectEdgeAt(mouseData_.press, flip);
        else if (selectType() == SelectType::POINT)
          selectVertexIn(mouseData_.press, mouseData_.move, flip);
      }
    }

    rubberBand_->hide();
  }

  mouseData_.pressed = false;
  mouseData_.button  = Qt::NoButton;
}

void
Canvas::
wheelEvent(QWheelEvent *e)
{
  auto d  = bbox_.getMaxSize()/100.0;
  auto dw = e->angleDelta().y()/250.0;

  camera_->setDistance(camera_->distance() - dw*d);
}

void
Canvas::
keyPressEvent(QKeyEvent *e)
{
  mouseData_.isControl = (e->modifiers() & Qt::ControlModifier);
  mouseData_.isShift   = (e->modifiers() & Qt::ShiftModifier);

  auto k = e->key();

  auto d  = bbox_.getMaxSize()/100.0;
  auto da = M_PI/60.0;

  if (k == Qt::Key_Escape) {
    if      (editType() == EditType::SELECT)
      setEditType(EditType::CAMERA);
    else if (editType() == EditType::CAMERA)
      setEditType(EditType::TCL);
    else
      setEditType(EditType::SELECT);
    return;
  }

  if      (editType() == EditType::TCL) {
    std::string keyStr;

    if      (e->key() == Qt::Key_Left ) keyStr = "left";
    else if (e->key() == Qt::Key_Right) keyStr = "right";
    else if (e->key() == Qt::Key_Up   ) keyStr = "up";
    else if (e->key() == Qt::Key_Down ) keyStr = "down";
    else if (e->key() == Qt::Key_Space) keyStr = "space";
    else                                keyStr = e->text().toStdString();

    if (keyStr == "")
      keyStr = "key." + std::to_string(e->key());

    std::vector<std::string> args;

    args.push_back(keyStr);

    args.push_back(mouseData_.isControl ? "1" : "0");
    args.push_back(mouseData_.isShift   ? "1" : "0");

    auto argList = app_->tcl()->mergeList(args);

    auto cmd = "keyPress " + argList;

    app_->runTclCmd(cmd);
  }
  else if (editType() == EditType::CAMERA) {
    if      (k == Qt::Key_Left) {
      camera_->moveRight(-d);
    }
    else if (k == Qt::Key_Right) {
      camera_->moveRight(d);
    }
    else if (k == Qt::Key_Up) {
      camera_->moveUp(d);
    }
    else if (k == Qt::Key_Down) {
      camera_->moveUp(-d);
    }
    else if (k == Qt::Key_Plus) {
      camera_->moveFront(d);
    }
    else if (k == Qt::Key_Minus) {
      camera_->moveFront(-d);
    }
    else if (k == Qt::Key_W) {
      camera_->rotateX(da);
    }
    else if (k == Qt::Key_S) {
      camera_->rotateX(-da);
    }
    else if (k == Qt::Key_A) {
      camera_->rotateY(da);
    }
    else if (k == Qt::Key_D) {
      camera_->rotateY(-da);
    }
    else if (k == Qt::Key_Q) {
      camera_->rotateZ(-da);
    }
    else if (k == Qt::Key_E) {
      camera_->rotateZ(da);
    }
    else if (k == Qt::Key_Space) {
      camera_->printMatrices();
    }
  }
}

//---

void
Canvas::
selectFaceAt(const CPoint2D &pos, bool flip)
{
  auto p = pixelToView(pos);
  //std::cerr << "PX: " << p.x << " " << p.y << "\n";

  double       minDist = 0.0;
  CGeomFace3D *minFace = nullptr;

  auto *scene = app_->scene();

  for (auto *object : scene->getObjects()) {
    if (! object->getVisible())
      continue;

    auto *object1 = dynamic_cast<GeomObject *>(object);
    assert(object1);

    const auto &faces = object->getFaces();

    for (auto *face : faces) {
      if (! face->getVisible())
        continue;

      auto orient = face->getProjectedOrientation();

      if (isCullFace()) {
        if (isFrontFace()) {
          if (orient == CPolygonOrientation::ANTICLOCKWISE)
            continue;
        }
        else {
          if (orient == CPolygonOrientation::CLOCKWISE)
            continue;
        }
      }

      auto c = face->calcProjectedCenter().toPoint2D();

      auto dist = p.distanceTo(c);

      if (! minFace || dist < minDist) {
        minFace = face;
        minDist = dist;
      }
    }
  }

  if (minFace) {
    if (flip)
      minFace->setSelected(! minFace->getSelected());
    else
      minFace->setSelected(true);
  }

  emitSelectionChanged();

  update();
}

void
Canvas::
selectFaceIn(const CPoint2D &p1, const CPoint2D &p2, bool flip)
{
  auto pv1 = pixelToView(p1);
  auto pv2 = pixelToView(p2);

  auto rect = CBBox2D(pv1, pv2);

  std::vector<CGeomFace3D *> faces1;

  auto *scene = app_->scene();

  for (auto *object : scene->getObjects()) {
    if (! object->getVisible())
      continue;

    auto *object1 = dynamic_cast<GeomObject *>(object);
    assert(object1);

    const auto &faces = object->getFaces();

    for (auto *face : faces) {
      if (! face->getVisible())
        continue;

      if (isCullFace()) {
        auto orient = face->getProjectedOrientation();

        if (isFrontFace()) {
          if (orient == CPolygonOrientation::ANTICLOCKWISE)
            continue;
        }
        else {
          if (orient == CPolygonOrientation::CLOCKWISE)
            continue;
        }
      }

      auto c = face->calcProjectedCenter().toPoint2D();

      if (rect.inside(c))
        faces1.push_back(face);
    }
  }

  for (auto *face : faces1) {
    if (flip)
      face->setSelected(! face->getSelected());
    else
      face->setSelected(true);
  }

  emitSelectionChanged();

  update();
}

void
Canvas::
selectEdgeAt(const CPoint2D &pos, bool flip)
{
  auto p = pixelToView(pos);
  //std::cerr << "PX: " << p.x << " " << p.y << "\n";

  double       minDist = 0.0;
  CGeomEdge3D *minEdge = nullptr;

  auto *scene = app_->scene();

  for (auto *object : scene->getObjects()) {
    if (! object->getVisible())
      continue;

    auto *object1 = dynamic_cast<GeomObject *>(object);
    assert(object1);

    const auto &edges = object->getEdges();

    for (auto *edge : edges) {
      if (! edge->getVisible())
        continue;

      auto c = edge->calcProjectedCenter().toPoint2D();

      auto dist = p.distanceTo(c);

      if (! minEdge || dist < minDist) {
        minEdge = edge;
        minDist = dist;
      }
    }
  }

  if (minEdge) {
    if (flip)
      minEdge->setSelected(! minEdge->getSelected());
    else
      minEdge->setSelected(true);
  }

  emitSelectionChanged();

  update();
}

void
Canvas::
selectVertexAt(const CPoint2D &pos, bool flip)
{
  auto p = pixelToView(pos);
  //std::cerr << "PX: " << p.x << " " << p.y << "\n";

  double         minDist = 0.0;
  CGeomVertex3D *minVertex = nullptr;

  auto *scene = app_->scene();

  for (auto *object : scene->getObjects()) {
    if (! object->getVisible())
      continue;

    auto *object1 = dynamic_cast<GeomObject *>(object);
    assert(object1);

    const auto &vertices = object->getVertices();

    for (auto *vertex : vertices) {
      if (! vertex || ! vertex->getVisible())
        continue;

      auto c = vertex->getProjected().toPoint2D();

      auto dist = p.distanceTo(c);

      if (! minVertex || dist < minDist) {
        minVertex = vertex;
        minDist = dist;
      }
    }
  }

  if (minVertex) {
    if (flip)
      minVertex->setSelected(! minVertex->getSelected());
    else
      minVertex->setSelected(true);
  }

  emitSelectionChanged();

  update();
}

void
Canvas::
selectVertexIn(const CPoint2D &p1, const CPoint2D &p2, bool flip)
{
  auto pv1 = pixelToView(p1);
  auto pv2 = pixelToView(p2);

  auto rect = CBBox2D(pv1, pv2);

  std::vector<CGeomVertex3D *> vertices1;

  auto *scene = app_->scene();

  for (auto *object : scene->getObjects()) {
    if (! object->getVisible())
      continue;

    auto *object1 = dynamic_cast<GeomObject *>(object);
    assert(object1);

    const auto &vertices = object->getVertices();

    for (auto *vertex : vertices) {
      if (! vertex || ! vertex->getVisible())
        continue;

      auto c = vertex->getProjected().toPoint2D();

      if (rect.inside(c))
        vertices1.push_back(vertex);
    }
  }

  for (auto *vertex : vertices1) {
    if (flip)
      vertex->setSelected(! vertex->getSelected());
    else
      vertex->setSelected(true);
  }

  emitSelectionChanged();

  update();
}

void
Canvas::
deselectAll()
{
  deselectAllI();

  emitSelectionChanged();

  update();
}

void
Canvas::
deselectAllI()
{
  auto *scene = app_->scene();

  for (auto *object : scene->getObjects()) {
    if (object->getSelected())
      object->setSelected(false);

    const auto &faces = object->getFaces();

    for (auto *face : faces) {
      if (face->getSelected())
        face->setSelected(false);
    }

    const auto &edges = object->getEdges();

    for (auto *edge : edges) {
      if (edge->getSelected())
        edge->setSelected(false);
    }

    const auto &vertices = object->getVertices();

    for (auto *vertex : vertices) {
      if (vertex && vertex->getSelected())
        vertex->setSelected(false);
    }
  }
}

void
Canvas::
emitSelectionChanged()
{
  Q_EMIT selectionChanged();

  auto cmd = "selectionProc";

  app_->runTclCmd(cmd);
}

//---

// convert pixel (mouse) position to GL coords
CPoint2D
Canvas::
pixelToView(const CPoint2D &p) const
{
  CPoint2D v;

  auto pixelWidth  = double(std::max(this->pixelWidth (), 1));
  auto pixelHeight = double(std::max(this->pixelHeight(), 1));

  auto aspect = this->aspect();

  if (aspect > 1.0) {
    v.x = CMathUtil::map(p.x, 0, pixelWidth  - 1.0, -aspect,  aspect);
    v.y = CMathUtil::map(p.y, 0, pixelHeight - 1.0,     1.0,    -1.0);
  }
  else {
    v.x = CMathUtil::map(p.x, 0, pixelWidth  - 1.0,   -1.0,      1.0);
    v.y = CMathUtil::map(p.y, 0, pixelHeight - 1.0,  aspect, -aspect);
  }

  return v;
}

//---

void
Canvas::
updateStatus()
{
  auto *status = app_->status();

  QString text;

  text += "Edit: ";

  if      (editType() == EditType::CAMERA) text += "Camera";
  else if (editType() == EditType::SELECT) text += "Select";
  else if (editType() == EditType::TCL   ) text += "Tcl";
  else                                     text += "???";

  text += " Select: ";

  if      (selectType() == SelectType::FACE ) text += "Face";
  else if (selectType() == SelectType::EDGE ) text += "Edge";
  else if (selectType() == SelectType::POINT) text += "Point";
  else                                        text += "???";

  status->setText(text);
}

//---

void
Canvas::
enableDepthTest()
{
  if (isDepthTest())
    glEnable(GL_DEPTH_TEST);
  else
    glDisable(GL_DEPTH_TEST);
}

//---

void
Canvas::
setCullFace(bool b)
{
  cullFace_ = b;

  enableCullFace();
}

void
Canvas::
enableCullFace()
{
  if (isCullFace())
    glEnable(GL_CULL_FACE);
  else
    glDisable(GL_CULL_FACE);
}

//---

void
Canvas::
setFrontFace(bool b)
{
  frontFace_ = b;

  enableFrontFace();
}

void
Canvas::
enableFrontFace()
{
  glFrontFace(isFrontFace() ? GL_CW : GL_CCW);
}

//---

void
Canvas::
enablePolygonLine()
{
  if (isPolygonLine())
    glEnable(GL_POLYGON_OFFSET_LINE);
  else
    glDisable(GL_POLYGON_OFFSET_LINE);

  glPolygonOffset(-1.0f, -1.0f);
}

//---

ShaderProgram *
Canvas::
getShader(const QString &vertex, const QString &fragment)
{
  auto id = QString("V:%1,F:%2").arg(vertex).arg(fragment);

  auto ps = shaders_.find(id);

  if (ps == shaders_.end()) {
    auto *shaderProgram = new ShaderProgram(app_);

    shaderProgram->addShaders(vertex, fragment);

    ps = shaders_.insert(ps, Shaders::value_type(id, shaderProgram));
  }

  return (*ps).second;
}

CQGLTexture *
Canvas::
getGLTexture(CGeomTexture *texture, bool /*add*/)
{
  auto *texture1 = dynamic_cast<Texture *>(texture);
  assert(texture1);

  if (! texture1->glTexture(this)) {
    //if (! add) return nullptr;

    initGLTexture(texture1);

    Q_EMIT textureAdded();
  }

  return texture1->glTexture(this);
}

void
Canvas::
initGLTexture(Texture *texture)
{
  const auto &image = texture->image()->image();

  auto flippedImage = image->dup();

  flippedImage->flipH();

  auto *t1 = makeTexture(image);
  auto *t2 = makeTexture(flippedImage);

  t1->setName(texture->name());
  t2->setName(texture->name() + ".flip");

  texture->setGlTextures(this, t1, t2);
}

CQGLTexture *
Canvas::
makeTexture(const CImagePtr &image) const
{
  auto *texture = new CQGLTexture(image);

  texture->setFunctions(const_cast<Canvas *>(this));

  return texture;
}

}
