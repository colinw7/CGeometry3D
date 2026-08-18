#include <Canvas.h>
#include <App.h>
#include <Status.h>
#include <ShaderProgram.h>
#include <Camera.h>
#include <Texture.h>
#include <GeomObject.h>
#include <ParticleSystem.h>
#include <Font.h>
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

#ifdef CQ_PERF_GRAPH
#include <CQPerfMonitor.h>
#else
struct CQPerfTrace {
  CQPerfTrace(const char *) { }
};
#endif

#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>

#define USE_CLIP 1

namespace CQTclModel3DView {

Canvas::
Canvas(App *app) :
 app_(app)
{
  setObjectName("canvas");

  setFocusPolicy(Qt::StrongFocus);

  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  //---

  rubberBand_ = new CQRubberBand(this);

  //---

  connect(app_, SIGNAL(modelAdded()), this, SLOT(updateScene()));

  //---

  addViewport(CBBox2D(0, 0, 1, 1));

  updateStatus();
}

//---

Camera *
Canvas::
camera() const
{
  auto *viewportData = getCurrentViewportData();

  return viewportData->camera;
}

ShaderProgram *
Canvas::
sceneShaderProgram()
{
  auto *program = getShader("scene.vs", "scene.fs");

  if (! program->buffer())
    program->createBuffer();

  return program;
}

ShaderProgram *
Canvas::
selectionShaderProgram()
{
  auto *program = getShader("selection.vs", "selection.fs");

  if (! program->buffer())
    program->createBuffer();

  return program;
}

ShaderProgram *
Canvas::
particleShaderProgram()
{
  auto *program = getShader("particle.vs", "particle.fs");

  if (! program->buffer())
    program->createBuffer();

  return program;
}

//---

void
Canvas::
initializeGL()
{
  initializeOpenGLFunctions();

  //---

  font_ = new Font(this);

  font_->init();

  font_->setSize(48);
  font_->setFontName("OpenSans-Regular.ttf");
}

void
Canvas::
resizeGL(int width, int height)
{
  setPixelWidth (width);
  setPixelHeight(height);

  for (uint i = 0; i < viewportDatas_.size(); ++i) {
    currentViewport_ = i;

    auto region = viewportRegion();

    auto *camera = this->camera();

    camera->setAspect(double(region.getWidth())/double(region.getHeight()));
  }

  currentViewport_ = 0;
}

void
Canvas::
paintGL()
{
  CQPerfTrace trace("Canvas::paintGL");

  //---

  if (invalid_) {
    addScene();

    invalid_ = false;
  }

  //---

  currentViewport_ = 0;

  auto region = viewportRegion();

  glViewport(region.getXMin(), region.getYMin(), region.getWidth(), region.getHeight());

  const auto &bgColor = this->bgColor();

  glClearColor(bgColor.redF(), bgColor.greenF(), bgColor.blueF(), 1.0f);

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

  enableClips(true);

  drawScene();

  enableClips(false);

  //---

  for (uint i = 1; i < viewportDatas_.size(); ++i) {
    currentViewport_ = i;

    auto region = viewportRegion();

    glViewport(region.getXMin(), region.getYMin(), region.getWidth(), region.getHeight());

    //---

    glEnable(GL_SCISSOR_TEST);

    glScissor(region.getXMin(), region.getYMin(), region.getWidth(), region.getHeight());

    const auto &bgColor = this->bgColor();

    glClearColor(bgColor.redF(), bgColor.greenF(), bgColor.blueF(), 1.0f);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glDisable(GL_SCISSOR_TEST);

    //---

    enableClips(true);

    drawScene();

    enableClips(false);
  }

  //---

  currentViewport_ = 0;
}

void
Canvas::
enableClips(bool b)
{
#if USE_CLIP
  auto *viewportData = getCurrentViewportData();

  if (b) {
    for (uint ic = 0; ic < viewportData->clips.size(); ++ic)
      glEnable(GL_CLIP_DISTANCE0 + ic);
  }
  else {
    for (uint ic = 0; ic < viewportData->clips.size(); ++ic)
      glDisable(GL_CLIP_DISTANCE0 + ic);
  }
#endif
}

//---

void
Canvas::
updateScene(bool updateBBox)
{
  if (updateBBox) {
    for (auto *viewportData : viewportDatas_)
      viewportData->updateBBox = true;
  }

  invalid_ = true;

  update();
}

void
Canvas::
addScene()
{
  CQPerfTrace trace("Canvas::addScene");

  //---

  // TODO: per viewport ?
  auto *viewportData = getCurrentViewportData();

  auto bbox = this->bbox();

  if (viewportData->updateBBox)
    bbox = CBBox3D();

  //---

  auto *scene = app_->scene();

  for (auto *object : scene->getObjects()) {
    if (! object->refObject())
      addObject(object, bbox, viewportData->updateBBox);
  }

  //---

  addParticles(bbox, viewportData->updateBBox);

  //---

  updateTexts();

  //---

  if (! bbox.isSet()) {
    bbox.add(CPoint3D(-1, -1, -1));
    bbox.add(CPoint3D( 1,  1,  1));
  }

  //---

  if (viewportData->updateBBox)
    updateCamera();
}

void
Canvas::
addObject(CGeomObject3D *object, CBBox3D &bbox, bool updateBBox)
{
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

    faceData.orient = face->getProjectedOrientation();

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

  if (updateBBox) {
    bbox += bbox1;

    setBBox(bbox);
  }

  //---

  buffer->load();
}

//---

const CBBox3D &
Canvas::
bbox() const
{
  auto *viewportData = getCurrentViewportData();

  return viewportData->bbox;
}

void
Canvas::
setBBox(const CBBox3D &b)
{
  auto *viewportData = getCurrentViewportData();

  setBBox(viewportData, b);
}

void
Canvas::
setBBox(ViewportData *viewportData, const CBBox3D &b)
{
  viewportData->bbox = b;

  updateCamera(viewportData->camera);

  viewportData->updateBBox = false;
}

void
Canvas::
updateCamera()
{
  auto *camera = this->camera();

  updateCamera(camera);
}

void
Canvas::
updateCamera(Camera *camera)
{
  auto bbox = this->bbox();

  auto c = bbox.getCenter();
  auto d = bbox.getMaxSize();

  camera->setOrigin(CVector3D(c));
  camera->setDistance(std::sqrt(2.0)*d);
}

//---

void
Canvas::
addParticles(CBBox3D &bbox, bool updateBBox)
{
  ShaderParticles shaderParticles;
  getShaderParticles(shaderParticles);

  //---

  for (const auto &ps : shaderParticles) {
    auto *shaderData = ps.first;

    ShaderProgram *program = nullptr;
    bool           point   = false;

    if (shaderData) {
      program = getShaderDataProgram(shaderData);

      point = shaderData->point;
    }
    else
      program = particleShaderProgram();

    auto *buffer = program->buffer();

    if (! program)
      continue;

    //---

    buffer->clearBuffers();

    int pos = 0;

    auto addRectPoint = [&](const CPoint3D &p, const QColor &c,
                            const CVector3D &normal, const CPoint2D &pt) {
      buffer->addPoint(p.x, p.y, p.z);
      buffer->addNormal(normal.getX(), normal.getY(), normal.getZ());
      buffer->addColor(c.redF(), c.greenF(), c.blueF());
      buffer->addTexturePoint(pt.x, pt.y);
    };

    auto addPoint = [&](const CPoint3D &p, const CRGBA &c, FaceData &faceData) {
      auto color = RGBAToQColor(c);

      faceData.pos = pos;
      faceData.len = 1;

      buffer->addPoint(p.x, p.y, p.z);
      buffer->addColor(color.redF(), color.greenF(), color.blueF());
    };

    auto addRect = [&](const CPoint3D &p1, const CPoint3D &p2,
                       const CPoint3D &p3, const CPoint3D &p4,
                       const CRGBA &c, FaceData &faceData) {
      auto color = RGBAToQColor(c);

      faceData.pos = pos;
      faceData.len = 4;

      CVector3D diff1(p1, p2);
      CVector3D diff2(p2, p3);
      CVector3D diff3(p3, p4);
      CVector3D diff4(p4, p1);

      auto normal1 = diff4.crossProduct(diff1).normalized();
      auto normal2 = diff1.crossProduct(diff2).normalized();
      auto normal3 = diff2.crossProduct(diff3).normalized();
      auto normal4 = diff3.crossProduct(diff4).normalized();

      addRectPoint(p1, color, normal1, CPoint2D(0, 0));
      addRectPoint(p2, color, normal2, CPoint2D(1, 0));

      addRectPoint(p3, color, normal3, CPoint2D(1, 1));
      addRectPoint(p4, color, normal4, CPoint2D(0, 1));

      pos += 4;
    };

    //---

    ParticleData *particleData = nullptr;

    if (shaderData)
      particleData = &shaderData->particleData;
    else
      particleData = &particleData_;

    particleData->faceDatas.clear();

    //---

    const auto &particles = ps.second;

    for (auto *particle : particles) {
      auto *particle1 = dynamic_cast<Particle *>(particle);

      auto color = CRGBA::white();

      if (particle1)
        color = particle1->color();

      FaceData faceData;

      if (! point)
        addRect(CPoint3D(-0.5, -0.5, 0.0), CPoint3D( 0.5, -0.5, 0.0),
                CPoint3D( 0.5,  0.5, 0.0), CPoint3D(-0.5,  0.5, 0.0),
                color, faceData);
      else
        addPoint(CPoint3D(0.0, 0.0, 0.0), color, faceData);

      particleData->faceDatas.push_back(faceData);

      if (updateBBox) {
        auto *p = particle->position();

        bbox.add(CPoint3D(p->x(), p->y(), p->z()));
      }
    }

    //---

    buffer->load();
  }
}

void
Canvas::
drawScene()
{
  CQPerfTrace trace("Canvas::drawScene");

  //---

  auto *program = this->sceneShaderProgram();

  program->bind();

  //---

  auto *camera = this->camera();

  // camera projection
  auto worldMatrix = calcWorldMatrix();
  program->setUniformValue("projection", CQGLUtil::toQMatrix(worldMatrix));

  // camera/view transformation
  auto viewMatrix = camera->viewMatrix();
  program->setUniformValue("view", CQGLUtil::toQMatrix(viewMatrix));

  // view pos
  auto viewPos = camera->position();
  program->setUniformValue("viewPos", CQGLUtil::toVector(viewPos));

  //---

#if USE_CLIP
  auto *viewportData = getCurrentViewportData();

  program->setUniformValue("numClipPlanes", int(viewportData->clips.size()));

  int clip_i = 0;

  for (const auto &clip : viewportData->clips) {
    const auto &n = clip.getNormal();

    auto cv = QVector4D(n.getX(), n.getY(), n.getZ(), clip.getConstant());

    auto clipName = "clipPlane[" + std::to_string(clip_i) + "]";

    program->setUniformValue(clipName.c_str(), cv);

    ++clip_i;
  }
#endif

  //---

#if 0
  // add light data to shader program
  addShaderLights(program);
#else
  program->setUniformValue("ambientColor", CQGLUtil::toVector(ambientColor()));
  program->setUniformValue("ambientStrength", float(ambientStrength()));

  program->setUniformValue("diffuseStrength", float(diffuseStrength()));

  program->setUniformValue("specularColor"   , CQGLUtil::toVector(specularColor()));
  program->setUniformValue("specularStrength", float(specularStrength()));

  program->setUniformValue("emissionColor"   , CQGLUtil::toVector(emissiveColor()));
  program->setUniformValue("emissiveStrength", float(emissiveStrength()));

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
    if (object->parent())
      continue;

    drawObject(object);
  }

  //---

  program->release();

  //---

  drawTexts();

  //---

  drawSelection();

  //---

  drawParticles();
}

void
Canvas::
drawObject(CGeomObject3D *object)
{
  CQPerfTrace trace("Canvas::drawObject");

  //---

  if (! object->getVisible())
    return;

  auto *geomObject = dynamic_cast<GeomObject *>(object);
  assert(geomObject);

  auto *particle = geomObject->particle();

  if (particle && particle->isDead())
    return;

  //---

  auto *geomObject1 = geomObject;

  if (object->refObject()) {
    geomObject1 = dynamic_cast<GeomObject *>(object->refObject());
    assert(geomObject1);
  }

  //---

  auto *buffer = geomObject1->buffer();

  if (! buffer) {
    for (auto *child : object->children()) {
      drawObject(child);
    }

    return;
  }

  //---

  auto *program = this->sceneShaderProgram();

  //---

  auto *camera = this->camera();

  auto viewMatrix  = camera->viewMatrix();
  auto worldMatrix = calcWorldMatrix();

  // mesh matrix
  auto meshMatrix = object->getMeshGlobalTransform();
  program->setUniformValue("meshMatrix", CQGLUtil::toQMatrix(meshMatrix));

  //---

  // model matrix
  //auto modelMatrix = CMatrix3DH::identity();
  auto modelMatrix = geomObject->getHierTransform();

#if 0
  if (refObject && refObject != object)
    modelMatrix = refObject->getHierTransform()*modelMatrix;
#endif

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

  auto *objectMaterial = object->getMaterialP();

  //---

  std::vector<CQGLTexture *> boundTextures;
  boundTextures.resize(4);

  auto bindTexture = [&](int ind, CQGLTexture *texture) {
    if (texture != boundTextures[ind]) {
      glActiveTexture(GL_TEXTURE0 + ind);
      texture->bind();

      boundTextures[ind] = texture;
    }
  };

  auto drawFace = [&](const FaceData &faceData, double transparency) {
    bool faceSelected = (faceData.face ? faceData.face->getSelected() : false);

    bool selected = objectSelected || faceSelected;

    program->setUniformValue("isSelected", selected);

    //---

    // diffuse (texture 0)
    bool useDiffuseTexture = (isTextured() ? !!faceData.diffuseTexture : false);

    program->setUniformValue("diffuseTexture.enabled", useDiffuseTexture);

    if (useDiffuseTexture) {
      bindTexture(0, faceData.diffuseTexture);
      program->setUniformValue("diffuseTexture.texture", 0);
    }

    //---

    // normal (texture 1)
    bool useNormalTexture = (isTextured() ? !!faceData.normalTexture : false);

    program->setUniformValue("normalTexture.enabled", useNormalTexture);

    if (useNormalTexture) {
      bindTexture(1, faceData.normalTexture);
      program->setUniformValue("normalTexture.texture", 1);
    }

    //---

    // specular (texture 2)
    bool useSpecularTexture = (isTextured() ? !!faceData.specularTexture : false);

    program->setUniformValue("specularTexture.enabled", useSpecularTexture);

    if (useSpecularTexture) {
      bindTexture(2, faceData.specularTexture);
      program->setUniformValue("specularTexture.texture", 2);
    }

    //---

    // emissive (texture 3)
    bool useEmissiveTexture = (isTextured() ? !!faceData.emissiveTexture : false);

    program->setUniformValue("emissiveTexture.enabled", useEmissiveTexture);

    if (useEmissiveTexture) {
      bindTexture(3, faceData.emissiveTexture);
      program->setUniformValue("emissiveTexture.texture", 3);
    }

    program->setUniformValue("emissionColor", CQGLUtil::toVector(faceData.emission));

    //---

    program->setUniformValue("shininess", float(faceData.shininess));

    program->setUniformValue("transparency", float(1.0 - transparency));

    if (isShowOrient())
      program->setUniformValue("orientation",
        (faceData.orient == CPolygonOrientation::CLOCKWISE ? 1.0f : -1.0f));
    else
      program->setUniformValue("orientation", 0.0f);

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

    if (isStoreProjected()) {
      // set view and project point
      for (int i = 0; i < faceData.len; ++i) {
        CQGLBuffer::PointData data;
        buffer->getPointData(faceData.pos + i, data);

        auto p1 = viewMatrix*data.point->point();
        auto p2 = worldMatrix*p1;

        auto &vertex = object->getVertex(data.ind.value());

        vertex.setViewed   (p1);
        vertex.setProjected(p2);
      }
    }
  };

  //---

#if 0
  bool anyTransparent = false;
#endif

  buffer->bind();

  const auto &faceDatas = geomObject1->faceDatas();

  glDisable(GL_BLEND);
  glDepthMask(GL_TRUE);

  for (const auto &faceData : faceDatas) {
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

    for (const auto &faceData : faceDatas) {
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

  buffer->unbind();

  //---

  for (auto *child : object->children()) {
    drawObject(child);
  }
}

void
Canvas::
drawSelection()
{
  auto *program = selectionShaderProgram();

  //---

  if (! drawSelectionData_.buffer)
    drawSelectionData_.buffer = program->createBuffer();

  drawSelectionData_.buffer->clearBuffers();

  //---

  auto color = CRGBA::yellow();

  auto addLine = [&](const CPoint3D &p1, const CPoint3D &p2) {
    drawSelectionData_.buffer->addPoint(float(p1.x), float(p1.y), float(p1.z));
    drawSelectionData_.buffer->addColor(color);

    drawSelectionData_.buffer->addPoint(float(p2.x), float(p2.y), float(p2.z));
    drawSelectionData_.buffer->addColor(color);
  };

  auto addPoint = [&](const CPoint3D &p) {
    drawSelectionData_.buffer->addPoint(float(p.x), float(p.y), float(p.z));
    drawSelectionData_.buffer->addColor(color);
  };

  //---

  auto *scene = app_->scene();

  drawSelectionData_.lineIndex = 0;

  for (auto *object : scene->getObjects()) {
    auto *geomObject = dynamic_cast<GeomObject *>(object);
    assert(geomObject);

    auto modelMatrix = CMatrix3DH(geomObject->getHierTransform());

    const auto &edges = object->getEdges();

    for (auto *e : edges) {
      if (e->getSelected())
        addLine(modelMatrix*e->modelStart(), modelMatrix*e->modelEnd());
    }
  }

  drawSelectionData_.vertexIndex = drawSelectionData_.buffer->numPoints();

  for (auto *object : scene->getObjects()) {
    auto *geomObject = dynamic_cast<GeomObject *>(object);
    assert(geomObject);

    auto modelMatrix = CMatrix3DH(geomObject->getHierTransform());

    const auto &vertices = object->getVertices();

    for (auto *v : vertices) {
      if (v && v->getSelected())
        addPoint(modelMatrix*v->getModel());
    }
  }

  drawSelectionData_.endIndex = drawSelectionData_.buffer->numPoints();

  //---

  drawSelectionData_.buffer->load();

  //---

  drawSelectionData_.buffer->bind();

  program->bind();

  //---

  auto *camera = this->camera();

  // camera projection
  auto worldMatrix = calcWorldMatrix();
  program->setUniformValue("projection", CQGLUtil::toQMatrix(worldMatrix));

  // camera/view transformation
  auto viewMatrix = camera->viewMatrix();
  program->setUniformValue("view", CQGLUtil::toQMatrix(viewMatrix));

  // model matrix
  auto modelMatrix = CMatrix3DH::identity();
  program->setUniformValue("model", CQGLUtil::toQMatrix(modelMatrix));

  //---

  if (drawSelectionData_.vertexIndex > drawSelectionData_.lineIndex) {
    glLineWidth(8);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glDrawArrays(GL_LINES, drawSelectionData_.lineIndex,
                 drawSelectionData_.vertexIndex - drawSelectionData_.lineIndex);

    glLineWidth(1);
  }

  if (drawSelectionData_.endIndex > drawSelectionData_.vertexIndex) {
    glPointSize(8);

    glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);

    glDrawArrays(GL_POINTS, drawSelectionData_.vertexIndex,
                 drawSelectionData_.endIndex - drawSelectionData_.vertexIndex);

    glPointSize(1);
  }

  //---

  drawSelectionData_.buffer->unbind();

  //---

  program->release();
}

void
Canvas::
drawParticles()
{
  ShaderParticles shaderParticles;
  getShaderParticles(shaderParticles);

  //---

  for (const auto &ps : shaderParticles) {
    auto *shaderData = ps.first;

    ShaderProgram *program = nullptr;
    double         lineWidth = 0.0;

    if (shaderData) {
      program = getShaderDataProgram(shaderData);

      lineWidth = shaderData->lineWidth;
    }
    else
      program = particleShaderProgram();

    auto *buffer = program->buffer();

    if (! program)
      continue;

    //---

    program->bind();

    buffer->bind();

    //---

    glPointSize(pointSize());
    glLineWidth(lineWidth);

    glDepthFunc(GL_LEQUAL);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    //---

    auto *camera = this->camera();

    // camera projection
    auto worldMatrix = calcWorldMatrix();
    program->setUniformValue("projection", CQGLUtil::toQMatrix(worldMatrix));

    // camera/view transformation
    auto viewMatrix = camera->viewMatrix();
    program->setUniformValue("view", CQGLUtil::toQMatrix(viewMatrix));

    // model matrix
    auto modelMatrix = CMatrix3DH::identity();
    program->setUniformValue("model", CQGLUtil::toQMatrix(modelMatrix));

    //---

    CQGLTexture *texture = nullptr;

    program->setUniformValue("textureId", 0);

    const auto &particles = ps.second;

    uint i = 0;

    for (auto *particle : particles) {
      drawParticle(particle, i, program, texture, shaderData);

      ++i;
    }

    //---

    buffer->unbind();

    //---

    program->release();

    //---

    glDisable(GL_BLEND);
  }
}

void
Canvas::
drawParticle(CPSysParticle *particle, int i, ShaderProgram *program,
             CQGLTexture* &texture, ShaderData *shaderData)
{
  if (particle->isDead())
    return;

  auto *particle1 = dynamic_cast<Particle *>(particle);

  if (particle1 && particle1->texture()) {
    auto *texture1 = getGLTexture(const_cast<Texture *>(particle1->texture()), /*add*/true);

    if (texture1 != texture) {
      texture = texture1;

      glActiveTexture(GL_TEXTURE0);
      texture->bind();
    }

    program->setUniformValue("useTexture", true);
  }
  else {
    program->setUniformValue("useTexture", false);
  }

  auto *p = particle->position();

  CVector3D pos(p->x(), p->y(), p->z());

  auto angle = (particle1 ? particle1->angle() : 0.0);

  auto tpos  = CPoint2D(0, 0);
  auto tsize = CSize2D (1, 1);

  if (particle1) {
    tpos  = particle1->tpos();
    tsize = particle1->tsize();
  }

  auto size  = (particle1 ? particle1->size () : 1.0);
  auto alpha = (particle1 ? particle1->alpha() : 1.0);

  program->setUniformValue("position", vectorToQVector(pos));
  program->setUniformValue("size"    , float(size));
  program->setUniformValue("alpha"   , float(alpha));
  program->setUniformValue("tpos"    , QVector2D(tpos.x, tpos.y));
  program->setUniformValue("tsize"   , QVector2D(tsize.getWidth(), tsize.getHeight()));

  auto modelMatrix = CMatrix3DH::rotation(CMathGen::Z_AXIS_3D, angle);
  program->setUniformValue("model", CQGLUtil::toQMatrix(modelMatrix));

  //---

  ParticleData *particleData = nullptr;
  bool          point        = false;

  if (shaderData) {
    particleData = &shaderData->particleData;
    point        = shaderData->point;
  }
  else
    particleData = &particleData_;

  const auto &faceData = particleData->faceDatas[i];

  if (! point) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glDrawArrays(GL_TRIANGLE_FAN, faceData.pos, faceData.len);
  }
  else {
    glDrawArrays(GL_POINTS, faceData.pos, faceData.len);
  }
}

void
Canvas::
getShaderParticles(ShaderParticles &shaderParticles) const
{
  auto *psys = app_->particleSystem();

  auto np = psys->numberOfParticles();

  for (uint i = 0; i < np; ++i) {
    auto *particle = psys->getParticle(i);
    assert(particle);

    auto *particle1 = dynamic_cast<Particle *>(particle);

    ShaderData *shaderData { nullptr };

    if (particle1) {
      auto id = particle1->shader();

      if (id != "")
        shaderData = getShaderData(id);
    }

    shaderParticles[shaderData].push_back(particle);
  }
}

ShaderProgram *
Canvas::
getShaderDataProgram(ShaderData *shaderData)
{
  if (! shaderData->program) {
    shaderData->program = getShader(QString::fromStdString(shaderData->vs),
                                    QString::fromStdString(shaderData->gs),
                                    QString::fromStdString(shaderData->fs));

    if (! shaderData->program->buffer())
      shaderData->program->createBuffer();
  }

  return shaderData->program;
}

//---

CMatrix3DH
Canvas::
calcWorldMatrix() const
{
  auto *camera = this->camera();

  if (isPerspective())
    return camera->perspectiveMatrix();
  else
    return camera->orthoMatrix();
}

//---

void
Canvas::
clearTexts()
{
  for (auto *text : texts_)
    delete text;

  texts_.clear();
}

void
Canvas::
addText(Text *text)
{
  texts_.push_back(text);
}

Text *
Canvas::
getTextById(uint id) const
{
  for (auto *text : texts_) {
    if (text->id() == id)
      return text;
  }

  return nullptr;
}

void
Canvas::
updateTexts()
{
  for (auto *text : texts_)
    text->updateText();
}

void
Canvas::
drawTexts()
{
  glDisable(GL_CULL_FACE);

  for (auto *text : texts_) {
    if (! text->isVisible())
      continue;

    if (text->viewport() >= 0 && currentViewport() != uint(text->viewport()))
      continue;

    text->render(this);
  }

  glEnable(GL_CULL_FACE);
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

  //---

  if (mouseData_.button == Qt::LeftButton) {
    if (editType() == EditType::SELECT) {
      rubberBand_->setBounds(QPoint(mouseData_.press.x, mouseData_.press.y),
                             QPoint(mouseData_.move .x, mouseData_.move .y));
      rubberBand_->show();
    }
  }

  if (editType() == EditType::TCL) {
    std::vector<std::string> args;

    args.push_back(std::to_string(e->x()));
    args.push_back(std::to_string(e->y()));

    auto argList = app_->tcl()->mergeList(args);

    auto cmd = "mousePress " + argList;

    app_->runTclCmd(cmd);
  }
}

void
Canvas::
mouseMoveEvent(QMouseEvent *e)
{
  mouseData_.move = CPoint2D(e->x(), e->y());

  mouseData_.isShift   = (e->modifiers() & Qt::ShiftModifier);
  mouseData_.isControl = (e->modifiers() & Qt::ControlModifier);

  //---

  auto *camera = this->camera();

  auto dx = CMathUtil::sign(mouseData_.move.x - mouseData_.press.x);
  auto dy = CMathUtil::sign(mouseData_.move.y - mouseData_.press.y);

  if      (mouseData_.button == Qt::LeftButton) {
    if (editType() == EditType::SELECT) {
      rubberBand_->setBounds(QPoint(mouseData_.press.x, mouseData_.press.y),
                             QPoint(mouseData_.move .x, mouseData_.move .y));
    }
  }
  else if (mouseData_.button == Qt::MiddleButton) {
    auto da = M_PI/180.0;

    camera->rotateY(-dx*da);
    camera->rotateX(-dy*da);
  }
  else if (mouseData_.button == Qt::RightButton) {
    camera->moveRight(-dx/10.0);
    camera->moveUp   ( dy/10.0);
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

  //----

  auto *camera = this->camera();

  if (mouseData_.button == Qt::LeftButton) {
    auto dx = std::abs(mouseData_.press.x - mouseData_.move.x);
    auto dy = std::abs(mouseData_.press.y - mouseData_.move.y);

    if (editType() == EditType::SELECT) {
#if 0
      setStoreProjected(true);
      drawScene();
      setStoreProjected(false);
#endif

      drawData_.worldMatrix = calcWorldMatrix();
      drawData_.viewMatrix  = camera->viewMatrix();

      //---

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

      rubberBand_->hide();
    }
  }

  //---

  mouseData_.pressed = false;
  mouseData_.button  = Qt::NoButton;
}

void
Canvas::
wheelEvent(QWheelEvent *e)
{
  auto bbox = this->bbox();

  auto d  = bbox.getMaxSize()/100.0;
  auto dw = e->angleDelta().y()/250.0;

  auto *camera = this->camera();

  camera->setDistance(camera->distance() - dw*d);
}

void
Canvas::
keyPressEvent(QKeyEvent *e)
{
  auto keyStr = getKeyString(e);

  keyPressed_[keyStr] = true;

  mouseData_.isControl = (e->modifiers() & Qt::ControlModifier);
  mouseData_.isShift   = (e->modifiers() & Qt::ShiftModifier);

  auto k = e->key();

  //---

  auto bbox = this->bbox();

  auto d  = bbox.getMaxSize()/100.0;
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

  auto *camera = this->camera();

  if      (editType() == EditType::TCL) {
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
      camera->moveRight(-d);
    }
    else if (k == Qt::Key_Right) {
      camera->moveRight(d);
    }
    else if (k == Qt::Key_Up) {
      camera->moveUp(d);
    }
    else if (k == Qt::Key_Down) {
      camera->moveUp(-d);
    }
    else if (k == Qt::Key_Plus) {
      camera->moveFront(d);
    }
    else if (k == Qt::Key_Minus) {
      camera->moveFront(-d);
    }
    else if (k == Qt::Key_W) {
      camera->rotateX(da);
    }
    else if (k == Qt::Key_S) {
      camera->rotateX(-da);
    }
    else if (k == Qt::Key_A) {
      camera->rotateY(da);
    }
    else if (k == Qt::Key_D) {
      camera->rotateY(-da);
    }
    else if (k == Qt::Key_Q) {
      camera->rotateZ(-da);
    }
    else if (k == Qt::Key_E) {
      camera->rotateZ(da);
    }
    else if (k == Qt::Key_Space) {
      camera->printMatrices();
    }
  }
}

void
Canvas::
keyReleaseEvent(QKeyEvent *e)
{
  auto keyStr = getKeyString(e);

  keyPressed_[keyStr] = false;
}

bool
Canvas::
getKeyPressed(const std::string &key) const
{
  auto p = keyPressed_.find(key);

  if (p == keyPressed_.end())
    return false;

  return (*p).second;
}

std::string
Canvas::
getKeyString(QKeyEvent *e) const
{
  std::string keyStr;

  if      (e->key() == Qt::Key_Left ) keyStr = "left";
  else if (e->key() == Qt::Key_Right) keyStr = "right";
  else if (e->key() == Qt::Key_Up   ) keyStr = "up";
  else if (e->key() == Qt::Key_Down ) keyStr = "down";
  else if (e->key() == Qt::Key_Space) keyStr = "space";
  else                                keyStr = e->text().toStdString();

  if (keyStr == "")
    keyStr = "key." + std::to_string(e->key());

  return keyStr;
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

  for (auto *object : getDrawObjects()) {
    auto *geomObject = dynamic_cast<GeomObject *>(object);
    assert(geomObject);

    //---

    drawData_.modelMatrix = CMatrix3DH(geomObject->getHierTransform());

    //---

    const auto &faces = object->getFaces();

    for (auto *face : faces) {
      if (! face->getVisible())
        continue;

      projectFaceVertices(face);

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

  for (auto *object : getDrawObjects()) {
    auto *geomObject = dynamic_cast<GeomObject *>(object);
    assert(geomObject);

    //---

    drawData_.modelMatrix = CMatrix3DH(geomObject->getHierTransform());

    //---

    const auto &faces = object->getFaces();

    for (auto *face : faces) {
      if (! face->getVisible())
        continue;

      projectFaceVertices(face);

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

  for (auto *object : getDrawObjects()) {
    auto *geomObject = dynamic_cast<GeomObject *>(object);
    assert(geomObject);

    //---

    drawData_.modelMatrix = CMatrix3DH(geomObject->getHierTransform());

    //---

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

  for (auto *object : getDrawObjects()) {
    auto *geomObject = dynamic_cast<GeomObject *>(object);
    assert(geomObject);

    //---

    drawData_.modelMatrix = CMatrix3DH(geomObject->getHierTransform());

    //---

    const auto &vertices = object->getVertices();

    for (auto *vertex : vertices) {
      if (! vertex || ! vertex->getVisible())
        continue;

      projectVertex(vertex);

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

  for (auto *object : getDrawObjects()) {
    auto *geomObject = dynamic_cast<GeomObject *>(object);
    assert(geomObject);

    //---

    drawData_.modelMatrix = CMatrix3DH(geomObject->getHierTransform());

    //---

    const auto &vertices = object->getVertices();

    for (auto *vertex : vertices) {
      if (! vertex || ! vertex->getVisible())
        continue;

      projectVertex(vertex);

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

std::vector<CGeomObject3D *>
Canvas::
getDrawObjects() const
{
  std::vector<CGeomObject3D *> objects;

  auto *scene = app_->scene();

  for (auto *object : scene->getObjects()) {
    if (object->parent())
      continue;

    addDrawObjects(object, objects);
  }

  return objects;
}

void
Canvas::
addDrawObjects(CGeomObject3D *object, std::vector<CGeomObject3D *> &objects) const
{
  if (! object->getVisible())
    return;

  objects.push_back(object);

  for (auto *child : object->children()) {
    addDrawObjects(child, objects);
  }
}

void
Canvas::
projectFaceVertices(CGeomFace3D *face)
{
  auto *object = face->getObject();

  for (const auto &vind : face->getVertices()) {
    auto *vertex = object->getVertexP(vind);

    projectVertex(vertex);
  }
}

void
Canvas::
projectVertex(CGeomVertex3D *vertex)
{
  auto p1 = drawData_.viewMatrix*drawData_.modelMatrix*vertex->getModel();
  auto p2 = drawData_.worldMatrix*p1;

  vertex->setViewed   (p1);
  vertex->setProjected(p2);
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

const QColor &
Canvas::
bgColor() const
{
  auto *viewportData = getCurrentViewportData();

  return viewportData->bgColor;
}

void
Canvas::
setBgColor(const QColor &c)
{
  auto *viewportData = getCurrentViewportData();

  viewportData->bgColor = c;
}

CBBox2D
Canvas::
viewportRegion() const
{
  auto *viewportData = getCurrentViewportData();

  auto pw = pixelWidth ();
  auto ph = pixelHeight();

  auto x1 = viewportData->rect.getXMin()*pw;
  auto y1 = viewportData->rect.getYMin()*ph;
  auto x2 = viewportData->rect.getXMax()*pw;
  auto y2 = viewportData->rect.getYMax()*ph;

  return CBBox2D(x1, y1, x2, y2);
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

std::string
Canvas::
addViewport(const CBBox2D &rect)
{
  auto *viewportData = new ViewportData;

  viewportData->id   = "viewport" + std::to_string(viewportDatas_.size() + 1);
  viewportData->rect = rect;

  viewportData->camera = new Camera(app_);

  viewportData->camera->setOrigin(CVector3D(0, 0, 0));
  viewportData->camera->setDistance(5);

  connect(viewportData->camera, SIGNAL(stateChangedSignal()), this, SLOT(update()));

  viewportDatas_.push_back(viewportData);

  //---

  currentViewport_ = viewportDatas_.size() - 1;

  auto region = viewportRegion();

  viewportData->camera->setAspect(double(region.getWidth())/double(region.getHeight()));

  currentViewport_ = 0;

  return viewportData->id;
}

Canvas::ViewportData *
Canvas::
getCurrentViewportData() const
{
  return viewportDatas_[currentViewport_];
}

Canvas::ViewportData *
Canvas::
getViewportData(const std::string &id) const
{
  if (id == "")
    return viewportDatas_[0];

  for (auto *viewportData : viewportDatas_)
    if (viewportData->id == id)
      return viewportData;

  return nullptr;
}

//---

std::string
Canvas::
addShaderData(const std::string &vs, const std::string &fs)
{
  return addShaderData(vs, "", fs);
}

std::string
Canvas::
addShaderData(const std::string &vs, const std::string &gs, const std::string &fs)
{
  auto id = "Shader." + std::to_string(shaderDatas_.size() + 1);

  auto *data = new ShaderData;

  data->id = id;
  data->vs = vs;
  data->gs = gs;
  data->fs = fs;

  shaderDatas_.push_back(data);

  return id;
}

Canvas::ShaderData *
Canvas::
getShaderData(const std::string &id) const
{
  for (auto *shaderData : shaderDatas_)
    if (shaderData->id == id)
      return shaderData;

  return nullptr;
}

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

ShaderProgram *
Canvas::
getShader(const QString &vertex, const QString &geometry, const QString &fragment)
{
  auto id = QString("V:%1,G:%2,F:%3").arg(vertex).arg(geometry).arg(fragment);

  auto ps = shaders_.find(id);

  if (ps == shaders_.end()) {
    auto *shaderProgram = new ShaderProgram(app_);

    shaderProgram->addShaders(vertex, fragment);

    if (geometry != "")
      shaderProgram->addGeometryShader(geometry);

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
