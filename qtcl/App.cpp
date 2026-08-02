#include <App.h>
#include <Canvas.h>
#include <Overview.h>
#include <Toolbar.h>
#include <Sidebar.h>
#include <Control.h>
#include <Status.h>
#include <GeomObject.h>
#include <Texture.h>
#include <ParticleSystem.h>
#include <Camera.h>
#include <Font.h>
#include <Util.h>

#include <CImportObj.h>

#include <CQTabSplit.h>
#include <CGeometry3D.h>
#include <CGeomScene3D.h>
#include <CGeomEdge3D.h>

#include <CGeomCube3D.h>
#include <CGeomCone3D.h>
#include <CGeomCylinder3D.h>
#include <CGeomSphere3D.h>
#include <CGeomPlane3D.h>

#include <CTclUtil.h>
#include <CTclObj.h>

#include <CRGBName.h>

#include <CSolidNoise.h>

#include <CQMetaEdit.h>
#include <CQAppOptions.h>

#ifdef CQ_PERF_GRAPH
#include <CQPerfGraph.h>
#endif

#include <QVBoxLayout>
#include <QTimer>

//---

#include <svg/pause_svg.h>

#include <svg/face_select_svg.h>
#include <svg/edge_select_svg.h>
#include <svg/point_select_svg.h>

#include <svg/depth3d_svg.h>
#include <svg/cull3d_svg.h>
#include <svg/front3d_svg.h>

#include <svg/wireframe_svg.h>
#include <svg/solid_fill_svg.h>
#include <svg/texture_fill_svg.h>

#include <svg/menu_svg.h>

#include <svg/select_svg.h>
#include <svg/camera_svg.h>
#include <svg/tcl_svg.h>

#if 0
#include <svg/cursor_svg.h>
#include <svg/deselected_svg.h>
#include <svg/extrude_svg.h>
#include <svg/invisible_svg.h>
#include <svg/light_svg.h>
#include <svg/loop_cut_svg.h>
#include <svg/model_svg.h>
#include <svg/move_svg.h>
#include <svg/object_select_svg.h>
#include <svg/play_one_svg.h>
#include <svg/play_svg.h>
#include <svg/rotate_svg.h>
#include <svg/scale_svg.h>
#include <svg/selected_svg.h>
#include <svg/settings_svg.h>
#include <svg/view_follow_svg.h>
#include <svg/visible_svg.h>
#endif

//---

#define Q(x) #x
#define QUOTE(x) Q(x)

//---

namespace CQTclModel3DView {

bool stringToInteger(const std::string &str, int &i) {
  try {
    i = std::stoi(str);
    return true;
  }
  catch (...) {
    return false;
  }
}

bool stringToReal(const std::string &str, double &r) {
  try {
    r = std::stod(str);
    return true;
  }
  catch (...) {
    return false;
  }
}

bool objToReal(const Tcl_Obj *obj, double &r) {
  return stringToReal(CTclUtil::stringFromObj(obj), r);
}

std::string toLower(const std::string &str) {
  std::string str1;
  for (auto &c : str)
    str1 += (isupper(c) ? tolower(c) : c);
  return str1;
}

bool stringToBool(const std::string &str, bool &b) {
  auto str1 = toLower(str);

  bool rc = true;
  if      (str1 == "yes" || str1 == "true"  || str1 == "1")
    b = true;
  else if (str1 == "no"  || str1 == "false" || str1 == "0")
    b = false;
  else
    rc = b = false;

  return rc;
}

//---

std::string encodeParticleId(uint ind) {
  return "p:" + std::to_string(ind);
}

bool decodeParticleId(const std::string &id, int &objId) {
  if (id == "null") { objId = -1; return true; }

  if (id.size() < 3 || id.substr(0, 2) != "p:")
    return false;

  if (! stringToInteger(id.substr(2), objId))
    return false;

  return true;
}

std::string encodeParticle(CPSysParticle *p) {
  if (! p) return "null";
  return encodeParticleId(p->ind());
}

std::string encodeSpringId(uint ind) {
  return "s:" + std::to_string(ind);
}

bool decodeSpringId(const std::string &id, int &objId) {
  if (id == "null") { objId = -1; return true; }

  if (id.size() < 3 || id.substr(0, 2) != "s:")
    return false;

  if (! stringToInteger(id.substr(2), objId))
    return false;

  return true;
}

std::string encodeSpring(CPSysSpring *p) {
  if (! p) return "null";
  return encodeSpringId(p->ind());
}

std::string encodeAttractionId(uint ind) {
  return "a:" + std::to_string(ind);
}

bool decodeAttractionId(const std::string &id, int &objId) {
  if (id == "null") { objId = -1; return true; }

  if (id.size() < 3 || id.substr(0, 2) != "a:")
    return false;

  if (! stringToInteger(id.substr(2), objId))
    return false;

  return true;
}

std::string encodeAttraction(CPSysAttraction *a) {
  if (! a) return "null";
  return encodeAttractionId(a->ind());
}

//---

std::string encodeObjectId(uint objId) {
  return "o:" + std::to_string(objId);
}

bool decodeObjectId(const std::string &id, int &objId) {
  if (id == "null") { objId = -1; return true; }

  if (id.size() < 3 || id.substr(0, 2) != "o:")
    return false;

  if (! stringToInteger(id.substr(2), objId))
    return false;

  return true;
}

std::string encodeObject(CGeomObject3D *object) {
  if (! object) return "null";
  return encodeObjectId(object->getInd());
}

std::string encodeMaterialId(uint materialId) {
  return "m:" + std::to_string(materialId);
}

std::string encodeMaterial(const CGeomMaterial *material) {
  return encodeMaterialId(material->id());
}

bool decodeMaterialId(const std::string &id, int &materialId) {
  if (id == "null") { materialId = -1; return true; }

  if (id.size() < 3 || id.substr(0, 2) != "m:")
    return false;

  if (! stringToInteger(id.substr(2), materialId))
    return false;

  return true;
}

std::string encodeTextureId(uint textureId) {
  return "t:" + std::to_string(textureId);
}

bool decodeTextureId(const std::string &id, int &textureId) {
  if (id == "null") { textureId = -1; return true; }

  if (id.size() < 3 || id.substr(0, 2) != "t:")
    return false;

  if (! stringToInteger(id.substr(2), textureId))
    return false;

  return true;
}

std::string encodeTexture(const CGeomTexture *texture) {
  return encodeTextureId(texture->id());
}

std::string encodeObjectVertexId(uint objId, uint vertexId) {
  return "v:" + std::to_string(objId) + ":" + std::to_string(vertexId);
}

std::string encodeObjectVertexId(const CGeomObject3D *object, int vertexId) {
  return encodeObjectVertexId(object->getInd(), vertexId);
}

std::string encodeVertex(const CGeomVertex3D *vertex) {
  return encodeObjectVertexId(vertex->getObject()->getInd(), vertex->getInd());
}

bool decodeObjectVertexId(const std::string &id, int &objId, int &vertexId) {
  if (id == "null") { objId = -1; return true; }

  if (id.size() < 3 || id.substr(0, 2) != "v:")
    return false;

  uint i1 = 2;
  uint i2 = i1;

  while (id[i2] && id[i2] != ':')
    ++i2;

  if (! stringToInteger(id.substr(i1, i2 - i1), objId))
    return false;

  ++i2;

  if (! stringToInteger(id.substr(i2), vertexId))
    return false;

  return true;
}

std::string encodeObjectFaceId(uint objId, uint faceId) {
  return "f:" + std::to_string(objId) + ":" + std::to_string(faceId);
}

std::string encodeFace(CGeomFace3D *face) {
  return encodeObjectFaceId(face->getObject()->getInd(), face->getInd());
}

bool decodeObjectFaceId(const std::string &id, int &objId, int &faceId) {
  if (id == "null") { objId = -1; return true; }

  if (id.size() < 3 || id.substr(0, 2) != "f:")
    return false;

  uint i1 = 2;
  uint i2 = i1;

  while (id[i2] && id[i2] != ':')
    ++i2;

  if (! stringToInteger(id.substr(i1, i2 - i1), objId))
    return false;

  ++i2;

  if (! stringToInteger(id.substr(i2), faceId))
    return false;

  return true;
}

std::string encodeObjectEdgeId(uint objId, uint edgeId) {
  return "e:" + std::to_string(objId) + ":" + std::to_string(edgeId);
}

std::string encodeEdge(CGeomEdge3D *edge) {
  return encodeObjectEdgeId(edge->getObject()->getInd(), edge->getInd());
}

bool decodeObjectEdgeId(const std::string &id, int &objId, int &edgeId) {
  if (id == "null") { objId = -1; return true; }

  if (id.size() < 3 || id.substr(0, 2) != "e:")
    return false;

  uint i1 = 2;
  uint i2 = i1;

  while (id[i2] && id[i2] != ':')
    ++i2;

  if (! stringToInteger(id.substr(i1, i2 - i1), objId))
    return false;

  ++i2;

  if (! stringToInteger(id.substr(i2), edgeId))
    return false;

  return true;
}

std::string encodeObjectLineId(uint objId, uint lineId) {
  return "l:" + std::to_string(objId) + ":" + std::to_string(lineId);
}

bool decodeObjectLineId(const std::string &id, int &objId, int &lineId) {
  if (id == "null") { objId = -1; return true; }

  if (id.size() < 3 || id.substr(0, 2) != "l:")
    return false;

  uint i1 = 2;
  uint i2 = i1;

  while (id[i2] && id[i2] != ':')
    ++i2;

  if (! stringToInteger(id.substr(i1, i2 - i1), objId))
    return false;

  ++i2;

  if (! stringToInteger(id.substr(i2), lineId))
    return false;

  return true;
}

std::string encodeTextId(uint textId) {
  return "T:" + std::to_string(textId);
}

bool decodeTextId(const std::string &id, int &textId) {
  if (id == "null") { textId = -1; return true; }

  if (id.size() < 3 || id.substr(0, 2) != "T:")
    return false;

  if (! stringToInteger(id.substr(2), textId))
    return false;

  return true;
}

std::string encodeText(const Text *text) {
  return encodeTextId(text->id());
}

int tclErrorMsg(const std::string &msg) {
  std::cerr << msg << "\n";
  return TCL_ERROR;
}

bool errorMsg(const std::string &msg) {
  std::cerr << msg << "\n";
  return false;
}

Tcl_Obj *vectorToObj(const CVector3D &v) {
  return CTclVector3D::newObj(v);
}

Tcl_Obj *matrixToObj(const CMatrix3D &m) {
  return CTclMatrix3D::newObj(m);
}

CTcl::RealList pointToRealArray(const CPoint3D &p) {
  CTcl::RealList realList;

  realList.push_back(p.x);
  realList.push_back(p.y);
  realList.push_back(p.z);

  return realList;
}

CTcl::RealList colorToRealArray(const CRGBA &c) {
  CTcl::RealList realList;

  realList.push_back(c.getRed  ());
  realList.push_back(c.getGreen());
  realList.push_back(c.getBlue ());

  return realList;
}

std::vector<CTcl::RealList> bboxToRealArrays(const CBBox3D &bbox) {
  std::vector<CTcl::RealList> realListArray;

  realListArray.push_back(pointToRealArray(bbox.getMin()));
  realListArray.push_back(pointToRealArray(bbox.getMax()));

  return realListArray;
}

double degToRad(double d) {
  return M_PI*d/180;
}

}

//---

namespace CQTclModel3DView {

class GeomFactory : public CGeometryFactory {
 public:
  GeomFactory() { }
 ~GeomFactory() override { }

  CGeomObject3D *createObject3D(CGeomScene3D *pscene, const std::string &name) const override {
    return new GeomObject(pscene, name);
  }

#if 0
  CGeomFace3D *createFace3D() const override {
    return new GeomFace;
  }

  CGeomLine3D *createLine3D() const override {
    return new GeomLine;
  }

  CGeomLight3D *createLight3D(CGeomScene3D *pscene, const std::string &name) const override {
    return new Light(pscene, name);
  }
#endif

  CGeomTexture *createTexture() const override {
    return new Texture;
  }
};

}

//---

namespace CQTclModel3DView {

CTCL_DCL_OBJECT_PROC(App, getAppValue, getAppValueProc, this)
CTCL_DCL_OBJECT_PROC(App, setAppValue, setAppValueProc, this)

CTCL_DCL_OBJECT_PROC (App, addShader     , addShaderProc     , this)
CTCL_DCL_OBJECT_PROC (App, setShaderValue, setShaderValueProc, this)

CTCL_DCL_OBJECT_PROC (App, addViewport     , addViewportProc     , this)
CTCL_DCL_OBJECT_PROC (App, setViewportValue, setViewportValueProc, this)

CTCL_DCL_OBJECT_PROC (App, addParticle       , addParticleProc       , this)
CTCL_DCL_OBJECT_PROC (App, getParticleValue  , getParticleValueProc  , this)
CTCL_DCL_TCL_OBJ_PROC(App, setParticleValue  , setParticleValueProc  , this)
CTCL_DCL_OBJECT_PROC (App, addSpring         , addSpringProc         , this)
CTCL_DCL_OBJECT_PROC (App, getSpringValue    , getSpringValueProc    , this)
CTCL_DCL_OBJECT_PROC (App, setSpringValue    , setSpringValueProc    , this)
CTCL_DCL_OBJECT_PROC (App, addAttraction     , addAttractionProc     , this)
CTCL_DCL_OBJECT_PROC (App, getAttractionValue, getAttractionValueProc, this)
CTCL_DCL_OBJECT_PROC (App, setAttractionValue, setAttractionValueProc, this)

CTCL_DCL_OBJECT_PROC(App, addObject  , addObjectProc  , this)
CTCL_DCL_OBJECT_PROC(App, addVertex  , addVertexProc  , this)
CTCL_DCL_OBJECT_PROC(App, addFace    , addFaceProc    , this)
CTCL_DCL_OBJECT_PROC(App, addLine    , addLineProc    , this)
CTCL_DCL_OBJECT_PROC(App, addMaterial, addMaterialProc, this)
CTCL_DCL_OBJECT_PROC(App, addTexture , addTextureProc , this)
CTCL_DCL_OBJECT_PROC(App, addText    , addTextProc    , this)

CTCL_DCL_OBJECT_PROC(App, addPlane   , addPlaneProc   , this)
CTCL_DCL_OBJECT_PROC(App, addCube    , addCubeProc    , this)
CTCL_DCL_OBJECT_PROC(App, addCone    , addConeProc    , this)
CTCL_DCL_OBJECT_PROC(App, addCylinder, addCylinderProc, this)
CTCL_DCL_OBJECT_PROC(App, addSphere  , addSphereProc  , this)
CTCL_DCL_OBJECT_PROC(App, addTerrain , addTerrainProc , this)

CTCL_DCL_OBJECT_PROC(App, getObjectValue   , getObjectValueProc   , this)
CTCL_DCL_OBJECT_PROC(App, setObjectValue   , setObjectValueProc   , this)
CTCL_DCL_OBJECT_PROC(App, getFaceValue     , getFaceValueProc     , this)
CTCL_DCL_OBJECT_PROC(App, setFaceValue     , setFaceValueProc     , this)
CTCL_DCL_OBJECT_PROC(App, getLineValue     , getLineValueProc     , this)
CTCL_DCL_OBJECT_PROC(App, setLineValue     , setLineValueProc     , this)
CTCL_DCL_OBJECT_PROC(App, getEdgeValue     , getEdgeValueProc     , this)
CTCL_DCL_OBJECT_PROC(App, setEdgeValue     , setEdgeValueProc     , this)
CTCL_DCL_OBJECT_PROC(App, getVertexValue   , getVertexValueProc   , this)
CTCL_DCL_OBJECT_PROC(App, setVertexValue   , setVertexValueProc   , this)
CTCL_DCL_OBJECT_PROC(App, setMaterialValue , setMaterialValueProc , this)
CTCL_DCL_OBJECT_PROC(App, getTextValue     , getTextValueProc     , this)
CTCL_DCL_OBJECT_PROC(App, setTextValue     , setTextValueProc     , this)
CTCL_DCL_OBJECT_PROC(App, getObjectProperty, getObjectPropertyProc, this)
CTCL_DCL_OBJECT_PROC(App, setObjectProperty, setObjectPropertyProc, this)

CTCL_DCL_OBJECT_PROC(App, intersectObjects, intersectObjectsProc, this)
CTCL_DCL_OBJECT_PROC(App, inverseObject   , inverseObjectProc   , this)
CTCL_DCL_OBJECT_PROC(App, unionObjects    , unionObjectsProc    , this)
CTCL_DCL_OBJECT_PROC(App, subtractObjects , subtractObjectsProc , this)

CTCL_DCL_OBJECT_PROC(App, extrudeFaces, extrudeFacesProc, this)
CTCL_DCL_OBJECT_PROC(App, extrudeEdges, extrudeEdgesProc, this)
CTCL_DCL_OBJECT_PROC(App, mergeEdge   , mergeEdgeProc   , this)
CTCL_DCL_OBJECT_PROC(App, separateFace, separateFaceProc, this)
CTCL_DCL_OBJECT_PROC(App, separateEdge, separateEdgeProc, this)
CTCL_DCL_OBJECT_PROC(App, mirrorObject, mirrorObjectProc, this)
CTCL_DCL_OBJECT_PROC(App, fillVertices, fillVerticesProc, this)

CTCL_DCL_OBJECT_PROC(App, deleteObjects , deleteObjectsProc , this)
CTCL_DCL_OBJECT_PROC(App, deleteFaces   , deleteFacesProc   , this)
CTCL_DCL_OBJECT_PROC(App, deleteVertices, deleteVerticesProc, this)

CTCL_DCL_OBJECT_PROC(App, animateReal, animateRealProc, this)

CTCL_DCL_OBJECT_PROC(App, readModel, readModelProc, this)
CTCL_DCL_OBJECT_PROC(App, writeObj , writeObjProc , this)

CTCL_DCL_OBJECT_PROC (App, vector     , vectorProc    , this)
CTCL_DCL_TCL_OBJ_PROC(App, get_vector , getVectorProc , this)
CTCL_DCL_TCL_OBJ_PROC(App, set_vector , setVectorProc , this)
CTCL_DCL_OBJECT_PROC (App, calc_vector, calcVectorProc, this)

CTCL_DCL_OBJECT_PROC (App, matrix     , matrixProc    , this)
CTCL_DCL_TCL_OBJ_PROC(App, get_matrix , getMatrixProc , this)
CTCL_DCL_TCL_OBJ_PROC(App, set_matrix , setMatrixProc , this)
CTCL_DCL_OBJECT_PROC (App, calc_matrix, calcMatrixProc, this)

App::
App()
{
  setObjectName("app");

  buildDir_ = QUOTE(BUILD_DIR);

  //---

  CGeometry3DInst->setFactory(new GeomFactory);

  scene_ = CGeometry3DInst->createScene3D();

  //---

  // create widgets first (in correct order for initialization)
  status_   = new Status  (this);
  canvas_   = new Canvas  (this);
  overview_ = new Overview(this);
  toolbar_  = new Toolbar (this);
  sidebar_  = new Sidebar (this);
  control_  = new Control (this);

  //---

  auto *layout = new QVBoxLayout(this);

  layout->addWidget(toolbar_);

  auto *clayout = new QHBoxLayout;

  layout->addLayout(clayout);

  clayout->addWidget(sidebar_);

  auto *tab = new CQTabSplit;

  tab->setState(CQTabSplit::State::TAB);

  tab->addWidget(canvas_  , "3D View" );
  tab->addWidget(overview_, "Overview");

  clayout->addWidget(tab);

  clayout->addWidget(control_);

  layout->addWidget(status_);

  //---

  psys_ = new ParticleSystem;

  //---

  timer_ = new QTimer;

  connect(timer_, SIGNAL(timeout()), this, SLOT(timerSlot()));

  //---

  initTcl();

  //---

  canvas_->setFocus();

  timer_->start(100);
}

bool
App::
loadModel(const std::string &fileName, CGeom3DType format, LoadData &loadData)
{
  loadData.topObj = nullptr;

  static uint modeInd;

  auto modelName = QString("Model.%1").arg(++modeInd);

  auto *im = CImportBase::createModel(format, modelName.toStdString());

  if (! im) {
    //std::cerr << "File format not recognised for '" << fileName << "'\n";
    return false;
  }

  CFile file(fileName);

  if (! im->read(file)) {
    delete im;
    //std::cerr << "Failed to read model for '" << fileName << "'\n";
    return false;
  }

  auto *scene = im->releaseScene();

  delete im;

  uint numTop = 0;

  for (auto *object : scene->getObjects()) {
    if (! object->parent()) {
      ++numTop;
      loadData.topObj = object;
    }
  }

  if (numTop > 1) {
    auto name = "object." + std::to_string(scene_->getObjects().size() + 1);

    auto *parentObj = CGeometry3DInst->createObject3D(scene_, name);

    scene_->addObject(parentObj);

    for (auto *object : scene->getObjects()) {
      scene_->addObject(object);

      if (! object->parent())
        parentObj->addChild(object);

      object->setInd(CGeometry3DInst->nextObjectId());
    }

    loadData.topObj = parentObj;
  }
  else {
    for (auto *object : scene->getObjects()) {
      scene_->addObject(object);

      object->setInd(CGeometry3DInst->nextObjectId());
    }
  }

  for (auto *material : scene->getMaterials()) {
    scene_->addMaterial(material);
  }

  for (auto *texture : scene->textures()) {
    scene_->addTexture(texture);
  }

  return true;
}

//---

void
App::
initTcl()
{
  tcl_ = new CTcl;

  CTclVector3D::init(tcl_->interp());

  tcl_->createAlias("echo", "puts");

  //---

  CTCL_OBJECT_PROC(tcl_, getAppValue, App, this);
  CTCL_OBJECT_PROC(tcl_, setAppValue, App, this);

  CTCL_OBJECT_PROC(tcl_, addShader     , App, this);
  CTCL_OBJECT_PROC(tcl_, setShaderValue, App, this);

  CTCL_OBJECT_PROC(tcl_, addViewport     , App, this);
  CTCL_OBJECT_PROC(tcl_, setViewportValue, App, this);

  //---

  // particles
  CTCL_OBJECT_PROC (tcl_, addParticle       , App, this);
  CTCL_OBJECT_PROC (tcl_, getParticleValue  , App, this);
  CTCL_TCL_OBJ_PROC(tcl_, setParticleValue  , App, this);
  CTCL_OBJECT_PROC (tcl_, addSpring         , App, this);
  CTCL_OBJECT_PROC (tcl_, getSpringValue    , App, this);
  CTCL_OBJECT_PROC (tcl_, setSpringValue    , App, this);
  CTCL_OBJECT_PROC (tcl_, addAttraction     , App, this);
  CTCL_OBJECT_PROC (tcl_, getAttractionValue, App, this);
  CTCL_OBJECT_PROC (tcl_, setAttractionValue, App, this);

  //---

  // add primitives
  CTCL_OBJECT_PROC(tcl_, addObject  , App, this);
  CTCL_OBJECT_PROC(tcl_, addVertex  , App, this);
  CTCL_OBJECT_PROC(tcl_, addFace    , App, this);
  CTCL_OBJECT_PROC(tcl_, addLine    , App, this);
  CTCL_OBJECT_PROC(tcl_, addMaterial, App, this);
  CTCL_OBJECT_PROC(tcl_, addTexture , App, this);
  CTCL_OBJECT_PROC(tcl_, addText    , App, this);

  // add shapes
  CTCL_OBJECT_PROC(tcl_, addPlane   , App, this)
  CTCL_OBJECT_PROC(tcl_, addCube    , App, this)
  CTCL_OBJECT_PROC(tcl_, addCone    , App, this)
  CTCL_OBJECT_PROC(tcl_, addCylinder, App, this)
  CTCL_OBJECT_PROC(tcl_, addSphere  , App, this)
  CTCL_OBJECT_PROC(tcl_, addTerrain , App, this)

  // get/set primitive data
  CTCL_OBJECT_PROC(tcl_, getObjectValue, App, this);
  CTCL_OBJECT_PROC(tcl_, setObjectValue, App, this);
  CTCL_OBJECT_PROC(tcl_, getFaceValue  , App, this);
  CTCL_OBJECT_PROC(tcl_, setFaceValue  , App, this);
  CTCL_OBJECT_PROC(tcl_, getLineValue  , App, this);
  CTCL_OBJECT_PROC(tcl_, setLineValue  , App, this);
  CTCL_OBJECT_PROC(tcl_, getEdgeValue  , App, this);
  CTCL_OBJECT_PROC(tcl_, setEdgeValue  , App, this);
  CTCL_OBJECT_PROC(tcl_, getVertexValue, App, this);
  CTCL_OBJECT_PROC(tcl_, setVertexValue, App, this);

  CTCL_OBJECT_PROC(tcl_, setMaterialValue, App, this);

  CTCL_OBJECT_PROC(tcl_, getTextValue, App, this);
  CTCL_OBJECT_PROC(tcl_, setTextValue, App, this);

  CTCL_OBJECT_PROC(tcl_, getObjectProperty, App, this);
  CTCL_OBJECT_PROC(tcl_, setObjectProperty, App, this);

  // operate on primitives
  CTCL_OBJECT_PROC(tcl_, intersectObjects, App, this);
  CTCL_OBJECT_PROC(tcl_, inverseObject   , App, this);
  CTCL_OBJECT_PROC(tcl_, unionObjects    , App, this);
  CTCL_OBJECT_PROC(tcl_, subtractObjects , App, this);

  CTCL_OBJECT_PROC(tcl_, extrudeFaces, App, this);
  CTCL_OBJECT_PROC(tcl_, extrudeEdges, App, this);
  CTCL_OBJECT_PROC(tcl_, mergeEdge   , App, this);
  CTCL_OBJECT_PROC(tcl_, separateFace, App, this);
  CTCL_OBJECT_PROC(tcl_, separateEdge, App, this);
  CTCL_OBJECT_PROC(tcl_, mirrorObject, App, this);
  CTCL_OBJECT_PROC(tcl_, fillVertices, App, this);

  CTCL_OBJECT_PROC(tcl_, deleteObjects , App, this);
  CTCL_OBJECT_PROC(tcl_, deleteFaces   , App, this);
  CTCL_OBJECT_PROC(tcl_, deleteVertices, App, this);

  // animate
  CTCL_OBJECT_PROC(tcl_, animateReal, App, this);

  // import/export
  CTCL_OBJECT_PROC(tcl_, readModel, App, this)
  CTCL_OBJECT_PROC(tcl_, writeObj , App, this)

  // vector
  CTCL_OBJECT_PROC (tcl_, vector     , App, this);
  CTCL_TCL_OBJ_PROC(tcl_, get_vector , App, this);
  CTCL_TCL_OBJ_PROC(tcl_, set_vector , App, this);
  CTCL_OBJECT_PROC (tcl_, calc_vector, App, this);

  // matrix
  CTCL_OBJECT_PROC (tcl_, matrix     , App, this);
  CTCL_TCL_OBJ_PROC(tcl_, get_matrix , App, this);
  CTCL_TCL_OBJ_PROC(tcl_, set_matrix , App, this);
  CTCL_OBJECT_PROC (tcl_, calc_matrix, App, this);

  //---

  runTclCmd("proc mousePress { args } { }");

  runTclCmd("proc keyPress { args } { }");

  runTclCmd("proc selectionProc { args } { }");

  runTclCmd("proc tickProc { args } { }");
}

int
App::
execFile(const std::string &filename)
{
  std::string res;
  return tcl_->eval("source \"" + filename + "\"", res, /*showError*/true);
}

//---

int
App::
getAppValueProc(const CTclUtil::StringList &args)
{
  if (args.size() < 1)
    return tclErrorMsg("Invalid args");

  auto name = args[0];

  if      (name == "color") {
    if (args.size() < 2)
      return tclErrorMsg("Invalid args");

    CRGBA c;
    if (! stringToColor(args[1], c))
      return tclErrorMsg("Invalid color");

    tcl_->setResult(colorToRealArray(c));
  }
  else if (name == "cursor") {
    auto p = this->cursor();

    tcl_->setResult(pointToRealArray(p));
  }
  else if (name == "objects") {
    StringList ids;

    for (auto *object : scene_->getObjects()) {
      auto objectId = encodeObject(object);

      ids.push_back(objectId);
    }

    tcl_->setResult(ids);
  }
  else if (name == "nearest_object") {
    if (args.size() < 2)
      return tclErrorMsg("Invalid args");

    CPoint3D p;
    if (! stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    auto *object = getNearestObject(p);

    tcl_->setResult(encodeObject(object));
  }
  else if (name == "gravity") {
    auto *gravity = psys_->getGravity();

    tcl_->setResult(pointToRealArray(CPoint3D(gravity->x(), gravity->y(), gravity->z())));
  }
  else if (name == "drag") {
    auto drag = psys_->drag();

    tcl_->setResult(drag);
  }
  else if (name == "particles") {
    StringList ids;

    const auto &particles = psys_->getParticles();

    for (uint i = 0; i < particles.size(); ++i) {
      auto *particle = particles.get(int(i));

      auto id = encodeParticle(particle);

      ids.push_back(id);
    }

    tcl_->setResult(ids);
  }
  else if (name == "running") {
    tcl_->setResult(isRunning());
  }
  else if (name == "ticks") {
    tcl_->setResult(int(ticks()));
  }
  else if (name == "viewport") {
    tcl_->setResult(int(canvas()->currentViewport()));
  }
  else if (name == "key") {
    if (args.size() < 2)
      return tclErrorMsg("Invalid args");

    tcl_->setResult(canvas()->getKeyPressed(args[1]));
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setAppValueProc(const CTclUtil::StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  auto name = args[0];

  if      (name == "bg_color" || name == "bgColor") {
    CRGBA c;
    if (! stringToColor(args[1], c))
      return tclErrorMsg("Invalid color '" + args[1] + "'");

    canvas()->setBgColor(RGBAToQColor(c));
  }
  else if (name == "point_size") {
    double size;
    if (! stringToReal(args[1], size))
      return tclErrorMsg("Invalid size '" + args[1] + "'");

    canvas()->setPointSize(size);
  }
  else if (name == "cursor") {
    CPoint3D p;
    if (! stringToPoint(args[1], p))
      return tclErrorMsg("Invalid point '" + args[1] + "'");

    setCursor(p);
  }
  else if (name == "gravity") {
    double gravity;
    if (! stringToReal(args[1], gravity))
      return tclErrorMsg("Invalid gravity '" + args[1] + "'");

    psys_->setGravity(gravity);
  }
  else if (name == "drag") {
    double drag;
    if (! stringToReal(args[1], drag))
      return tclErrorMsg("Invalid drag '" + args[1] + "'");

    psys_->setDrag(drag);
  }
  else if (name == "bbox") {
    CBBox3D bbox;

    if (! stringToBBox(args[1], bbox))
      return tclErrorMsg("Invalid bbox '" + args[1] + "'");

    canvas()->setBBox(bbox);
  }
  else if (name == "fixed_diffuse") {
    bool b;
    if (! stringToBool(args[1], b))
      return tclErrorMsg("Invalid bool");

    canvas()->setFixedDiffuse(b);
  }
  else if (name == "fov") {
    double fov;
    if (! stringToReal(args[1], fov))
      return tclErrorMsg("Invalid fov '" + args[1] + "'");

    canvas()->camera()->setFov(fov);
  }
  else if (name == "pitch") {
    double pitch;
    if (! stringToReal(args[1], pitch))
      return tclErrorMsg("Invalid pitch '" + args[1] + "'");

    canvas()->camera()->setPitch(degToRad(pitch));
  }
  else if (name == "yaw") {
    double yaw;
    if (! stringToReal(args[1], yaw))
      return tclErrorMsg("Invalid yaw '" + args[1] + "'");

    canvas()->camera()->setYaw(degToRad(yaw));
  }
  else if (name == "roll") {
    double roll;
    if (! stringToReal(args[1], roll))
      return tclErrorMsg("Invalid roll '" + args[1] + "'");

    canvas()->camera()->setRoll(degToRad(roll));
  }
  else if (name == "camera_distance") {
    double distance;
    if (! stringToReal(args[1], distance))
      return tclErrorMsg("Invalid distance '" + args[1] + "'");

    canvas()->camera()->setDistance(distance);
  }
#if 0
  else if (name == "perspective") {
    bool b;
    if (! stringToBool(args[1], b))
      return tclErrorMsg("Invalid bool");

    canvas()->setPerspective(b);
  }
#endif
  else if (name == "running") {
    bool b;
    if (! stringToBool(args[1], b))
      return TCL_ERROR;

    setRunning(b);
  }
  else if (name == "tick") {
    tick(false);
  }
  else if (name == "edit_type") {
    auto astr = toLower(args[1]);
    if      (astr == "tcl")
      canvas()->setEditType(EditType::TCL);
    else if (astr == "select")
      canvas()->setEditType(EditType::SELECT);
    else if (astr == "camera")
      canvas()->setEditType(EditType::CAMERA);
    else
      return tclErrorMsg("Invalid edit type '" + args[1] + "'");
  }
  else if (name == "viewport") {
    int v;
    if (! stringToInteger(args[1], v))
      return tclErrorMsg("Invalid integer '" + args[1] + "'");

    canvas()->setCurrentViewport(v);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

//---

int
App::
addViewportProc(const CTclUtil::StringList &args)
{
  if (args.size() < 1)
    return tclErrorMsg("Invalid args");

  CBBox2D r;
  if (! stringToRect(args[0], r))
    return tclErrorMsg("Invalid rect '" + args[0] + "'");

  auto id = canvas()->addViewport(r);

  tcl_->setResult(id);

  return TCL_OK;
}

int
App::
setViewportValueProc(const CTclUtil::StringList &args)
{
  if (args.size() < 3)
    return tclErrorMsg("Invalid args");

  auto id = args[0];

  auto *viewportData = canvas()->getViewportData(id);
  if (! viewportData)
    return tclErrorMsg("Invalid viewport id");

  auto name = args[1];

  if      (name == "bg_color" || name == "bgColor") {
    CRGBA c;
    if (! stringToColor(args[2], c))
      return tclErrorMsg("Invalid color '" + args[1] + "'");

    viewportData->bgColor = RGBAToQColor(c);
  }
  else if (name == "bbox") {
    CBBox3D bbox;

    if (! stringToBBox(args[2], bbox))
      return tclErrorMsg("Invalid bbox '" + args[1] + "'");

    canvas()->setBBox(viewportData, bbox);
  }
  else if (name == "camera_distance") {
    double distance;
    if (! stringToReal(args[2], distance))
      return tclErrorMsg("Invalid distance '" + args[1] + "'");

    viewportData->camera->setDistance(distance);
  }
  else if (name == "clip") {
    if (args.size() < 4)
      return tclErrorMsg("Invalid args");

    CVector3D n;
    if (! stringToVector(args[2], n))
      return tclErrorMsg("Invalid clip normal '" + args[2] + "'");

    double d;
    if (! stringToReal(args[3], d))
      return tclErrorMsg("Invalid clip distance '" + args[3] + "'");

    viewportData->clips.push_back(CPlane3D(n, d));
  }
  else {
    return tclErrorMsg("Invalid value name");
  }

  return TCL_OK;
}

//---

int
App::
addShaderProc(const CTclUtil::StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  auto vs = args[0];

  int i = 1;

  std::string gs;

  if (args.size() >= 3)
    gs = args[i++];

  auto fs = args[i];

  std::string id;

  if (gs != "")
    id = canvas()->addShaderData(vs, gs, fs);
  else
    id = canvas()->addShaderData(vs, fs);

  tcl_->setResult(id);

  return TCL_OK;
}

int
App::
setShaderValueProc(const CTclUtil::StringList &args)
{
  if (args.size() < 3)
    return tclErrorMsg("Invalid args");

  auto id = args[0];

  auto *shaderData = canvas()->getShaderData(id);
  if (! shaderData)
    return tclErrorMsg("Invalid Shader id");

  auto name = args[1];

  if      (name == "point") {
    bool b;
    if (! stringToBool(args[2], b))
      return TCL_ERROR;

    shaderData->point = b;
  }
  else if (name == "line_width") {
    double w;
    if (! stringToReal(args[2], w))
      return tclErrorMsg("Invalid width");

    shaderData->lineWidth = w;
  }
  else {
    return tclErrorMsg("Invalid value name");
  }

  return TCL_OK;
}

//---

int
App::
addParticleProc(const CTclUtil::StringList &args)
{
  if (args.size() < 1)
    return tclErrorMsg("Invalid args");

  CPoint3D p;
  if (! stringToPoint(args[0], p))
    return tclErrorMsg("Invalid point '" + args[1] + "'");

  double mass = 1.0;

  if (args.size() > 1) {
    if (! stringToReal(args[1], mass))
      return tclErrorMsg("Invalid mass");
  }

  if (args.size() > 2) {
    CGeomObject3D *object;

    if (! decodeObject(args[2], object))
      return tclErrorMsg("Invalid object id '" + args[2] + "'");

    auto *object1 = dynamic_cast<GeomObject *>(object);
    assert(object1);

    psys_->setParticleObj(object1);
  }

  auto *particle = psys_->makeParticle(mass, p.x, p.y, p.z);

  psys_->setParticleObj(nullptr);

  tcl_->setResult(encodeParticle(particle));

  canvas()->updateScene(/*updateBBox*/false);

  return TCL_OK;
}

int
App::
getParticleValueProc(const CTclUtil::StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CPSysParticle *particle;
  if (! decodeParticle(args[0], particle))
    return tclErrorMsg("Invalid particle id '" + args[0] + "'");

  auto *particle1 = dynamic_cast<Particle *>(particle);

  auto name = args[1];

  if      (name == "position") {
    auto *position = particle->position();

    tcl_->setResult(pointToRealArray(CPoint3D(position->x(), position->y(), position->z())));
  }
  else if (name == "velocity") {
    auto *velocity = particle->velocity();

    tcl_->setResult(pointToRealArray(CPoint3D(velocity->x(), velocity->y(), velocity->z())));
  }
  else if (name == "mass") {
    auto mass = particle->mass();

    tcl_->setResult(mass);
  }
  else if (name == "force") {
    auto *force = particle->force();

    tcl_->setResult(pointToRealArray(CPoint3D(force->x(), force->y(), force->z())));
  }
  else if (name == "age") {
    auto age = particle->age();

    tcl_->setResult(age);
  }
  else if (name == "color") {
    if (! particle1)
      return tclErrorMsg("Invalid particle");

    const auto &color = particle1->color();

    tcl_->setResult(colorToRealArray(color));
  }
  else if (name == "dead") {
    auto dead = particle->isDead();

    tcl_->setResult(dead);
  }
  else if (name == "size") {
    if (! particle1)
      return tclErrorMsg("Invalid particle");

    auto size = particle1->size();

    tcl_->setResult(size);
  }
  else if (name == "alpha") {
    if (! particle1)
      return tclErrorMsg("Invalid particle");

    auto alpha = particle1->alpha();

    tcl_->setResult(alpha);
  }
  else if (name == "angle") {
    if (! particle1)
      return tclErrorMsg("Invalid particle");

    auto angle = 180.0*particle1->angle()/M_PI;

    tcl_->setResult(angle);
  }
  else if (name == "meta") {
    if (! particle1)
      return tclErrorMsg("Invalid particle");

    const auto &meta = particle1->meta();

    tcl_->setResult(meta);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setParticleValueProc(const std::vector<Tcl_Obj *> &objs)
{
  if (objs.size() < 3)
    return tclErrorMsg("Invalid args");

  CPSysParticle *particle;
  if (! decodeParticle(objs[0], particle))
    return TCL_ERROR;

  auto *particle1 = dynamic_cast<Particle *>(particle);

  auto name = CTclUtil::stringFromObj(objs[1]);

  if      (name == "position") {
    CPoint3D p;
    if (! decodePoint(objs[2], p))
      return TCL_ERROR;

    particle->setPosition(p.x, p.y, p.z);
  }
  else if (name == "velocity") {
    CPoint3D p;
    if (! decodePoint(objs[2], p))
      return TCL_ERROR;

    particle->setVelocity(p.x, p.y, p.z);
  }
  else if (name == "mass") {
    double mass;
    if (! decodeReal(objs[2], mass))
      return TCL_ERROR;

    particle->setMass(mass);
  }
  else if (name == "force") {
    CPoint3D p;
    if (! decodePoint(objs[2], p))
      return TCL_ERROR;

    particle->setForce(p.x, p.y, p.z);
  }
  else if (name == "age") {
    double age;
    if (! decodeReal(objs[2], age))
      return TCL_ERROR;

    particle->setAge(age);
  }
  else if (name == "color") {
    if (! particle1)
      return tclErrorMsg("Invalid particle");

    CPoint3D p;
    if (! decodePoint(objs[2], p))
      return TCL_ERROR;

    particle1->setColor(CRGBA(p.x, p.y, p.z));
  }
  else if (name == "dead") {
    bool dead;
    if (! decodeBool(objs[2], dead))
      return TCL_ERROR;

    particle->setDead(dead);
  }
  else if (name == "texture") {
    if (! particle1)
      return tclErrorMsg("Invalid particle");

    CGeomTexture* texture;
    if (! decodeTexture(objs[2], texture))
      return TCL_ERROR;

    auto *texture1 = dynamic_cast<Texture *>(texture);
    assert(texture1);

    particle1->setTexture(texture1);
  }
  else if (name == "size") {
    if (! particle1)
      return tclErrorMsg("Invalid particle");

    double size;
    if (! decodeReal(objs[2], size))
      return TCL_ERROR;

    particle1->setSize(size);
  }
  else if (name == "alpha") {
    if (! particle1)
      return tclErrorMsg("Invalid particle");

    double alpha;
    if (! decodeReal(objs[2], alpha))
      return TCL_ERROR;

    particle1->setAlpha(alpha);
  }
  else if (name == "angle") {
    if (! particle1)
      return tclErrorMsg("Invalid particle");

    double angle;
    if (! decodeReal(objs[2], angle))
      return TCL_ERROR;

    particle1->setAngle(M_PI*angle/180.0);
  }
  else if (name == "tpos") {
    if (! particle1)
      return tclErrorMsg("Invalid particle");

    CPoint2D tpos;
    if (! decodePoint(objs[2], tpos))
      return TCL_ERROR;

    particle1->setTPos(tpos);
  }
  else if (name == "tsize") {
    if (! particle1)
      return tclErrorMsg("Invalid particle");

    CPoint2D tsize;
    if (! decodePoint(objs[2], tsize))
      return TCL_ERROR;

    particle1->setTSize(CSize2D(tsize.x, tsize.y));
  }
  else if (name == "shader") {
    if (! particle1)
      return tclErrorMsg("Invalid particle");

    auto id = CTclUtil::stringFromObj(objs[2]);

    particle1->setShader(id);
  }
  else if (name == "meta") {
    if (! particle1)
      return tclErrorMsg("Invalid particle");

    auto value = CTclUtil::stringFromObj(objs[2]);

    particle1->setMeta(value);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  canvas()->updateScene(/*updateBBox*/false);

  return TCL_OK;
}

int
App::
addSpringProc(const CTclUtil::StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CPSysParticle *particle1;
  if (! decodeParticle(args[0], particle1))
    return tclErrorMsg("Invalid particle id '" + args[0] + "'");

  CPSysParticle *particle2;
  if (! decodeParticle(args[1], particle2))
    return tclErrorMsg("Invalid particle id '" + args[1] + "'");

  auto *spring = psys_->makeSpring(particle1, particle2);

  tcl_->setResult(encodeSpring(spring));

  canvas()->updateScene(/*updateBBox*/false);

  return TCL_OK;
}

int
App::
getSpringValueProc(const CTclUtil::StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CPSysSpring *spring;
  if (! decodeSpring(args[0], spring))
    return tclErrorMsg("Invalid spring id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "strength") {
    auto strength = spring->strength();

    tcl_->setResult(strength);
  }
  else if (name == "damping") {
    auto damping = spring->damping();

    tcl_->setResult(damping);
  }
  else if (name == "rest_length") {
    auto restLength = spring->restLength();

    tcl_->setResult(restLength);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setSpringValueProc(const CTclUtil::StringList &args)
{
  if (args.size() < 3)
    return tclErrorMsg("Invalid args");

  CPSysSpring *spring;
  if (! decodeSpring(args[0], spring))
    return tclErrorMsg("Invalid spring id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "strength") {
    double strength;
    if (! stringToReal(args[2], strength))
      return tclErrorMsg("Invalid strength '" + args[2] + "'");

    spring->setStrength(strength);
  }
  else if (name == "damping") {
    double damping;
    if (! stringToReal(args[2], damping))
      return tclErrorMsg("Invalid damping '" + args[2] + "'");

    spring->setDamping(damping);
  }
  else if (name == "rest_length") {
    double restLength;
    if (! stringToReal(args[2], restLength))
      return tclErrorMsg("Invalid rest length '" + args[2] + "'");

    spring->setRestLength(restLength);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
addAttractionProc(const CTclUtil::StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CPSysParticle *particle1;
  if (! decodeParticle(args[0], particle1))
    return tclErrorMsg("Invalid particle id '" + args[0] + "'");

  CPSysParticle *particle2;
  if (! decodeParticle(args[1], particle2))
    return tclErrorMsg("Invalid particle id '" + args[1] + "'");

  auto *attraction = psys_->makeAttraction(particle1, particle2);

  tcl_->setResult(encodeAttraction(attraction));

  canvas()->updateScene(/*updateBBox*/false);

  return TCL_OK;
}

int
App::
getAttractionValueProc(const CTclUtil::StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CPSysAttraction *attraction;
  if (! decodeAttraction(args[0], attraction))
    return tclErrorMsg("Invalid attraction id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "strength") {
    auto strength = attraction->getStrength();

    tcl_->setResult(strength);
  }
  else if (name == "min_distance") {
    auto dist = attraction->getMinimumDistance();

    tcl_->setResult(dist);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setAttractionValueProc(const CTclUtil::StringList &args)
{
  if (args.size() < 3)
    return tclErrorMsg("Invalid args");

  CPSysAttraction *attraction;
  if (! decodeAttraction(args[0], attraction))
    return tclErrorMsg("Invalid attraction id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "strength") {
    double strength;
    if (! stringToReal(args[2], strength))
      return tclErrorMsg("Invalid mass '" + args[2] + "'");

    attraction->setStrength(strength);
  }
  else if (name == "min_distance") {
    double dist;
    if (! stringToReal(args[2], dist))
      return tclErrorMsg("Invalid distance '" + args[2] + "'");

    attraction->setMinimumDistance(dist);
  }
  else if (name == "target") {
    CPSysParticle *particle;
    if (! decodeParticle(args[2], particle))
      return tclErrorMsg("Invalid particle id '" + args[2] + "'");

    attraction->setB(particle);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

//---

int
App::
addObjectProc(const CTclUtil::StringList &args)
{
  if (args.size() != 0)
    return tclErrorMsg("Invalid args");

  auto name = "object." + std::to_string(scene()->getObjects().size() + 1);

  auto *object = CGeometry3DInst->createObject3D(scene(), name);

  scene()->addObject(object);

  tcl()->setResult(encodeObject(object));

  canvas()->updateScene(/*updateBBox*/false);

  return TCL_OK;
}

int
App::
addVertexProc(const CTclUtil::StringList &args)
{
  if (args.size() != 2)
    return tclErrorMsg("Invalid args");

  CGeomObject3D *object;
  if (! decodeObject(args[0], object))
    return tclErrorMsg("Invalid object id '" + args[0] + "'");

  CPoint3D p;
  if (! stringToPoint(args[1], p))
    return tclErrorMsg("Invalid point '" + args[1] + "'");

  auto vind = object->addVertex(p);

  tcl()->setResult(encodeObjectVertexId(object, vind));

  canvas()->updateScene(/*updateBBox*/false);

  return TCL_OK;
}

int
App::
addFaceProc(const CTclUtil::StringList &args)
{
  if (args.size() != 2)
    return tclErrorMsg("Invalid args");

  CGeomObject3D *object;
  if (! decodeObject(args[0], object))
    return tclErrorMsg("Invalid object id '" + args[0] + "'");

  StringList strs;
  tcl()->splitList(args[1], strs);

  std::vector<uint> vertices;

  for (const auto &str : strs) {
    int objId1, vertexId;
    if (! decodeObjectVertexId(str, objId1, vertexId) || objId1 != int(object->getInd()))
      return tclErrorMsg("Invalid vertex '" + str + "'");

    vertices.push_back(vertexId);
  }

  if (vertices.size() < 3)
    return tclErrorMsg("Invalid number of face vertices");

  auto faceId = object->addFace(vertices);

  tcl()->setResult(encodeObjectFaceId(object->getInd(), faceId));

  canvas()->updateScene(/*updateBBox*/false);

  return TCL_OK;
}

int
App::
addLineProc(const CTclUtil::StringList &args)
{
  if (args.size() != 2)
    return tclErrorMsg("Invalid args");

  CGeomObject3D *object;
  if (! decodeObject(args[0], object))
    return tclErrorMsg("Invalid object id '" + args[0] + "'");

  StringList strs;
  tcl()->splitList(args[1], strs);

  std::vector<uint> vertices;

  for (const auto &str : strs) {
    int objId1, vertexId;
    if (! decodeObjectVertexId(str, objId1, vertexId) || objId1 != int(object->getInd()))
      return tclErrorMsg("Invalid vertex '" + str + "'");

    vertices.push_back(vertexId);
  }

  if (vertices.size() != 2)
    return tclErrorMsg("Invalid number of line vertices");

  auto lineId = object->addLine(vertices[0], vertices[1]);

  tcl()->setResult(encodeObjectLineId(object->getInd(), lineId));

  canvas()->updateScene(/*updateBBox*/false);

  return TCL_OK;
}

int
App::
addMaterialProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  auto name = args[0];

  auto *material = CGeometry3DInst->createMaterial();

  material->setName(name);

  scene_->addMaterial(material);

  tcl()->setResult(encodeMaterial(material));

  return TCL_OK;
}

int
App::
addTextureProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  auto name = args[0];

  auto *texture = CGeometry3DInst->createTexture(name);
  if (! texture)
    return tclErrorMsg("Invalid texture");

  texture->setFilename(name);

  scene_->addTexture(texture);

  tcl()->setResult(encodeTexture(texture));

  return TCL_OK;
}

int
App::
addTextProc(const CTclUtil::StringList &args)
{
  if (args.size() != 3)
    return tclErrorMsg("Invalid args");

  auto str = args[0];

  CPoint3D p;
  if (! stringToPoint(args[1], p))
    return tclErrorMsg("Invalid point '" + args[1] + "'");

  double size;
  if (! stringToReal(args[2], size))
    return tclErrorMsg("Invalid size '" + args[2] + "'");

  auto *font = canvas()->font();

  auto *text = new Text(str);

  text->setId(canvas()->texts().size() + 1);
  text->setFont(font);
  text->setColor(CRGBA::white());
  text->setPosition(CGLVector3D(p.x, p.y, p.z));
  text->setSize(size);
  text->setOverlay(false);

  canvas()->addText(text);

  text->updateText();

  tcl()->setResult(encodeText(text));

  return TCL_OK;
}

int
App::
addPlaneProc(const CTclUtil::StringList &args)
{
  double w = 1.0;
  double h = 1.0;

  if      (args.size() == 2) {
    if (! stringToReal(args[0], w) || ! stringToReal(args[1], h))
      return tclErrorMsg("Invalid args");
  }
  else if (args.size() == 1) {
    if (! stringToReal(args[0], w))
      return tclErrorMsg("Invalid args");

    h = w;
  }
  else if (! args.empty())
    return tclErrorMsg("Invalid args");

  auto c = cursor();

  auto n = scene_->getObjects().size();
  auto name = "plane." + std::to_string(n + 1);

  //auto *plane = new CGeomPlane3D(scene_, name, c, w, h);
  auto *plane = CGeometry3DInst->createObject3D(scene_, name);

  CGeomPlane3D::addGeometry(plane, c, w, h);

  plane->setInd(CGeometry3DInst->nextObjectId());

  scene_->addObject(plane);

  tcl()->setResult(encodeObject(plane));

  return TCL_OK;
}

int
App::
addCubeProc(const CTclUtil::StringList &args)
{
  double r = 1.0;

  if      (args.size() == 1) {
    if (! stringToReal(args[0], r))
      return tclErrorMsg("Invalid args");
  }
  else if (! args.empty())
    return tclErrorMsg("Invalid args");

  auto c = cursor();

  auto n = scene_->getObjects().size();
  auto name = "cube." + std::to_string(n + 1);

//auto *cube = new CGeomCube3D(scene_, name, c, r);
  auto *cube = CGeometry3DInst->createObject3D(scene_, name);

  CGeomCube3D::addGeometry(cube, c, r);

  cube->setInd(CGeometry3DInst->nextObjectId());

  scene_->addObject(cube);

  tcl()->setResult(encodeObject(cube));

  return TCL_OK;
}

int
App::
addConeProc(const CTclUtil::StringList &args)
{
  double w = 1.0;
  double h = 1.0;

  if      (args.size() == 2) {
    if (! stringToReal(args[0], w) || ! stringToReal(args[1], h))
      return tclErrorMsg("Invalid args");
  }
  else if (args.size() == 1) {
    if (! stringToReal(args[0], w))
      return tclErrorMsg("Invalid args");

    h = w;
  }
  else if (! args.empty())
    return tclErrorMsg("Invalid args");

  auto c = cursor();

  auto n = scene_->getObjects().size();
  auto name = "cone." + std::to_string(n + 1);

  //auto *cone = new CGeomCone3D(scene_, name, c, w, h);
  auto *cone = CGeometry3DInst->createObject3D(scene_, name);

  CGeomCone3D::addGeometry(cone, c, w, h);

  cone->setInd(CGeometry3DInst->nextObjectId());

  scene_->addObject(cone);

  tcl()->setResult(encodeObject(cone));

  return TCL_OK;
}

int
App::
addCylinderProc(const CTclUtil::StringList &args)
{
  double w = 1.0;
  double h = 1.0;

  if      (args.size() == 2) {
    if (! stringToReal(args[0], w) || ! stringToReal(args[1], h))
      return tclErrorMsg("Invalid args");
  }
  else if (args.size() == 1) {
    if (! stringToReal(args[0], w))
      return tclErrorMsg("Invalid args");

    h = w;
  }
  else if (! args.empty())
    return tclErrorMsg("Invalid args");

  auto c = cursor();

  auto n = scene_->getObjects().size();
  auto name = "cylinder." + std::to_string(n + 1);

  //auto *cylinder = new CGeomCylinder3D(scene_, name, c, w, h);
  auto *cylinder = CGeometry3DInst->createObject3D(scene_, name);

  CGeomCylinder3D::addGeometry(cylinder, c, w, h);

  cylinder->setInd(CGeometry3DInst->nextObjectId());

  scene_->addObject(cylinder);

  tcl()->setResult(encodeObject(cylinder));

  return TCL_OK;
}

int
App::
addSphereProc(const CTclUtil::StringList &args)
{
  double r = 1.0;

  if      (args.size() == 1) {
    if (! stringToReal(args[0], r))
      return tclErrorMsg("Invalid args");
  }
  else if (! args.empty())
    return tclErrorMsg("Invalid args");

  auto c = cursor();

  auto n = scene_->getObjects().size();
  auto name = "sphere." + std::to_string(n + 1);

  //auto *sphere = new CGeomSphere3D(scene_, name, c, r);
  auto *sphere = CGeometry3DInst->createObject3D(scene_, name);

  CGeomSphere3D::addGeometry(sphere, c, r);

  sphere->setInd(CGeometry3DInst->nextObjectId());

  CGeomSphere3D::addTexturePoints(sphere);
  CGeomSphere3D::addNormals(sphere, 1.0);

  scene_->addObject(sphere);

  tcl()->setResult(encodeObject(sphere));

  return TCL_OK;
}

int
App::
addTerrainProc(const CTclUtil::StringList &args)
{
  if (args.size() != 3)
    return tclErrorMsg("Invalid args");

  double width  = 1.0;
  double height = 1.0;
  double depth  = 1.0;

  if (! stringToReal(args[0], width) ||
      ! stringToReal(args[1], height) ||
      ! stringToReal(args[2], depth))
    return tclErrorMsg("Invalid args");

  double xmin = 0.0;
  double ymin = 0.0;
  double xmax = width;
  double ymax = height;

  // calc terrain
  CSolidNoise2D noise;

  int n       = 100;
  int octaves = 8;

  std::vector<double> x, y, z;

  x.resize(n);
  y.resize(n);
  z.resize(n*n);

  for (int iy = 0, iz = 0; iy < n; ++iy) {
    y[iy] = CMathUtil::map(iy, 0, n - 1, ymin, ymax);

    for (int ix = 0; ix < n; ++ix, ++iz) {
      x[ix] = CMathUtil::map(ix, 0, n - 1, xmin, xmax);

      z[iz] = noise.turbulence(CVector2D(x[ix], y[iy]), octaves);
    }
  }

  //---

  auto name = "terrain." + std::to_string(n + 1);

  auto *object = CGeometry3DInst->createObject3D(scene_, name);

  scene_->addObject(object);

  //---

  auto addPoint = [&](const CPoint3D &p, const CRGBA &c, const CVector3D &normal,
                      const CPoint2D &tp) {
    auto *vertex = CGeometry3DInst->createVertex3D(object, p);

    object->addVertex(vertex);

    vertex->setColor(c);
    vertex->setNormal(normal);
    vertex->setTextureMap(tp);

    return vertex->getInd();
  };

  struct PointData {
    CPoint3D  p;
    CVector3D n;
    CPoint2D  tp;
    CRGBA     c;
  };

  auto pointColor = [&](const CPoint3D &p) {
    auto y1 = p.y/depth;

    if      (y1 < 0.2) {
      auto f = CMathUtil::map(y1, 0.0, 0.2, 0, 1.0);

      return CRGBA(0.1, 0.1, f);
    }
    else if (y1 < 0.8) {
      auto f = CMathUtil::map(y1, 0.2, 0.8, 0, 1.0);

      return CRGBA(0.1, f, 0.1);
    }
    else {
      auto f = CMathUtil::map(y1, 0.8, 1.0, 0, 1.0);

      return CRGBA(f, f, f);
    }
  };

  auto addRect = [&](const PointData &p1, const PointData &p2,
                     const PointData &p3, const PointData &p4) {
    auto v1 = addPoint(p1.p, p1.c, p1.n, p1.tp);
    auto v2 = addPoint(p2.p, p2.c, p2.n, p2.tp);
    auto v3 = addPoint(p3.p, p3.c, p3.n, p3.tp);
    auto v4 = addPoint(p4.p, p4.c, p4.n, p4.tp);

    std::vector<uint> vertices;

    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);
    vertices.push_back(v4);

    auto faceInd = object->addFace(vertices);

    auto *face = object->getFaceP(faceInd);

    auto fn = (p1.n + p2.n + p3.n + p4.n).normalized();

    face->setNormal(fn);
  };

  auto genIZ = [&](int ix, int iy) {
    return (iy*n + ix);
  };

  auto getTerrainPoint = [&](int ix, int iy) {
    auto x1 = width*x[ix];
    auto y1 = width*y[iy];

    auto z1 = depth*z[genIZ(ix, iy)];

    return CPoint3D(x1, z1, y1);
  };

  auto dx = 1.0/(n - 1.0);
  auto dy = 1.0/(n - 1.0);

  auto calcTerrainPoint = [&](int ix, int iy, PointData &p) {
    p.p = getTerrainPoint(ix, iy);

    if (ix > 0 && ix < n - 1 && iy > 0 && iy < n - 1) {
#if 1
      auto p1 = getTerrainPoint(ix - 1, iy);
      auto p2 = getTerrainPoint(ix + 1, iy);
      auto p3 = getTerrainPoint(ix, iy - 1);
      auto p4 = getTerrainPoint(ix, iy + 1);

      CVector3D diff1(p.p, p1);
      CVector3D diff2(p.p, p2);
      CVector3D diff3(p.p, p3);
      CVector3D diff4(p.p, p4);

      auto n1 = diff1.crossProduct(diff2).normalized();
      auto n2 = diff1.crossProduct(diff3).normalized();
      auto n3 = diff4.crossProduct(diff2).normalized();
      auto n4 = diff4.crossProduct(diff3).normalized();

      p.n = (n1 + n2 + n3 + n4).normalized();
#else
      p.n = CVector3D(0, 1, 0);
#endif
    }
    else
      p.n = CVector3D(0, 1, 0);

    p.tp = CPoint2D(ix*dx, iy*dy);

    p.c = pointColor(p.p);
  };

  for (int iy = 1; iy < n; ++iy) {
    for (int ix = 1; ix < n; ++ix) {
      PointData p1, p2, p3, p4;

      calcTerrainPoint(ix - 1, iy - 1, p1);
      calcTerrainPoint(ix    , iy - 1, p2);
      calcTerrainPoint(ix    , iy    , p3);
      calcTerrainPoint(ix - 1, iy    , p4);

      addRect(p1, p2, p3, p4);
    }
  }

  //---

  tcl()->setResult(encodeObject(object));

  return TCL_OK;
}

int
App::
getObjectValueProc(const StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CGeomObject3D *object;
  if (! decodeObject(args[0], object))
    return tclErrorMsg("Invalid object id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "name") {
    tcl()->setResult(object->getName());
  }
  else if (name == "selected") {
    tcl()->setResult(object->getSelected());
  }
  else if (name == "visible") {
    tcl()->setResult(object->getVisible());
  }
  else if (name == "faces") {
    const auto &faces = object->getFaces();

    StringList faceIds1;

    for (auto *face : faces) {
      auto faceId1 = encodeFace(face);

      faceIds1.push_back(faceId1);
    }

    tcl()->setResult(faceIds1);
  }
  else if (name == "edges") {
    const auto &edges = object->getEdges();

    StringList edgeIds1;

    for (auto *edge : edges) {
      auto edgeId1 = encodeEdge(edge);

      edgeIds1.push_back(edgeId1);
    }

    tcl()->setResult(edgeIds1);
  }
  else if (name == "vertices") {
    const auto &vertices = object->getVertices();

    StringList vertices1;

    for (auto *vertex : vertices) {
      if (! vertex) continue;

      auto vertexId1 = encodeVertex(vertex);

      vertices1.push_back(vertexId1);
    }

    tcl()->setResult(vertices1);
  }
  else if (name == "nearest_face") {
    if (args.size() < 2)
      return tclErrorMsg("Invalid args");

    CPoint3D p;
    if (! stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    auto *face = getNearestFace(object, p);

    tcl()->setResult(encodeFace(face));
  }
  else if (name == "named_face") {
    if (args.size() < 2)
      return tclErrorMsg("Invalid args");

    auto *face = getNamedFace(object, args[2]);

    tcl()->setResult(encodeFace(face));
  }
  else if (name == "nearest_edge") {
    if (args.size() < 2)
      return tclErrorMsg("Invalid args");

    CPoint3D p;
    if (! stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    auto *edge = getNearestEdge(object, p);

    tcl()->setResult(encodeEdge(edge));
  }
  else if (name == "nearest_vertex") {
    if (args.size() < 2)
      return tclErrorMsg("Invalid args");

    CPoint3D p;
    if (! stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    auto *vertex = getNearestVertex(object, p);

    tcl()->setResult(encodeVertex(vertex));
  }
  else if (name == "selected_faces") {
    const auto &faces = object->getFaces();

    StringList faceIds1;

    for (auto *face : faces) {
      if (face && ! face->getSelected())
        continue;

      auto faceId1 = encodeFace(face);

      faceIds1.push_back(faceId1);
    }

    tcl()->setResult(faceIds1);
  }
  else if (name == "selected_vertices") {
    const auto &vertices = object->getVertices();

    StringList vertexIds1;

    for (auto *vertex : vertices) {
      if (! vertex || ! vertex->getSelected())
        continue;

      auto vertexId1 = encodeVertex(vertex);

      vertexIds1.push_back(vertexId1);
    }

    tcl()->setResult(vertexIds1);
  }
  else if (name == "bbox") {
    CBBox3D bbox;
    object->getModelBBox(bbox);

    tcl()->setResult(bboxToRealArrays(bbox));
  }
  else if (name == "center") {
    auto c = object->getModelCenter();

    tcl()->setResult(pointToRealArray(c));
  }
  else if (name == "ref_object") {
    auto *object1 = object->createRef();

    object1->setInd(CGeometry3DInst->nextObjectId());

    scene_->addObject(object1);

    auto children = object1->hierChildren();

    for (auto *child : children) {
      child->setInd(CGeometry3DInst->nextObjectId());

      scene_->addObject(child);
    }

    scene()->addObject(object1);

    tcl()->setResult(encodeObject(object1));
  }
  else if (name == "child") {
    if (args.size() < 3)
      return tclErrorMsg("Invalid args");

    auto *child = object->getChildOfName(args[2]);

    tcl()->setResult(encodeObject(child));
  }
  else if (name == "meta") {
    auto *object1 = dynamic_cast<GeomObject *>(object);

    auto meta = (object1 ? object1->meta() : "");

    tcl()->setResult(meta);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setObjectValueProc(const StringList &args)
{
  if (args.size() < 3)
    return tclErrorMsg("Invalid args");

  CGeomObject3D *object;
  if (! decodeObject(args[0], object))
    return tclErrorMsg("Invalid object id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "name") {
    object->setName(args[2]);
  }
  else if (name == "selected") {
    bool selected;
    if (! stringToBool(args[2], selected))
      return tclErrorMsg("Invalid bool");

    object->setSelected(selected);
  }
  else if (name == "visible") {
    bool visible;
    if (! stringToBool(args[2], visible))
      return tclErrorMsg("Invalid bool");

    object->setVisible(visible);
  }
  else if (name == "material") {
    CGeomMaterial *material;
    if (! decodeMaterial(args[2], material))
      return tclErrorMsg("Invalid material id '" + args[2] + "'");

    object->setMaterialP(material);
  }
  else if (name == "translate") {
    StringList strs;
    tcl()->splitList(args[2], strs);

    double tx, ty, tz;

    if      (strs.size() == 3) {
      if (! stringToReal(strs[0], tx) ||
          ! stringToReal(strs[1], ty) ||
          ! stringToReal(strs[2], tz))
        return tclErrorMsg("Invalid translate value");
    }
    else if (strs.size() == 1) {
      if (! stringToReal(strs[0], tx))
        return tclErrorMsg("Invalid translate value");

      ty = tx;
      tz = tx;
    }
    else
      return tclErrorMsg("Invalid translate value");

    bool transform = false;

    if (args.size() > 3) {
      if (args[3] == "set")
        transform = true;
    }

    if (transform)
      object->setTranslate(tx, ty, tz);
    else
      object->translate(tx, ty, tz, /*hier*/true);

    canvas()->updateScene(/*updateBBox*/false);
  }
  else if (name == "scale") {
    StringList strs;
    tcl()->splitList(args[2], strs);

    double sx, sy, sz;

    if      (strs.size() == 3) {
      if (! stringToReal(strs[0], sx) ||
          ! stringToReal(strs[1], sy) ||
          ! stringToReal(strs[2], sz))
        return tclErrorMsg("Invalid scale value");
    }
    else if (strs.size() == 1) {
      if (! stringToReal(strs[0], sx))
        return tclErrorMsg("Invalid scale value");

      sy = sx;
      sz = sx;
    }
    else
      return tclErrorMsg("Invalid scale value");

    object->scale(sx, sy, sz, /*hier*/true);

    canvas()->updateScene(/*updateBBox*/false);
  }
  else if (name == "rotate") {
    if (args.size() < 4)
      return tclErrorMsg("Invalid rotate value");

    CVector3D v;
    if (! stringToVector(args[2], v))
      return tclErrorMsg("Invalid rotate vector '" + args[2] + "'");

    double a;
    if (! stringToReal(args[3], a))
      return tclErrorMsg("Invalid rotate angle '" + args[3] + "'");

    object->rotateModel(degToRad(a), v, /*hier*/true);

    canvas()->updateScene(/*updateBBox*/false);
  }
  else if (name == "rotate_at") {
    if (args.size() < 5)
      return tclErrorMsg("Invalid rotate_at value");

    CVector3D v;
    if (! stringToVector(args[2], v))
      return tclErrorMsg("Invalid rotate_at vector '" + args[2] + "'");

    CPoint3D o;
    if (! stringToPoint(args[3], o))
      return tclErrorMsg("Invalid rotate_at point '" + args[3] + "'");

    double a;
    if (! stringToReal(args[4], a))
      return tclErrorMsg("Invalid rotate angle '" + args[4] + "'");

    CMatrix3D m1, m2, m3;
    m1.setTranslation(o);
    m2.setRotation(a, v);
    m3.setTranslation(-o);

    object->transform(m1*m2*m3, /*hier*/true);

    canvas()->updateScene(/*updateBBox*/false);
  }
  else if (name == "rotate_xyz") {
    StringList strs;
    tcl()->splitList(args[2], strs);

    double ax, ay, az;

    if      (strs.size() == 3) {
      if (! stringToReal(strs[0], ax) ||
          ! stringToReal(strs[1], ay) ||
          ! stringToReal(strs[2], az))
        return tclErrorMsg("Invalid rotate_xyz value");
    }
    else if (strs.size() == 1) {
      if (! stringToReal(strs[0], ax))
        return tclErrorMsg("Invalid rotate_xyz value");

      ay = ax;
      az = ax;
    }
    else
      return tclErrorMsg("Invalid rotate_xyz value");

    object->rotateModelX(degToRad(ax));
    object->rotateModelY(degToRad(ay));
    object->rotateModelZ(degToRad(az));
  }
  else if (name == "meta") {
    auto *object1 = dynamic_cast<GeomObject *>(object);

    if (object1)
      object1->setMeta(args[2]);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  tcl()->setResult(encodeObject(object));

  return TCL_OK;
}

int
App::
getFaceValueProc(const StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CGeomFace3D *face;
  if (! decodeFace(args[0], face))
    return tclErrorMsg("Invalid face id '" + args[0] + "'");

  auto *object = face->getObject();

  auto name = args[1];

  if      (name == "ind") {
    tcl()->setResult(int(face->getInd()));
  }
  else if (name == "name") {
    tcl()->setResult(face->name());
  }
  else if (name == "color") {
    auto color = face->getColor();

    tcl()->setResult(color.stringEncode());
  }
  else if (name == "center") {
    auto center = face->calcModelCenter();

    tcl()->setResult(pointToRealArray(center));
  }
  else if (name == "normal") {
    CVector3D normal;
    face->calcModelNormal(normal);

    tcl()->setResult(pointToRealArray(normal.point()));
  }
  else if (name == "edge_vector") {
    if (args.size() < 3)
      return tclErrorMsg("Invalid args");

    CGeomEdge3D *edge;
    if (! decodeEdge(args[2], edge))
      return tclErrorMsg("Invalid edge id '" + args[2] + "'");

    auto v = face->edgeVector(edge);

    tcl()->setResult(pointToRealArray(v.point()));
  }
  else if (name == "vertices") {
    const auto &vertexIds = face->getVertices();

    StringList vertices1;

    for (const auto &vertexId : vertexIds) {
      auto vertexId1 = encodeObjectVertexId(object, vertexId);

      vertices1.push_back(vertexId1);
    }

    tcl()->setResult(vertices1);
  }
  else if (name == "nearest_vertex") {
    if (args.size() < 2)
      return tclErrorMsg("Invalid args");

    CPoint3D p;
    if (! stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    auto *vertex = getNearestVertex(face, p);

    tcl()->setResult(encodeVertex(vertex));
  }
  else if (name == "edges") {
    const auto &edges = face->getEdges();

    StringList edges1;

    for (const auto &edge : edges) {
      auto edgeId1 = encodeEdge(edge);

      edges1.push_back(edgeId1);
    }

    tcl()->setResult(edges1);
  }
  else if (name == "nearest_edge") {
    if (args.size() < 2)
      return tclErrorMsg("Invalid args");

    CPoint3D p;
    if (! stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    auto *edge = getNearestEdge(face, p);

    tcl()->setResult(encodeEdge(edge));
  }
  else if (name == "bbox") {
    CBBox3D bbox;
    face->getModelBBox(bbox);

    tcl()->setResult(bboxToRealArrays(bbox));
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setFaceValueProc(const StringList &args)
{
  if (args.size() < 3)
    return tclErrorMsg("Invalid args");

  CGeomFace3D *face;
  if (! decodeFace(args[0], face))
    return tclErrorMsg("Invalid face id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "selected") {
    bool selected;
    if (! stringToBool(args[2], selected))
      return tclErrorMsg("Invalid bool");

    face->setSelected(selected);

    tcl()->setResult(encodeFace(face));
  }
  else if (name == "color") {
    if (args.size() != 3)
      return tclErrorMsg("Invalid args");

    CRGBA c;
    if (! stringToColor(args[2], c))
      return tclErrorMsg("Invalid color");

    face->setColor(c);

    tcl()->setResult(encodeFace(face));
  }
  else if (name == "normal") {
    if (args.size() != 3)
      return tclErrorMsg("Invalid args");

    CPoint3D p;
    if (! stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    face->setNormal(CVector3D(p));

    tcl()->setResult(encodeFace(face));
  }
  else if (name == "material") {
    CGeomMaterial *material;
    if (! decodeMaterial(args[2], material))
      return tclErrorMsg("Invalid material id '" + args[2] + "'");

    face->setMaterialP(material);

    tcl()->setResult(encodeFace(face));
  }
  else if (name == "translate") {
    StringList strs;
    tcl()->splitList(args[2], strs);

    double tx, ty, tz;

    if      (strs.size() == 3) {
      if (! stringToReal(strs[0], tx) ||
          ! stringToReal(strs[1], ty) ||
          ! stringToReal(strs[2], tz))
        return tclErrorMsg("Invalid translate value");
    }
    else if (strs.size() == 1) {
      if (! stringToReal(strs[0], tx))
        return tclErrorMsg("Invalid translate value");

      ty = tx;
      tz = tx;
    }
    else
      return tclErrorMsg("Invalid translate value");

    face->moveBy(CVector3D(tx, ty, tz));

    tcl()->setResult(encodeFace(face));
  }
  else if (name == "scale") {
    StringList strs;
    tcl()->splitList(args[2], strs);

    double sx, sy, sz;

    if      (strs.size() == 3) {
      if (! stringToReal(strs[0], sx) ||
          ! stringToReal(strs[1], sy) ||
          ! stringToReal(strs[2], sz))
        return tclErrorMsg("Invalid scale value");
    }
    else if (strs.size() == 1) {
      if (! stringToReal(strs[0], sx))
        return tclErrorMsg("Invalid scale value");

      sy = sx;
      sz = sx;
    }
    else
      return tclErrorMsg("Invalid scale value");

    face->scale(sx, sy, sz);

    tcl()->setResult(encodeFace(face));
  }
  else if (name == "center_scale") {
    if (args.size() < 4)
      return tclErrorMsg("Invalid args");

    CVector3D c;
    if (! stringToVector(args[2], c))
      return tclErrorMsg("Invalid center '" + args[2] + "'");

    StringList strs;
    tcl()->splitList(args[3], strs);

    double sx, sy, sz;

    if      (strs.size() == 3) {
      if (! stringToReal(strs[0], sx) ||
          ! stringToReal(strs[1], sy) ||
          ! stringToReal(strs[2], sz))
        return tclErrorMsg("Invalid scale value");
    }
    else if (strs.size() == 1) {
      if (! stringToReal(strs[0], sx))
        return tclErrorMsg("Invalid scale value");

      sy = sx;
      sz = sx;
    }
    else
      return tclErrorMsg("Invalid scale value");

    face->centerScale(c.point(), CVector3D(sx, sy, sz));

    tcl()->setResult(encodeFace(face));
  }
  else if (name == "rotate_xyz") {
    StringList strs;
    tcl()->splitList(args[2], strs);

    double ax, ay, az;

    if      (strs.size() == 3) {
      if (! stringToReal(strs[0], ax) ||
          ! stringToReal(strs[1], ay) ||
          ! stringToReal(strs[2], az))
        return tclErrorMsg("Invalid rotate value");
    }
    else if (strs.size() == 1) {
      if (! stringToReal(strs[0], ax))
        return tclErrorMsg("Invalid rotate value");

      ay = ax;
      az = ax;
    }
    else
      return tclErrorMsg("Invalid rotate value");

    face->rotateModelX(degToRad(ax));
    face->rotateModelY(degToRad(ay));
    face->rotateModelZ(degToRad(az));

    tcl()->setResult(encodeFace(face));
  }
  else if (name == "bevel") {
    double d;
    if (! stringToReal(args[2], d))
      return tclErrorMsg("Invalid bevel '" + args[2] + "'");

    face->bevel(d);

    tcl()->setResult(encodeFace(face));
  }
  else if (name == "inset") {
    double d;
    if (! stringToReal(args[2], d))
      return tclErrorMsg("Invalid inset '" + args[2] + "'");

    face->inset(d);

    tcl()->setResult(encodeFace(face));
  }
  else if (name == "subdivide") {
    int n;
    if (! stringToInteger(args[2], n) || n < 1)
      return tclErrorMsg("Invalid count '" + args[2] + "'");

    face->subdivide(n);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
getEdgeValueProc(const StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CGeomEdge3D *edge;
  if (! decodeEdge(args[0], edge))
    return tclErrorMsg("Invalid edge id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "start") {
    auto start = edge->getStart();

    tcl()->setResult(encodeObjectVertexId(edge->getObject(), start));
  }
  else if (name == "end") {
    auto end = edge->getEnd();

    tcl()->setResult(encodeObjectVertexId(edge->getObject(), end));
  }
  else if (name == "vertices") {
    const auto &vertexIds = edge->getVertices();

    StringList vertices1;

    for (const auto &vertexId : vertexIds) {
      auto vertexId1 = encodeObjectVertexId(edge->getObject(), vertexId);

      vertices1.push_back(vertexId1);
    }

    tcl()->setResult(vertices1);
  }
  else if (name == "normal") {
    auto normal = edge->calcNormal();

    tcl()->setResult(pointToRealArray(normal.point()));
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setEdgeValueProc(const StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CGeomEdge3D *edge;
  if (! decodeEdge(args[0], edge))
    return tclErrorMsg("Invalid edge id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "selected") {
    bool selected;
    if (! stringToBool(args[2], selected))
      return tclErrorMsg("Invalid bool");

    edge->setSelected(selected);
  }
  else if (name == "move") {
    if (args.size() < 3)
      return tclErrorMsg("Invalid args");

    StringList strs;
    tcl()->splitList(args[2], strs);

    CVector3D d;
    if (! stringToVector(args[2], d))
      return tclErrorMsg("Invalid delta '" + args[2] + "'");

    edge->moveBy(d);
  }
  else if (name == "scale") {
    if (args.size() < 3)
      return tclErrorMsg("Invalid args");

    double s;
    if (! stringToReal(args[2], s))
      return tclErrorMsg("Invalid scale '" + args[2] + "'");

    edge->scale(s);
  }
  else if (name == "bevel") {
    if (args.size() < 3)
      return tclErrorMsg("Invalid args");

    double s;
    if (! stringToReal(args[2], s))
      return tclErrorMsg("Invalid bevel '" + args[2] + "'");

    edge->bevel(s);
  }
  else if (name == "loop_cut") {
    int n = 1;

    if (args.size() > 2) {
      if (! stringToInteger(args[2], n))
        return tclErrorMsg("Invalid loop cut count '" + args[2] + "'");
    }

    auto faces = edge->loopCut(n);

    std::vector<StringList> ids;

    for (const auto &faces1 : faces) {
      StringList ids1;

      for (const auto &faceId : faces1)
        ids1.push_back(encodeObjectFaceId(edge->getObject()->getInd(), faceId));

      ids.push_back(ids1);
    }

    tcl()->setResult(ids);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
getLineValueProc(const StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CGeomLine3D *line;
  if (! decodeLine(args[0], line))
    return tclErrorMsg("Invalid line id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "start") {
    auto start = line->getStartInd();

    tcl()->setResult(encodeObjectVertexId(line->getObject(), start));
  }
  else if (name == "end") {
    auto end = line->getEndInd();

    tcl()->setResult(encodeObjectVertexId(line->getObject(), end));
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setLineValueProc(const StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CGeomLine3D *line;
  if (! decodeLine(args[0], line))
    return tclErrorMsg("Invalid line id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "selected") {
    bool selected;
    if (! stringToBool(args[2], selected))
      return tclErrorMsg("Invalid bool");

    line->setSelected(selected);
  }
  else if (name == "color") {
    if (args.size() != 3)
      return tclErrorMsg("Invalid args");

    CRGBA c;
    if (! stringToColor(args[2], c))
      return tclErrorMsg("Invalid color");

    line->setColor(c);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
getVertexValueProc(const StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CGeomVertex3D *vertex;
  if (! decodeVertex(args[0], vertex))
    return tclErrorMsg("Invalid vertex id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "model") {
    auto p = vertex->getModel();

    tcl()->setResult(pointToRealArray(p));
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setVertexValueProc(const StringList &args)
{
  if (args.size() < 3)
    return tclErrorMsg("Invalid args");

  CGeomVertex3D *vertex;
  if (! decodeVertex(args[0], vertex))
    return tclErrorMsg("Invalid vertex id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "selected") {
    bool selected;
    if (! stringToBool(args[2], selected))
      return tclErrorMsg("Invalid bool");

    vertex->setSelected(selected);
  }
  else if (name == "model") {
    CPoint3D p;
    if (! stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    vertex->setModel(p);
  }
  else if (name == "bevel") {
    double d;
    if (! stringToReal(args[2], d))
      return tclErrorMsg("Invalid bevel '" + args[2] + "'");

    vertex->bevel(d);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setMaterialValueProc(const StringList &args)
{
  if (args.size() < 3)
    return tclErrorMsg("Invalid args");

  CGeomMaterial *material;
  if (! decodeMaterial(args[0], material))
    return tclErrorMsg("Invalid material id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "diffuse") {
    if (args.size() != 3)
      return tclErrorMsg("Invalid args");

    CRGBA c;
    if (! stringToColor(args[2], c))
      return tclErrorMsg("Invalid color");

    material->setDiffuse(c);
  }
  else if (name == "texture" || name == "diffuse_texture") {
    CGeomTexture* texture;
    if (! decodeTexture(args[2], texture))
      return tclErrorMsg("Invalid texture id '" + args[2] + "'");

    material->setDiffuseTexture(texture);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  tcl()->setResult(encodeMaterial(material));

  return TCL_OK;
}

int
App::
getTextValueProc(const StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  Text *text;
  if (! decodeText(args[0], text))
    return tclErrorMsg("Invalid text id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "text") {
    tcl()->setResult(text->text());
  }
  else if (name == "position") {
    auto pos = text->position();

    tcl_->setResult(pointToRealArray(CPoint3D(pos.x(), pos.y(), pos.z())));
  }
  else if (name == "angle") {
    auto angle = text->angle();

    tcl_->setResult(pointToRealArray(CPoint3D(angle.x(), angle.y(), angle.z())));
  }
  else if (name == "size") {
    auto size = text->size();

    tcl_->setResult(size);
  }
  else if (name == "color") {
    auto c = text->color();

    tcl_->setResult(colorToRealArray(c));
  }
  else if (name == "overlay") {
    auto b = text->isOverlay();

    tcl_->setResult(b);
  }
  else if (name == "billboard") {
    auto b = text->isBillboard();

    tcl_->setResult(b);
  }
  else if (name == "viewport") {
    auto v = text->viewport();

    tcl_->setResult(v);
  }
  else if (name == "visible") {
    auto b = text->isVisible();

    tcl_->setResult(b);
  }
  else if (name == "halign") {
    auto a = text->halign();
    if      (a == Text::HAlign::LEFT  ) tcl_->setResult("left");
    else if (a == Text::HAlign::RIGHT ) tcl_->setResult("right");
    else if (a == Text::HAlign::CENTER) tcl_->setResult("center");
  }
  else if (name == "valign") {
    auto a = text->valign();
    if      (a == Text::VAlign::BOTTOM) tcl_->setResult("bottom");
    else if (a == Text::VAlign::TOP   ) tcl_->setResult("top");
    else if (a == Text::VAlign::CENTER) tcl_->setResult("center");
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setTextValueProc(const StringList &args)
{
  if (args.size() < 3)
    return tclErrorMsg("Invalid args");

  Text *text;
  if (! decodeText(args[0], text))
    return tclErrorMsg("Invalid text id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "text") {
    text->setText(args[2]);
  }
  else if (name == "position") {
    CPoint3D p;
    if (! stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    text->setPosition(CGLVector3D(p.x, p.y, p.z));
  }
  else if (name == "angle") {
    CPoint3D p;
    if (! stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    text->setAngle(CGLVector3D(p.x, p.y, p.z));
  }
  else if (name == "size") {
    double size;
    if (! stringToReal(args[2], size))
      return tclErrorMsg("Invalid size '" + args[2] + "'");

    text->setSize(size);
  }
  else if (name == "color") {
    CRGBA c;
    if (! stringToColor(args[2], c))
      return tclErrorMsg("Invalid color");

    text->setColor(c);
  }
  else if (name == "overlay") {
    bool b;
    if (! stringToBool(args[2], b))
      return tclErrorMsg("Invalid bool");

    text->setOverlay(b);
  }
  else if (name == "billboard") {
    bool b;
    if (! stringToBool(args[2], b))
      return tclErrorMsg("Invalid bool");

    text->setBillboard(b);
  }
  else if (name == "viewport") {
    int v;
    if (! stringToInteger(args[2], v))
      return tclErrorMsg("Invalid bool");

    text->setViewport(v);
  }
  else if (name == "visible") {
    bool b;
    if (! stringToBool(args[2], b))
      return tclErrorMsg("Invalid bool");

    text->setVisible(b);
  }
  else if (name == "halign") {
    auto astr = toLower(args[2]);
    if      (astr == "l" || astr == "left")
      text->setHAlign(Text::HAlign::LEFT);
    else if (astr == "r" || astr == "right")
      text->setHAlign(Text::HAlign::RIGHT);
    else if (astr == "c" || astr == "center")
      text->setHAlign(Text::HAlign::CENTER);
    else
      return tclErrorMsg("Invalid align '" + args[2] + "'");
  }
  else if (name == "valign") {
    auto astr = toLower(args[2]);
    if      (astr == "b" || astr == "bottom")
      text->setVAlign(Text::VAlign::BOTTOM);
    else if (astr == "t" || astr == "top")
      text->setVAlign(Text::VAlign::TOP);
    else if (astr == "c" || astr == "center")
      text->setVAlign(Text::VAlign::CENTER);
    else
      return tclErrorMsg("Invalid align '" + args[2] + "'");
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
getObjectPropertyProc(const CTclUtil::StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CGeomObject3D *object;
  if (! decodeObject(args[0], object))
    return tclErrorMsg("Invalid object id '" + args[0] + "'");

  auto *object1 = dynamic_cast<GeomObject *>(object);
  assert(object1);

  auto name = args[1];

  std::string value;

  if (! object1->getProperty(name, value))
    value = "";

  tcl_->setResult(value);

  return TCL_OK;
}

int
App::
setObjectPropertyProc(const CTclUtil::StringList &args)
{
  if (args.size() < 3)
    return tclErrorMsg("Invalid args");

  CGeomObject3D *object;
  if (! decodeObject(args[0], object))
    return tclErrorMsg("Invalid object id '" + args[0] + "'");

  auto *object1 = dynamic_cast<GeomObject *>(object);
  assert(object1);

  auto name  = args[1];
  auto value = args[2];

  object1->setProperty(name, value);

  return TCL_OK;
}

int
App::
intersectObjectsProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  StringList strs;
  tcl()->splitList(args[0], strs);

  std::vector<CGeomObject3D *> objects;

  for (const auto &str : strs) {
    CGeomObject3D *object;
    if (! decodeObject(str, object))
      return tclErrorMsg("Invalid object id '" + str + "'");

    objects.push_back(object);
  }

  auto *object = scene()->intersectObjects(objects);

  scene()->addObject(object);

  tcl()->setResult(encodeObject(object));

  return TCL_OK;
}

int
App::
inverseObjectProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  CGeomObject3D *object;
  if (! decodeObject(args[0], object))
    return tclErrorMsg("Invalid object id '" + args[0] + "'");

  auto *object1 = scene_->inverseObject(object);

  scene_->addObject(object1);

  tcl()->setResult(encodeObject(object1));

  return TCL_OK;
}

int
App::
unionObjectsProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  StringList strs;
  tcl()->splitList(args[0], strs);

  std::vector<CGeomObject3D *> objects;

  for (const auto &str : strs) {
    CGeomObject3D *object;
    if (! decodeObject(str, object))
      return tclErrorMsg("Invalid object id '" + str + "'");

    objects.push_back(object);
  }

  auto *object = scene_->unionObjects(objects);

  scene_->addObject(object);

  tcl()->setResult(encodeObject(object));

  return TCL_OK;
}

int
App::
subtractObjectsProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  StringList strs;
  tcl()->splitList(args[0], strs);

  std::vector<CGeomObject3D *> objects;

  for (const auto &str : strs) {
    CGeomObject3D *object;
    if (! decodeObject(str, object))
      return tclErrorMsg("Invalid object id '" + str + "'");

    objects.push_back(object);
  }

  auto *object = scene_->subtractObjects(objects);

  scene_->addObject(object);

  tcl()->setResult(encodeObject(object));

  return TCL_OK;
}

int
App::
extrudeFacesProc(const CTclUtil::StringList &args)
{
  if (args.size() != 2)
    return tclErrorMsg("Invalid args");

  StringList fstrs;
  tcl()->splitList(args[0], fstrs);

  if (fstrs.size() < 1)
    return tclErrorMsg("Invalid args");

  std::vector<CGeomFace3D *> faces;

  if (! decodeFaces(fstrs, faces))
    return TCL_ERROR;

  double d;
  if (! stringToReal(args[1], d))
    return tclErrorMsg("Invalid delta '" + args[1] + "'");

  if (faces.size() > 1) {
    auto *object = faces[0]->getObject();

    object->extrudeFaces(faces, d);
  }
  else {
    auto *face = faces[0];

    auto extrudeData = face->extrude(d);

    StringList faceIds;

    faceIds.push_back(encodeFace(extrudeData.topFace));

    for (auto *face : extrudeData.sideFaces)
      faceIds.push_back(encodeFace(face));

    tcl()->setResult(faceIds);
  }

  return TCL_OK;
}

int
App::
extrudeEdgesProc(const CTclUtil::StringList &args)
{
  if (args.size() != 2)
    return tclErrorMsg("Invalid args");

  CGeomEdge3D *edge;
  if (! decodeEdge(args[0], edge))
    return tclErrorMsg("Invalid edge id '" + args[0] + "'");

  double d;
  if (! stringToReal(args[1], d))
    return tclErrorMsg("Invalid delta '" + args[1] + "'");

  auto *face1 = edge->extrude(d);

  auto faceId1 = encodeFace(face1);

  tcl()->setResult(faceId1);

  return TCL_OK;
}

int
App::
mergeEdgeProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  CGeomEdge3D *edge;
  if (! decodeEdge(args[0], edge))
    return tclErrorMsg("Invalid edge id '" + args[0] + "'");

  auto vind = edge->getObject()->mergeEdge(edge->getInd());

  auto vertexId = encodeObjectVertexId(edge->getObject(), vind);

  tcl()->setResult(vertexId);

  return TCL_OK;
}

int
App::
duplicateFaceProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  CGeomFace3D *face;
  if (! decodeFace(args[0], face))
    return tclErrorMsg("Invalid face id '" + args[0] + "'");

  auto *face1 = face->duplicate();

  face->getObject()->addFace(face1);

  tcl()->setResult(encodeFace(face1));

  return TCL_OK;
}

int
App::
separateFaceProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  CGeomFace3D *face;
  if (! decodeFace(args[0], face))
    return tclErrorMsg("Invalid face id '" + args[0] + "'");

  auto *object1 = face->getObject()->separateFace(face);

  tcl()->setResult(encodeObject(object1));

  return TCL_OK;
}

int
App::
separateEdgeProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  CGeomEdge3D *edge;
  if (! decodeEdge(args[0], edge))
    return tclErrorMsg("Invalid edge id '" + args[0] + "'");

  auto *object1 = edge->getObject()->separateEdge(edge);

  tcl()->setResult(encodeObject(object1));

  return TCL_OK;
}

int
App::
mirrorObjectProc(const CTclUtil::StringList &args)
{
  if (args.size() != 2)
    return tclErrorMsg("Invalid args");

  CGeomObject3D *object;
  if (! decodeObject(args[0], object))
    return tclErrorMsg("Invalid edge id '" + args[0] + "'");

  uint mirrorDir = 0;

  for (int i = 0; args[1][i] != '\0'; ++i)
    if      (args[1][i] == 'x' || args[1][i] == 'X')
       mirrorDir |= uint(CGeomObject3D::MirrorDir::X);
    else if (args[1][i] == 'y' || args[1][i] == 'Y')
       mirrorDir |= uint(CGeomObject3D::MirrorDir::Y);
    else if (args[1][i] == 'z' || args[1][i] == 'Z')
       mirrorDir |= uint(CGeomObject3D::MirrorDir::Z);
    else {
      return tclErrorMsg("Invalid mirror direction '" + args[1] + "'");
  }

  auto c = cursor();

  const auto &objects = object->mirror(CGeomObject3D::MirrorDir(mirrorDir), c);

  StringList objectIds1;

  for (auto *object1 : objects) {
    auto name = "object." + std::to_string(scene_->getObjects().size() + 1);

    scene_->addObject(object1);

    object1->setName(name);

    object1->setInd(CGeometry3DInst->nextObjectId());

    auto objectId1 = encodeObject(object1);

    objectIds1.push_back(objectId1);
  }

  tcl()->setResult(objectIds1);

  return TCL_OK;
}

int
App::
fillVerticesProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  std::vector<CGeomVertex3D *> vertices;

  if (! decodeVertices(args[0], vertices))
    return TCL_ERROR;

  auto *object = vertices[0]->getObject();

  object->fillVertices(vertices);

  tcl()->setResult(0);

  return TCL_OK;
}

int
App::
scaleFacesProc(const CTclUtil::StringList &args)
{
  if (args.size() < 3)
    return tclErrorMsg("Invalid args");

  StringList fstrs;
  tcl_->splitList(args[0], fstrs);

  if (fstrs.size() < 1)
    return tclErrorMsg("Invalid args");

  std::vector<CGeomFace3D *> faces;

  if (! decodeFaces(fstrs, faces))
    return TCL_ERROR;

  CVector3D c;
  if (! stringToVector(args[1], c))
    return tclErrorMsg("Invalid center '" + args[1] + "'");

  StringList sstrs;
  tcl_->splitList(args[2], sstrs);

  double sx, sy, sz;

  if      (sstrs.size() == 3) {
    if (! stringToReal(sstrs[0], sx) ||
        ! stringToReal(sstrs[1], sy) ||
        ! stringToReal(sstrs[2], sz))
      return tclErrorMsg("Invalid scale value");
  }
  else if (sstrs.size() == 1) {
    if (! stringToReal(sstrs[0], sx))
      return tclErrorMsg("Invalid scale value");

    sy = sx;
    sz = sx;
  }
  else
    return tclErrorMsg("Invalid scale value");

  auto *object = faces[0]->getObject();

  if (! object->scaleFaces(faces, c.point(), CVector3D(sx, sy, sz)))
    return tclErrorMsg("scaleFaces failed");

  return TCL_OK;
}

int
App::
circularizeFacesProc(const CTclUtil::StringList &args)
{
  if (args.size() < 1)
    return tclErrorMsg("Invalid args");

  StringList strs;
  tcl_->splitList(args[0], strs);

  if (strs.size() < 1)
    return tclErrorMsg("Invalid args");

  std::vector<CGeomFace3D *> faces;

  if (! decodeFaces(strs, faces))
    return TCL_ERROR;

  auto *object = faces[0]->getObject();

  if (! object->circularizeFaces(faces))
    return tclErrorMsg("circularizeFaces failed");

  return TCL_OK;
}

int
App::
deleteObjectsProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  StringList strs;
  tcl()->splitList(args[0], strs);

  std::vector<CGeomObject3D *> objects;

  for (const auto &str : strs) {
    CGeomObject3D *object;
    if (! decodeObject(str, object))
      return tclErrorMsg("Invalid object id '" + str + "'");

    objects.push_back(object);
  }

  for (auto *object : objects) {
    scene_->removeObject(object, /*force*/true);

    delete object;
  }

  tcl()->setResult(0);

  return TCL_OK;
}

int
App::
deleteFacesProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  StringList strs;
  tcl()->splitList(args[0], strs);

  std::vector<CGeomFace3D *> faces;

  if (! decodeFaces(strs, faces))
    return TCL_ERROR;

  auto *object = faces[0]->getObject();

  for (auto *face : faces) {
    object->removeFace(face);

    delete face;
  }

  tcl()->setResult(0);

  return TCL_OK;
}

int
App::
deleteVerticesProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  StringList strs;
  tcl()->splitList(args[0], strs);

  std::vector<CGeomVertex3D *> vertices;

  for (const auto &str : strs) {
    CGeomVertex3D *vertex;
    if (! decodeVertex(str, vertex))
      return tclErrorMsg("Invalid object id '" + str + "'");

    vertices.push_back(vertex);
  }

  auto *object = vertices[0]->getObject();

  for (auto *vertex : vertices) {
    object->removeVertex(vertex);

    delete vertex;
  }

  tcl()->setResult(0);

  return TCL_OK;
}

int
App::
animateRealProc(const CTclUtil::StringList &args)
{
  if (args.size() != 2)
    return tclErrorMsg("Invalid args");

  StringList strs;
  tcl_->splitList(args[0], strs);

  if (strs.size() != 3)
    return false;

  double start, end, delta;
  if (! stringToReal(strs[0], start) ||
      ! stringToReal(strs[1], end) ||
      ! stringToReal(strs[2], delta))
    return false;

  auto proc = args[1];

  auto *animRealData = new AnimRealData(this, start, end, delta, proc);

  animRealData->setActive(true);

  animDatas_.push_back(animRealData);

  return TCL_OK;
}

int
App::
readModelProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  auto filename = args[0];

  auto format = CImportBase::filenameToType(filename);

  LoadData loadData;

#if 1
  if (! loadModel(filename, format, loadData))
    return tclErrorMsg("Failed to read model for '" + filename + "'");

  Q_EMIT modelAdded();
#else
  loadData.topObj = nullptr;

  auto *im = CImportBase::createModel(format, filename);

  if (! im)
    return tclErrorMsg("File format not recognised for '" + filename + "'");

  CFile file(filename);

  if (! im->read(file)) {
    delete im;
    return tclErrorMsg("Failed to read model for '" + filename + "'");
  }

  auto *scene = im->releaseScene();

  delete im;

  uint numTop = 0;

  for (auto *object : scene->getObjects()) {
    if (! object->parent()) {
      ++numTop;
      loadData.topObj = object;
    }
  }

  if (numTop > 1) {
    auto name = "object." + std::to_string(scene_->getObjects().size() + 1);

    auto *parentObj = CGeometry3DInst->createObject3D(scene_, name);

    scene_->addObject(parentObj);

    for (auto *object : scene->getObjects()) {
      scene_->addObject(object);

      if (! object->parent())
        parentObj->addChild(object);
    }

    loadData.topObj = parentObj;
  }
  else {
    for (auto *object : scene->getObjects())
      scene_->addObject(object);
  }

  for (auto *material : scene->getMaterials()) {
    scene_->addMaterial(material);
  }

  for (auto *texture : scene->textures()) {
    scene_->addTexture(texture);
  }
#endif

  tcl_->setResult(encodeObject(loadData.topObj));

  return TCL_OK;
}

int
App::
writeObjProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  CFile file(args[0]);

  CImportObj obj;

  obj.write(&file, scene_);

  return TCL_OK;
}

//---

int
App::
vectorProc(const CTclUtil::StringList &args)
{
  CVector3D v;

  if (args.size() == 3) {
    double x, y, z;
    if (! stringToReal(args[0], x) || ! stringToReal(args[1], y) || ! stringToReal(args[2], z))
      return tclErrorMsg("Invalid real values");

    v = CVector3D(x, y, z);
  }
  else if (args.size() == 1) {
    if (! CTclVector3D::fromString(args[0], v))
      return tclErrorMsg("Invalid vector '" + args[0] + "'");
  }
  else
    return tclErrorMsg("Invalid args");

  tcl()->setResult(vectorToObj(v));

  return TCL_OK;
}

int
App::
getVectorProc(const std::vector<Tcl_Obj *> &objs)
{
  if (objs.size() < 2)
    return tclErrorMsg("Invalid args");

  CVector3D v;
  if (! objToVector(objs[0], v))
    return tclErrorMsg("Invalid vector");

  auto name = CTclUtil::stringFromObj(objs[1]);

  if      (name == "x")
    tcl_->setResult(v.getX());
  else if (name == "y")
    tcl_->setResult(v.getY());
  else if (name == "z")
    tcl_->setResult(v.getZ());
  else if (name == "length")
    tcl_->setResult(v.length());
  else if (name == "distance" || name == "magnitude") {
    if (objs.size() < 3)
      return tclErrorMsg("Invalid args");

    CVector3D v1;
    if (! objToVector(objs[2], v1))
      return tclErrorMsg("Invalid vector");

    tcl_->setResult(v.getDistance(v1));
  }
  else if (name == "dot") {
    if (objs.size() < 3)
      return tclErrorMsg("Invalid args");

    CVector3D v1;
    if (! objToVector(objs[2], v1))
      return tclErrorMsg("Invalid vector");

    tcl_->setResult(v.dotProduct(v1));
  }
  else if (name == "cross") {
    if (objs.size() < 3)
      return tclErrorMsg("Invalid args");

    CVector3D v1;
    if (! objToVector(objs[2], v1))
      return tclErrorMsg("Invalid vector");

    tcl_->setResult(vectorToObj(v.crossProduct(v1)));
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setVectorProc(const std::vector<Tcl_Obj *> &objs)
{
  if (objs.size() < 3)
    return tclErrorMsg("Invalid args");

  CVector3D v;
  if (! objToVector(objs[0], v))
    return tclErrorMsg("Invalid vector");

  auto *res = objs[0];

  auto name = CTclUtil::stringFromObj(objs[1]);

  if      (name == "x") {
    double x;
    if (! objToReal(objs[2], x))
      return tclErrorMsg("Invalid real");

    v.setX(x);
  }
  else if (name == "y") {
    double y;
    if (! objToReal(objs[2], y))
      return tclErrorMsg("Invalid real");

    v.setY(y);
  }
  else if (name == "z") {
    double z;
    if (! objToReal(objs[2], z))
      return tclErrorMsg("Invalid real");

    v.setZ(z);
  }
  else if (name == "+" || name == "add") {
    CVector3D v1;
    if (! objToVector(objs[2], v1))
      return tclErrorMsg("Invalid vector");

    v += v1;
  }
  else if (name == "-" || name == "subtract") {
    CVector3D v1;
    if (! objToVector(objs[2], v1))
      return tclErrorMsg("Invalid vector");

    v -= v1;
  }
  else if (name == "*" || name == "multiply") {
    if (CTclVector3D::isObj(objs[2])) {
      CVector3D v1;
      if (! objToVector(objs[2], v1))
        return tclErrorMsg("Invalid vector");

      v *= v1;
    }
    else {
      double s;
      if (! objToReal(objs[2], s))
        return tclErrorMsg("Invalid real");

      v *= s;
    }
  }
  else if (name == "magnitude") {
    double s;
    if (! objToReal(objs[2], s))
      return tclErrorMsg("Invalid real");

    v.setMagnitude(s);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  if (CTclVector3D::setObj(tcl_->interp(), objs[0], v) == TCL_ERROR)
    return tclErrorMsg("Invalid vector");

  tcl()->setResult(res);

  return TCL_OK;
}

int
App::
calcVectorProc(const CTclUtil::StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CVector3D v;
  if (! stringToVector(args[0], v))
    return tclErrorMsg("Invalid vector '" + args[0] + "'");

  auto name = args[1];

  if     (name == "rotate") {
    if (args.size() < 4)
      return tclErrorMsg("Invalid args");

    CVector3D v1;
    if (! stringToVector(args[2], v1))
      return tclErrorMsg("Invalid vector '" + args[2] + "'");

    double a;
    if (! stringToReal(args[3], a))
      return tclErrorMsg("Invalid rotate angle '" + args[3] + "'");

    auto m = CMatrix3D::rotation(degToRad(a), v1);

    auto v2 = m*v;

    tcl_->setResult(pointToRealArray(v2.point()));
  }
  else if (name == "add") {
    if (args.size() < 3)
      return tclErrorMsg("Invalid args");

    CVector3D v1;
    if (! stringToVector(args[2], v1))
      return tclErrorMsg("Invalid vector '" + args[2] + "'");

    auto v2 = v + v1;

    tcl_->setResult(pointToRealArray(v2.point()));
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

//---

int
App::
matrixProc(const CTclUtil::StringList &args)
{
  CMatrix3D m;

  if      (args.size() == 16) {
    double mm[16];

    for (uint i = 0; i < 16; ++i) {
      if (! stringToReal(args[i], mm[i]))
        return tclErrorMsg("Invalid real values");
    }

    m = CMatrix3D(mm, 16);
  }
  else if (args.size() == 1) {
    if (args[0] == "identity") {
      m = CMatrix3D::identity();
    }
    else {
      if (! CTclMatrix3D::fromString(args[0], m))
        return tclErrorMsg("Invalid matrix '" + args[0] + "'");
    }
  }
  else if (args.size() == 0) {
  }
  else
    return tclErrorMsg("Invalid args");

  tcl()->setResult(matrixToObj(m));

  return TCL_OK;
}

int
App::
getMatrixProc(const std::vector<Tcl_Obj *> &objs)
{
  if (objs.size() < 2)
    return tclErrorMsg("Invalid args");

  if (! CTclMatrix3D::isObj(objs[0]))
    return tclErrorMsg("Invalid matrix");

  CMatrix3D m;
  if (! objToMatrix(objs[0], m))
    return tclErrorMsg("Invalid matrix");

  auto name = CTclUtil::stringFromObj(objs[1]);

  if      (name == "determinant")
    tcl_->setResult(m.determinant());
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setMatrixProc(const std::vector<Tcl_Obj *> &objs)
{
  if (objs.size() < 2)
    return tclErrorMsg("Invalid args");

  if (! CTclMatrix3D::isObj(objs[0]))
    return tclErrorMsg("Invalid matrix");

  CMatrix3D m;
  if (! objToMatrix(objs[0], m))
    return tclErrorMsg("Invalid matrix");

  auto *res = objs[0];

  auto name = CTclUtil::stringFromObj(objs[1]);

  if      (name == "translation") {
    if (objs.size() < 3)
      return tclErrorMsg("Invalid args");

    CVector3D v;
    if (! objToVector(objs[2], v))
      return tclErrorMsg("Invalid vector");

    m.setTranslation(v.getX(), v.getY(), v.getZ());
  }
  else if (name == "scale") {
    if (objs.size() < 3)
      return tclErrorMsg("Invalid args");

    CVector3D v;
    if (! objToVector(objs[2], v))
      return tclErrorMsg("Invalid vector");

    m.setScale(v.getX(), v.getY(), v.getZ());
  }
  else if (name == "rotation") {
    if (objs.size() < 4)
      return tclErrorMsg("Invalid args");

    CVector3D v;
    if (! objToVector(objs[2], v))
      return tclErrorMsg("Invalid rotate vector");

    double a;
    if (! objToReal(objs[3], a))
      return tclErrorMsg("Invalid rotate angle");

    m.setRotation(a, v);
  }
  else if (name == "+" || name == "add") {
    if (objs.size() < 3)
      return tclErrorMsg("Invalid args");

    CMatrix3D m1;
    if (! objToMatrix(objs[2], m1))
      return tclErrorMsg("Invalid matrix");

    auto m2 = m + m1;

    res = matrixToObj(m2);
  }
  else if (name == "-" || name == "subtract") {
    if (objs.size() < 3)
      return tclErrorMsg("Invalid args");

    CMatrix3D m1;
    if (! objToMatrix(objs[2], m1))
      return tclErrorMsg("Invalid matrix");

    auto m2 = m - m1;

    res = matrixToObj(m2);
  }
  else if (name == "*" || name == "multiply") {
    if (objs.size() < 3)
      return tclErrorMsg("Invalid args");

    CMatrix3D m2;

    if (CTclMatrix3D::isObj(objs[2])) {
      CMatrix3D m1;
      if (! objToMatrix(objs[2], m1))
        return tclErrorMsg("Invalid matrix");

      m2 = m*m1;
    }
    else {
      double s;
      if (! objToReal(objs[2], s))
        return tclErrorMsg("Invalid real");

      m2 = m*s;
    }

    res = matrixToObj(m2);
  }
  else if (name == "inverse") {
    auto m1 = m.inverse();

    res = matrixToObj(m1);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  if (CTclMatrix3D::setObj(tcl_->interp(), objs[0], m) == TCL_ERROR)
    return tclErrorMsg("Invalid matrix");

  tcl()->setResult(res);

  return TCL_OK;
}

int
App::
calcMatrixProc(const CTclUtil::StringList &args)
{
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CMatrix3D m;
  if (! stringToMatrix(args[0], m))
    return tclErrorMsg("Invalid matrix '" + args[0] + "'");

  auto name = args[1];

  return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}
//---

void
App::
timerSlot()
{
  if (! isRunning())
    return;

  tick();
}

void
App::
tick(bool update)
{
  ++ticks_;

  psys_->tick(0.01);

  for (auto *animData : animDatas_) {
    if (animData->isActive())
      animData->tick();
  }

  auto cmd = "tickProc";

  runTclCmd(cmd);

  if (update) {
    canvas()->update();

    overview()->invalidate();
  }
}

bool
App::
decodeParticle(Tcl_Obj *obj, CPSysParticle* &particle) const
{
  auto arg = CTclUtil::stringFromObj(obj);

  if (! decodeParticle(arg, particle))
    return errorMsg("Invalid particle id '" + arg + "'");

  return true;
}

bool
App::
decodeParticle(const std::string &arg, CPSysParticle* &particle) const
{
  int ind;
  if (! decodeParticleId(arg, ind))
    return false;

  if (ind >= 0) {
    particle = psys_->getParticle(ind);
    if (! particle)
      return false;
  }
  else
    particle = nullptr;

  return true;
}

bool
App::
decodeSpring(const std::string &arg, CPSysSpring* &spring) const
{
  int ind;
  if (! decodeSpringId(arg, ind))
    return false;

  spring = psys_->getSpring(ind);
  if (! spring)
    return false;

  return true;
}

bool
App::
decodeAttraction(const std::string &arg, CPSysAttraction* &attraction) const
{
  int ind;
  if (! decodeAttractionId(arg, ind))
    return false;

  attraction = psys_->getAttraction(ind);
  if (! attraction)
    return false;

  return true;
}

bool
App::
decodeObject(const std::string &arg, CGeomObject3D* &object) const
{
  int objId;
  if (! decodeObjectId(arg, objId))
    return false;

  object = scene_->getObjectByInd(objId);
  if (! object)
    return false;

  return true;
}

bool
App::
decodeFaces(const std::string &arg, std::vector<CGeomFace3D *> &faces) const
{
  StringList strs;
  tcl_->splitList(arg, strs);

  if (strs.size() < 1)
    return errorMsg("Invalid args");

  return decodeFaces(strs, faces);
}

bool
App::
decodeFaces(const std::vector<std::string> &strs, std::vector<CGeomFace3D *> &faces) const
{
  CGeomObject3D *object = nullptr;

  for (const auto &str : strs) {
    CGeomFace3D *face;
    if (! decodeFace(str, face))
      return errorMsg("Invalid face");

    auto *object1 = face->getObject();

    if (object && object1 != object)
      return errorMsg("inconsistent parent object");

    object = object1;

    faces.push_back(face);
  }

  if (faces.empty())
    return errorMsg("no faces specified");

  return true;
}

bool
App::
decodeFace(const std::string &arg, CGeomFace3D* &face) const
{
  int objId, faceId;
  if (! decodeObjectFaceId(arg, objId, faceId))
    return false;

  auto *object = scene_->getObjectByInd(objId);
  if (! object)
    return false;

  face = object->getFaceP(faceId);
  if (! face)
    return false;

  return true;
}

bool
App::
decodeEdge(const std::string &arg, CGeomEdge3D* &edge) const
{
  int objId, edgeId;
  if (! decodeObjectEdgeId(arg, objId, edgeId))
    return false;

  auto *object = scene_->getObjectByInd(objId);
  if (! object)
    return false;

  edge = const_cast<CGeomEdge3D *>(object->getEdgeP(edgeId));
  if (! edge)
    return false;

  return true;
}

bool
App::
decodeLine(const std::string &arg, CGeomLine3D* &line) const
{
  int objId, lineId;
  if (! decodeObjectLineId(arg, objId, lineId))
    return false;

  auto *object = scene_->getObjectByInd(objId);
  if (! object)
    return false;

  line = const_cast<CGeomLine3D *>(object->getLineP(lineId));
  if (! line)
    return false;

  return true;
}

bool
App::
decodeVertices(const std::string &arg, std::vector<CGeomVertex3D *> &vertices) const
{
  StringList strs;
  tcl_->splitList(arg, strs);

  if (strs.size() < 1)
    return errorMsg("Invalid args");

  return decodeVertices(strs, vertices);
}

bool
App::
decodeVertices(const std::vector<std::string> &strs, std::vector<CGeomVertex3D *> &vertices) const
{
  CGeomObject3D *object = nullptr;

  for (const auto &str : strs) {
    CGeomVertex3D *vertex;
    if (! decodeVertex(str, vertex))
      return errorMsg("Invalid vertex");

    auto *object1 = vertex->getObject();

    if (object && object1 != object)
      return errorMsg("inconsistent parent object");

    object = object1;

    vertices.push_back(vertex);
  }

  if (vertices.empty())
    return errorMsg("no vertices specified");

  return true;
}

bool
App::
decodeVertex(const std::string &arg, CGeomVertex3D* &vertex) const
{
  int objId, vertexId;
  if (! decodeObjectVertexId(arg, objId, vertexId))
    return false;

  auto *object = scene_->getObjectByInd(objId);
  if (! object)
    return false;

  vertex = const_cast<CGeomVertex3D *>(object->getVertexP(vertexId));
  if (! vertex)
    return false;

  return true;
}

bool
App::
decodeTexture(Tcl_Obj *obj, CGeomTexture* &texture) const
{
  auto arg = CTclUtil::stringFromObj(obj);

  if (! decodeTexture(arg, texture))
    return errorMsg("Invalid texture id '" + arg + "'");

  return true;
}

bool
App::
decodeTexture(const std::string &arg, CGeomTexture* &texture) const
{
  int textureId;
  if (! decodeTextureId(arg, textureId))
    return false;

  texture = scene_->getTextureById(textureId);
  if (! texture)
    return false;

  return true;
}

bool
App::
decodeMaterial(const std::string &arg, CGeomMaterial* &material) const
{
  int materialId;
  if (! decodeMaterialId(arg, materialId))
    return false;

  material = scene_->getMaterialById(materialId);
  if (! material)
    return false;

  return true;
}

bool
App::
decodeText(const std::string &arg, Text* &text) const
{
  int textId;
  if (! decodeTextId(arg, textId))
    return false;

  text = canvas_->getTextById(textId);
  if (! text)
    return false;

  return true;
}

CGeomObject3D *
App::
getNearestObject(const CPoint3D &p) const
{
  CGeomFace3D *minFace { nullptr };
  double       minDist { 0.0 };

  for (auto *object : scene_->getObjects()) {
    for (auto *face : object->getFaces()) {
      auto dist = face->distanceTo(p);

      if (! minFace || dist < minDist) {
        minFace = face;
        minDist = dist;
      }
    }
  }

  if (! minFace)
    return nullptr;

  return minFace->getObject();
}

CGeomFace3D *
App::
getNearestFace(CGeomObject3D *object, const CPoint3D &p) const
{
  CGeomFace3D *minFace { nullptr };
  double       minDist { 0.0 };

  for (auto *face : object->getFaces()) {
    auto dist = face->distanceToCenter(p);

    if (! minFace || dist < minDist) {
      minFace = face;
      minDist = dist;
    }
  }

  return minFace;
}

CGeomFace3D *
App::
getNamedFace(CGeomObject3D *object, const std::string &name) const
{
  for (auto *face : object->getFaces()) {
    if (face->name() == name)
      return face;
  }

  return nullptr;
}

CGeomEdge3D *
App::
getNearestEdge(CGeomObject3D *object, const CPoint3D &p) const
{
  CGeomEdge3D *minEdge { nullptr };
  double       minDist { 0.0 };

  for (auto *edge : object->getEdges()) {
    auto dist = edge->distanceTo(p);

    if (! minEdge || dist < minDist) {
      minEdge = edge;
      minDist = dist;
    }
  }

  return minEdge;
}

CGeomEdge3D *
App::
getNearestEdge(CGeomFace3D *face, const CPoint3D &p) const
{
  CGeomEdge3D *minEdge { nullptr };
  double       minDist { 0.0 };

  for (auto *edge : face->getEdges()) {
    auto dist = edge->distanceTo(p);

    if (! minEdge || dist < minDist) {
      minEdge = edge;
      minDist = dist;
    }
  }

  return minEdge;
}

CGeomVertex3D *
App::
getNearestVertex(CGeomObject3D *object, const CPoint3D &p) const
{
  CGeomVertex3D *minVertex { nullptr };
  double         minDist   { 0.0 };

  for (auto *vertex : object->getVertices()) {
    if (! vertex) continue;

    const auto &pv = vertex->getModel();

    auto dist = p.distanceTo(pv);

    if (! minVertex || dist < minDist) {
      minVertex = vertex;
      minDist   = dist;
    }
  }

  return minVertex;
}

CGeomVertex3D *
App::
getNearestVertex(CGeomFace3D *face, const CPoint3D &p) const
{
  CGeomVertex3D *minVertex { nullptr };
  double         minDist   { 0.0 };

  auto *object = face->getObject();

  for (const auto &vind : face->getVertices()) {
    auto *vertex = object->getVertexP(vind);

    const auto &pv = vertex->getModel();

    auto dist = p.distanceTo(pv);

    if (! minVertex || dist < minDist) {
      minVertex = vertex;
      minDist   = dist;
    }
  }

  return minVertex;
}

bool
App::
stringToBBox(const std::string &str, CBBox3D &bbox) const
{
  StringList strs;
  tcl_->splitList(str, strs);

  if (strs.size() != 6)
    return false;

  double x1, y1, z1, x2, y2, z2;
  if (! stringToReal(strs[0], x1) || ! stringToReal(strs[1], y1) || ! stringToReal(strs[2], z1) ||
      ! stringToReal(strs[3], x2) || ! stringToReal(strs[4], y2) || ! stringToReal(strs[5], z2))
    return false;

  bbox = CBBox3D(CPoint3D(x1, y1, z1), CPoint3D(x2, y2, z2));

  return true;
}

bool
App::
decodePoint(Tcl_Obj *obj, CPoint2D &p) const
{
  auto arg = CTclUtil::stringFromObj(obj);

  if (! stringToPoint(arg, p))
    return errorMsg("Invalid point '" + arg + "'");

  return true;
}

bool
App::
stringToPoint(const std::string &str, CPoint2D &p) const
{
  StringList strs;
  tcl_->splitList(str, strs);

  if (strs.size() != 2)
    return false;

  double x, y;
  if (! stringToReal(strs[0], x) ||
      ! stringToReal(strs[1], y))
    return false;

  p = CPoint2D(x, y);

  return true;
}

bool
App::
decodePoint(Tcl_Obj *obj, CPoint3D &p) const
{
  if (CTclVector3D::isObj(obj)) {
    CVector3D v;
    if (! objToVector(obj, v))
      return errorMsg("Invalid point '" + CTclUtil::stringFromObj(obj) + "'");

    p = v.point();

    return true;
  }

  auto arg = CTclUtil::stringFromObj(obj);

  if (! stringToPoint(arg, p))
    return errorMsg("Invalid point '" + arg + "'");

  return true;
}

bool
App::
stringToPoint(const std::string &str, CPoint3D &p) const
{
  StringList strs;
  tcl_->splitList(str, strs);

  if (strs.size() != 3)
    return false;

  double x, y, z;
  if (! stringToReal(strs[0], x) ||
      ! stringToReal(strs[1], y) ||
      ! stringToReal(strs[2], z))
    return false;

  p = CPoint3D(x, y, z);

  return true;
}

bool
App::
stringToVector(const std::string &str, CVector3D &v) const
{
#if 0
  StringList strs;
  tcl_->splitList(str, strs);

  if (strs.size() != 3)
    return false;

  double x, y, z;
  if (! stringToReal(strs[0], x) ||
      ! stringToReal(strs[1], y) ||
      ! stringToReal(strs[2], z))
    return false;

  v = CVector3D(x, y, z);

  return true;
#else
  return CTclVector3D::fromString(str, v);
#endif
}

bool
App::
objToVector(Tcl_Obj *obj, CVector3D &v) const
{
  if (! CTclVector3D::isObj(obj))
    return false;

  if (CTclVector3D::getObj(tcl_->interp(), obj, v) == TCL_ERROR)
    return false;

  return true;
}

bool
App::
stringToMatrix(const std::string &str, CMatrix3D &m) const
{
  return CTclMatrix3D::fromString(str, m);
}

bool
App::
objToMatrix(Tcl_Obj *obj, CMatrix3D &m) const
{
  if (! CTclMatrix3D::isObj(obj))
    return false;

  if (CTclMatrix3D::getObj(tcl_->interp(), obj, m) == TCL_ERROR)
    return false;

  return true;
}

bool
App::
stringToRect(const std::string &str, CBBox2D &r) const
{
  StringList strs;
  tcl_->splitList(str, strs);

  if (strs.size() != 4)
    return false;

  double x1, y1, x2, y2;
  if (! stringToReal(strs[0], x1) || ! stringToReal(strs[1], y1) ||
      ! stringToReal(strs[2], x2) || ! stringToReal(strs[3], y2))
    return false;

  r = CBBox2D(x1, y1, x2, y2);

  return true;
}

bool
App::
stringToColor(const std::string &str, CRGBA &c) const
{
  c = CRGBName::toRGBA(str);

  return true;
}

bool
App::
decodeReal(Tcl_Obj *obj, double &r) const
{
  auto arg = CTclUtil::stringFromObj(obj);

  if (! stringToReal(arg, r))
    return errorMsg("Invalid real '" + arg + "'");

  return true;
}

bool
App::
decodeBool(Tcl_Obj *obj, bool &b) const
{
  auto arg = CTclUtil::stringFromObj(obj);

  if (! stringToBool(arg, b))
    return errorMsg("Invalid bool '" + arg + "'");

  return true;
}

bool
App::
runTclCmd(const std::string &cmd)
{
  auto rc = tcl_->eval(cmd, /*showError*/true, /*showResult*/false);

  if (! rc)
    (void) errorMsg("Command '" + cmd + "' failed");

  return rc;
}

void
App::
setVar(const std::string &name, double value)
{
  tcl_->createVar(name, value);
}

void
App::
setVar(const std::string &name, const std::string &value)
{
  tcl_->createVar(name, value);
}

//---

void
App::
showMetaEdit()
{
  static CQMetaEdit *metaEdit;

  if (! metaEdit)
    metaEdit = new CQMetaEdit;

  metaEdit->show();

  metaEdit->raise();
}

void
App::
showPerfDialog()
{
#ifdef CQ_PERF_GRAPH
  auto *dialog = CQPerfDialog::instance();

  dialog->show();
#endif
}

void
App::
showAppOptions()
{
  CQAppOptions::show();
}

}
