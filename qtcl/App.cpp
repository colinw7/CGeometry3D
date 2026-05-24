#include <App.h>
#include <Canvas.h>
#include <Overview.h>
#include <Toolbar.h>
#include <Sidebar.h>
#include <Control.h>
#include <Status.h>
#include <GeomObject.h>
#include <Texture.h>

#include <CQTabSplit.h>
#include <CImportObj.h>
#include <CGeometry3D.h>
#include <CGeomScene3D.h>
#include <CGeomEdge3D.h>

#include <CGeomCube3D.h>
#include <CGeomCone3D.h>
#include <CGeomCylinder3D.h>
#include <CGeomSphere3D.h>
#include <CGeomPlane3D.h>

#include <QVBoxLayout>
#include <QTimer>

#include <CTclUtil.h>

#include <CRGBName.h>

#if 0
#include <CSolidNoise.h>
#endif

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
#include <svg/pause_svg.h>
#include <svg/play_one_svg.h>
#include <svg/play_svg.h>
#include <svg/rotate_svg.h>
#include <svg/scale_svg.h>
#include <svg/selected_svg.h>
#include <svg/settings_svg.h>
#include <svg/view_follow_svg.h>
#include <svg/visible_svg.h>
#endif

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

bool stringToBool(const std::string &str, bool &b) {
  std::string str1;
  for (auto &c : str)
    str1 += (isupper(c) ? tolower(c) : c);

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
  return "m:" + std::to_string(material->id());
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

std::string encodeObjectEdge(CGeomObject3D *object, CGeomEdge3D *edge) {
  return encodeObjectEdgeId(object->getInd(), edge->getInd());
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

int tclErrorMsg(const std::string &msg) {
  std::cerr << msg << "\n";
  return TCL_ERROR;
}

int errorMsg(const std::string &msg) {
  std::cerr << msg << "\n";
  return false;
}

CTcl::RealList pointToRealArray(const CPoint3D &p) {
  CTcl::RealList realList;

  realList.push_back(p.x);
  realList.push_back(p.y);
  realList.push_back(p.z);

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

CTCL_DCL_OBJECT_PROC(App, getObjectProperty, getObjectPropertyProc, this)
CTCL_DCL_OBJECT_PROC(App, setObjectProperty, setObjectPropertyProc, this)
CTCL_DCL_OBJECT_PROC(App, animateReal      , animateRealProc      , this)
CTCL_DCL_OBJECT_PROC(App, readModel        , readModelProc        , this)
CTCL_DCL_OBJECT_PROC(App, writeObj         , writeObjProc         , this)
CTCL_DCL_OBJECT_PROC(App, calcVector       , calcVectorProc       , this)

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

  timer_ = new QTimer;

  connect(timer_, SIGNAL(timeout()), this, SLOT(timerSlot()));

  //---

  initTcl();

  //---

  timer_->start(100);
}

bool
App::
loadModel(const QString &fileName, CGeom3DType format)
{
  static uint modeInd;

  auto modelName = QString("Model.%1").arg(++modeInd);

  auto *im = CImportBase::createModel(format, modelName.toStdString());

  if (! im) {
    std::cerr << "File format not recognised for '" << fileName.toStdString() << "'\n";
    return false;
  }

  CFile file(fileName.toStdString());

  if (! im->read(file)) {
    delete im;
    std::cerr << "Failed to read model for '" << fileName.toStdString() << "'\n";
    return false;
  }

  auto *scene = im->releaseScene();

  delete im;

  uint numTop = 0;

  for (auto *object : scene->getObjects()) {
    if (! object->parent())
      ++numTop;
  }

  std::vector<CGeomObject3D *> objects;

  if (numTop > 1) {
    auto *parentObj = CGeometry3DInst->createObject3D(scene_, modelName.toStdString());

    scene_->addObject(parentObj);

    //objects.push_back(parentObj);

    for (auto *object : scene->getObjects()) {
      scene_->addObject(object);

      if (! object->parent())
        parentObj->addChild(object);

      objects.push_back(object);
    }
  }
  else {
    for (auto *object : scene->getObjects()) {
      scene_->addObject(object);

      objects.push_back(object);
    }
  }

  for (auto *material : scene->getMaterials()) {
    scene_->addMaterial(material);
  }

  for (auto *texture : scene->textures()) {
    scene_->addTexture(texture);
  }

  Q_EMIT modelAdded();

  return true;
}

//---

void
App::
initTcl()
{
  tcl_ = new CTcl;

  tcl_->createAlias("echo", "puts");

  //---

  // add primitives
  tcl_->createObjCommand("addObject"  , addObjectProc  , this);
  tcl_->createObjCommand("addVertex"  , addVertexProc  , this);
  tcl_->createObjCommand("addFace"    , addFaceProc    , this);
  tcl_->createObjCommand("addMaterial", addMaterialProc, this);
  tcl_->createObjCommand("addTexture" , addTextureProc , this);

  // add shapes
  tcl_->createObjCommand("addPlane"   , addPlaneProc   , this);
  tcl_->createObjCommand("addCube"    , addCubeProc    , this);
  tcl_->createObjCommand("addCone"    , addConeProc    , this);
  tcl_->createObjCommand("addCylinder", addCylinderProc, this);
  tcl_->createObjCommand("addSphere"  , addSphereProc  , this);
//tcl_->createObjCommand("addTerrain" , addTerrainProc , this);

  // get/set primitive data
  tcl_->createObjCommand("getAppValue"     , getAppValueProc     , this);
  tcl_->createObjCommand("setAppValue"     , setAppValueProc     , this);
  tcl_->createObjCommand("getObjectValue"  , getObjectValueProc  , this);
  tcl_->createObjCommand("setObjectValue"  , setObjectValueProc  , this);
  tcl_->createObjCommand("getFaceValue"    , getFaceValueProc    , this);
  tcl_->createObjCommand("setFaceValue"    , setFaceValueProc    , this);
  tcl_->createObjCommand("getEdgeValue"    , getEdgeValueProc    , this);
  tcl_->createObjCommand("setEdgeValue"    , setEdgeValueProc    , this);
  tcl_->createObjCommand("getVertexValue"  , getVertexValueProc  , this);
  tcl_->createObjCommand("setVertexValue"  , setVertexValueProc  , this);
  tcl_->createObjCommand("setMaterialValue", setMaterialValueProc, this);

  CTCL_OBJECT_PROC(tcl_, getObjectProperty, App, this);
  CTCL_OBJECT_PROC(tcl_, setObjectProperty, App, this);

  // operate on primitives
  tcl_->createObjCommand("intersectObjects", intersectObjectsProc, this);
  tcl_->createObjCommand("inverseObject"   , inverseObjectProc   , this);
  tcl_->createObjCommand("unionObjects"    , unionObjectsProc    , this);
  tcl_->createObjCommand("subtractObjects" , subtractObjectsProc , this);

  tcl_->createObjCommand("extrudeFaces", extrudeFacesProc, this);
  tcl_->createObjCommand("extrudeEdges", extrudeEdgesProc, this);
  tcl_->createObjCommand("mergeEdge"   , mergeEdgeProc   , this);
  tcl_->createObjCommand("separateFace", separateFaceProc, this);
  tcl_->createObjCommand("separateEdge", separateEdgeProc, this);
  tcl_->createObjCommand("mirrorObject", mirrorObjectProc, this);
  tcl_->createObjCommand("fillVertices", fillVerticesProc, this);

  tcl_->createObjCommand("deleteObjects" , deleteObjectsProc , this);
  tcl_->createObjCommand("deleteFaces"   , deleteFacesProc   , this);
  tcl_->createObjCommand("deleteVertices", deleteVerticesProc, this);

  // animate
  CTCL_OBJECT_PROC(tcl_, animateReal, App, this);

  // import/export
  CTCL_OBJECT_PROC(tcl_, readModel, App, this)
  CTCL_OBJECT_PROC(tcl_, writeObj , App, this)

  // util
  CTCL_OBJECT_PROC(tcl_, calcVector, App, this);

  //---

  runTclCmd("proc keyPress { args } { }");

  runTclCmd("proc selectionProc { args } { }");
}

int
App::
execFile(const std::string &filename)
{
  std::string res;
  return tcl_->eval("source \"" + filename + "\"", res, /*showError*/true);
}

int
App::
addObjectProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "addObjectProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 0)
    return tclErrorMsg("Invalid args");

  auto name = "object." + std::to_string(app->scene_->getObjects().size() + 1);

  auto *object = CGeometry3DInst->createObject3D(app->scene_, name);

  app->scene_->addObject(object);

  app->tcl_->setResult(encodeObject(object));

  app->canvas()->updateScene(/*updateBBox*/false);

  return TCL_OK;
}

int
App::
addVertexProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "addVertexProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 2)
    return tclErrorMsg("Invalid args");

  CGeomObject3D *object;
  if (! app->decodeObject(args[0], object))
    return tclErrorMsg("Invalid object id '" + args[0] + "'");

  CPoint3D p;
  if (! app->stringToPoint(args[1], p))
    return tclErrorMsg("Invalid point '" + args[1] + "'");

  auto vind = object->addVertex(p);

  app->tcl_->setResult(encodeObjectVertexId(object, vind));

  app->canvas()->updateScene(/*updateBBox*/false);

  return TCL_OK;
}

int
App::
addFaceProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "addFaceProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 2)
    return tclErrorMsg("Invalid args");

  CGeomObject3D *object;
  if (! app->decodeObject(args[0], object))
    return tclErrorMsg("Invalid object id '" + args[0] + "'");

  StringList strs;
  app->tcl_->splitList(args[1], strs);

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

  app->tcl_->setResult(encodeObjectFaceId(object->getInd(), faceId));

  app->canvas()->updateScene(/*updateBBox*/false);

  return TCL_OK;
}

int
App::
addLineProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "addFaceProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 2)
    return tclErrorMsg("Invalid args");

  CGeomObject3D *object;
  if (! app->decodeObject(args[0], object))
    return tclErrorMsg("Invalid object id '" + args[0] + "'");

  StringList strs;
  app->tcl_->splitList(args[1], strs);

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

  app->tcl_->setResult(encodeObjectLineId(object->getInd(), lineId));

  app->canvas()->updateScene(/*updateBBox*/false);

  return TCL_OK;
}

int
App::
addMaterialProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "addMaterialProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  auto name = args[0];

  auto *material = CGeometry3DInst->createMaterial();

  material->setName(name);

  app->scene_->addMaterial(material);

  app->tcl_->setResult(encodeMaterial(material));

  return TCL_OK;
}

int
App::
addTextureProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "addTextureProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  auto name = args[0];

  auto *texture = CGeometry3DInst->createTexture(name);
  if (! texture)
    return tclErrorMsg("Invalid texture");

  texture->setFilename(name);

  app->scene_->addTexture(texture);

  app->tcl_->setResult(encodeTexture(texture));

  return TCL_OK;
}

int
App::
addPlaneProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "addPlaneProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);

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

  auto c = app->cursor();

  auto n = app->scene_->getObjects().size();
  auto name = "plane." + std::to_string(n + 1);

  //auto *plane = new CGeomPlane3D(app->scene_, name, c, w, h);
  auto *plane = CGeometry3DInst->createObject3D(app->scene_, name);

  CGeomPlane3D::addGeometry(plane, c, w, h);

  plane->setInd(CGeometry3DInst->nextObjectId());

  app->scene_->addObject(plane);

  app->tcl_->setResult(encodeObject(plane));

  return TCL_OK;
}

int
App::
addCubeProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "addCubeProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);

  double r = 1.0;

  if      (args.size() == 1) {
    if (! stringToReal(args[0], r))
      return tclErrorMsg("Invalid args");
  }
  else if (! args.empty())
    return tclErrorMsg("Invalid args");

  auto c = app->cursor();

  auto n = app->scene_->getObjects().size();
  auto name = "cube." + std::to_string(n + 1);

//auto *cube = new CGeomCube3D(app->scene_, name, c, r);
  auto *cube = CGeometry3DInst->createObject3D(app->scene_, name);

  CGeomCube3D::addGeometry(cube, c, r);

  cube->setInd(CGeometry3DInst->nextObjectId());

  app->scene_->addObject(cube);

  app->tcl_->setResult(encodeObject(cube));

  return TCL_OK;
}

int
App::
addConeProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "addConeProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);

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

  auto c = app->cursor();

  auto n = app->scene_->getObjects().size();
  auto name = "cone." + std::to_string(n + 1);

  //auto *cone = new CGeomCone3D(app->scene_, name, c, w, h);
  auto *cone = CGeometry3DInst->createObject3D(app->scene_, name);

  CGeomCone3D::addGeometry(cone, c, w, h);

  cone->setInd(CGeometry3DInst->nextObjectId());

  app->scene_->addObject(cone);

  app->tcl_->setResult(encodeObject(cone));

  return TCL_OK;
}

int
App::
addCylinderProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "addCylinderProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);

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

  auto c = app->cursor();

  auto n = app->scene_->getObjects().size();
  auto name = "cylinder." + std::to_string(n + 1);

  //auto *cylinder = new CGeomCylinder3D(app->scene_, name, c, w, h);
  auto *cylinder = CGeometry3DInst->createObject3D(app->scene_, name);

  CGeomCylinder3D::addGeometry(cylinder, c, w, h);

  cylinder->setInd(CGeometry3DInst->nextObjectId());

  app->scene_->addObject(cylinder);

  app->tcl_->setResult(encodeObject(cylinder));

  return TCL_OK;
}

int
App::
addSphereProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "addSphereProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);

  double r = 1.0;

  if      (args.size() == 1) {
    if (! stringToReal(args[0], r))
      return tclErrorMsg("Invalid args");
  }
  else if (! args.empty())
    return tclErrorMsg("Invalid args");

  auto c = app->cursor();

  auto n = app->scene_->getObjects().size();
  auto name = "sphere." + std::to_string(n + 1);

  //auto *sphere = new CGeomSphere3D(app->scene_, name, c, r);
  auto *sphere = CGeometry3DInst->createObject3D(app->scene_, name);

  CGeomSphere3D::addGeometry(sphere, c, r);

  sphere->setInd(CGeometry3DInst->nextObjectId());

  CGeomSphere3D::addTexturePoints(sphere);
  CGeomSphere3D::addNormals(sphere, 1.0);

  app->scene_->addObject(sphere);

  app->tcl_->setResult(encodeObject(sphere));

  return TCL_OK;
}

#if 0
int
App::
addTerrainProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "addTerrainProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
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

  auto *object = CGeometry3DInst->createObject3D(app->scene_, name);

  app->scene_->addObject(object);

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
    std::vector<uint> vertices;

    vertices.push_back(addPoint(p1.p, p1.c, p1.n, p1.tp));
    vertices.push_back(addPoint(p2.p, p2.c, p2.n, p2.tp));

    vertices.push_back(addPoint(p3.p, p3.c, p3.n, p3.tp));
    vertices.push_back(addPoint(p4.p, p4.c, p4.n, p4.tp));

    object->addFace(vertices);
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

      auto n1 = diff3.crossProduct(diff1).normalized();
      auto n2 = diff1.crossProduct(diff4).normalized();
      auto n3 = diff2.crossProduct(diff3).normalized();
      auto n4 = diff4.crossProduct(diff2).normalized();

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

  app->tcl_->setResult(encodeObject(object));

  return TCL_OK;
}
#endif

int
App::
getAppValueProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "getAppValueProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() < 1)
    return tclErrorMsg("Invalid args");

  auto name = args[0];

  if      (name == "cursor") {
    auto p = app->cursor();

    app->tcl_->setResult(pointToRealArray(p));
  }
  else if (name == "objects") {
    StringList objectIds;

    for (auto *object : app->scene_->getObjects()) {
      auto objectId = encodeObject(object);

      objectIds.push_back(objectId);
    }

    app->tcl_->setResult(objectIds);
  }
  else if (name == "nearest_object") {
    if (args.size() < 2)
      return tclErrorMsg("Invalid args");

    CPoint3D p;
    if (! app->stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    auto *object = app->getNearestObject(p);

    app->tcl_->setResult(encodeObject(object));
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setAppValueProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "setAppValueProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  auto name = args[0];

  if      (name == "cursor") {
    CPoint3D p;
    if (! app->stringToPoint(args[1], p))
      return tclErrorMsg("Invalid point '" + args[1] + "'");

    app->setCursor(p);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
getObjectValueProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "getObjectValueProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CGeomObject3D *object;
  if (! app->decodeObject(args[0], object))
    return tclErrorMsg("Invalid object id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "faces") {
    const auto &faces = object->getFaces();

    StringList faceIds1;

    for (auto *face : faces) {
      auto faceId1 = encodeFace(face);

      faceIds1.push_back(faceId1);
    }

    app->tcl_->setResult(faceIds1);
  }
  else if (name == "edges") {
    const auto &edges = object->getEdges();

    StringList edgeIds1;

    for (auto *edge : edges) {
      auto edgeId1 = encodeObjectEdge(object, edge);

      edgeIds1.push_back(edgeId1);
    }

    app->tcl_->setResult(edgeIds1);
  }
  else if (name == "vertices") {
    const auto &vertices = object->getVertices();

    StringList vertices1;

    for (auto *vertex : vertices) {
      if (! vertex) continue;

      auto vertexId1 = encodeVertex(vertex);

      vertices1.push_back(vertexId1);
    }

    app->tcl_->setResult(vertices1);
  }
  else if (name == "nearest_face") {
    if (args.size() < 2)
      return tclErrorMsg("Invalid args");

    CPoint3D p;
    if (! app->stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    auto *face = app->getNearestFace(object, p);

    app->tcl_->setResult(encodeFace(face));
  }
  else if (name == "named_face") {
    if (args.size() < 2)
      return tclErrorMsg("Invalid args");

    auto *face = app->getNamedFace(object, args[2]);

    app->tcl_->setResult(encodeFace(face));
  }
  else if (name == "nearest_edge") {
    if (args.size() < 2)
      return tclErrorMsg("Invalid args");

    CPoint3D p;
    if (! app->stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    auto *edge = app->getNearestEdge(object, p);

    app->tcl_->setResult(encodeObjectEdge(object, edge));
  }
  else if (name == "nearest_vertex") {
    if (args.size() < 2)
      return tclErrorMsg("Invalid args");

    CPoint3D p;
    if (! app->stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    auto *vertex = app->getNearestVertex(object, p);

    app->tcl_->setResult(encodeVertex(vertex));
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

    app->tcl_->setResult(faceIds1);
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

    app->tcl_->setResult(vertexIds1);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setObjectValueProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "setObjectValueProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() < 3)
    return tclErrorMsg("Invalid args");

  CGeomObject3D *object;
  if (! app->decodeObject(args[0], object))
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
  else if (name == "material") {
    int materialId;
    if (! decodeMaterialId(args[2], materialId))
      return tclErrorMsg("Invalid material id '" + args[2] + "'");

    auto *material = app->scene_->getMaterialById(materialId);
    if (! material)
      return tclErrorMsg("Invalid material id " + std::to_string(materialId));

    object->setMaterialP(material);
  }
  else if (name == "translate") {
    StringList strs;
    app->tcl_->splitList(args[2], strs);

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

    object->translate(tx, ty, tz);
  }
  else if (name == "scale") {
    StringList strs;
    app->tcl_->splitList(args[2], strs);

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

    object->scale(sx, sy, sz);
  }
  else if (name == "rotate") {
    if (args.size() < 4)
      return tclErrorMsg("Invalid rotate value");

    CVector3D v;
    if (! app->stringToVector(args[2], v))
      return tclErrorMsg("Invalid rotate vector '" + args[2] + "'");

    double a;
    if (! stringToReal(args[3], a))
      return tclErrorMsg("Invalid rotate angle '" + args[3] + "'");

    object->rotateModel(degToRad(a), v);

    app->canvas()->updateScene(/*updateBBox*/false);
  }
  else if (name == "rotate_xyz") {
    StringList strs;
    app->tcl_->splitList(args[2], strs);

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
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  app->tcl_->setResult(encodeObject(object));

  return TCL_OK;
}

int
App::
getFaceValueProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "getFaceValueProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CGeomFace3D *face;
  if (! app->decodeFace(args[0], face))
    return tclErrorMsg("Invalid face id '" + args[0] + "'");

  auto *object = face->getObject();

  auto name = args[1];

  if      (name == "ind") {
    app->tcl_->setResult(int(face->getInd()));
  }
  else if (name == "name") {
    app->tcl_->setResult(face->name());
  }
  else if (name == "color") {
    auto color = face->getColor();

    app->tcl_->setResult(color.stringEncode());
  }
  else if (name == "center") {
    auto center = face->calcModelCenter();

    app->tcl_->setResult(pointToRealArray(center));
  }
  else if (name == "normal") {
    CVector3D normal;
    face->calcModelNormal(normal);

    app->tcl_->setResult(pointToRealArray(normal.point()));
  }
  else if (name == "edge_vector") {
    if (args.size() < 3)
      return tclErrorMsg("Invalid args");

    CGeomEdge3D *edge;
    if (! app->decodeEdge(args[2], edge))
      return tclErrorMsg("Invalid edge id '" + args[2] + "'");

    auto v = face->edgeVector(edge);

    app->tcl_->setResult(pointToRealArray(v.point()));
  }
  else if (name == "vertices") {
    const auto &vertexIds = face->getVertices();

    StringList vertices1;

    for (const auto &vertexId : vertexIds) {
      auto vertexId1 = encodeObjectVertexId(object, vertexId);

      vertices1.push_back(vertexId1);
    }

    app->tcl_->setResult(vertices1);
  }
  else if (name == "nearest_vertex") {
    if (args.size() < 2)
      return tclErrorMsg("Invalid args");

    CPoint3D p;
    if (! app->stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    auto *vertex = app->getNearestVertex(face, p);

    app->tcl_->setResult(encodeVertex(vertex));
  }
  else if (name == "edges") {
    const auto &edges = face->getEdges();

    StringList edges1;

    for (const auto &edge : edges) {
      auto edgeId1 = encodeObjectEdge(object, edge);

      edges1.push_back(edgeId1);
    }

    app->tcl_->setResult(edges1);
  }
  else if (name == "nearest_edge") {
    if (args.size() < 2)
      return tclErrorMsg("Invalid args");

    CPoint3D p;
    if (! app->stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    auto *edge = app->getNearestEdge(face, p);

    app->tcl_->setResult(encodeObjectEdge(object, edge));
  }
  else if (name == "bbox") {
    CBBox3D bbox;
    face->getModelBBox(bbox);

    app->tcl_->setResult(bboxToRealArrays(bbox));
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setFaceValueProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "setFaceValueProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() < 3)
    return tclErrorMsg("Invalid args");

  CGeomFace3D *face;
  if (! app->decodeFace(args[0], face))
    return tclErrorMsg("Invalid face id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "selected") {
    bool selected;
    if (! stringToBool(args[2], selected))
      return tclErrorMsg("Invalid bool");

    face->setSelected(selected);

    app->tcl_->setResult(encodeFace(face));
  }
  else if (name == "color") {
    if (args.size() != 3)
      return tclErrorMsg("Invalid args");

    face->setColor(CRGBName::toRGBA(args[2]));

    app->tcl_->setResult(encodeFace(face));
  }
  else if (name == "normal") {
    if (args.size() != 3)
      return tclErrorMsg("Invalid args");

    CPoint3D p;
    if (! app->stringToPoint(args[2], p))
      return tclErrorMsg("Invalid point '" + args[2] + "'");

    face->setNormal(CVector3D(p));

    app->tcl_->setResult(encodeFace(face));
  }
  else if (name == "material") {
    int materialId;
    if (! decodeMaterialId(args[2], materialId))
      return tclErrorMsg("Invalid material id '" + args[2] + "'");

    auto *material = app->scene_->getMaterialById(materialId);
    if (! material)
      return tclErrorMsg("Invalid material id " + std::to_string(materialId));

    face->setMaterialP(material);

    app->tcl_->setResult(encodeFace(face));
  }
  else if (name == "translate") {
    StringList strs;
    app->tcl_->splitList(args[2], strs);

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

    app->tcl_->setResult(encodeFace(face));
  }
  else if (name == "scale") {
    StringList strs;
    app->tcl_->splitList(args[2], strs);

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

    app->tcl_->setResult(encodeFace(face));
  }
  else if (name == "center_scale") {
    if (args.size() < 4)
      return tclErrorMsg("Invalid args");

    CVector3D c;
    if (! app->stringToVector(args[2], c))
      return tclErrorMsg("Invalid center '" + args[2] + "'");

    StringList strs;
    app->tcl_->splitList(args[3], strs);

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

    app->tcl_->setResult(encodeFace(face));
  }
  else if (name == "rotate_xyz") {
    StringList strs;
    app->tcl_->splitList(args[2], strs);

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

    app->tcl_->setResult(encodeFace(face));
  }
  else if (name == "bevel") {
    double d;
    if (! stringToReal(args[2], d))
      return tclErrorMsg("Invalid bevel '" + args[2] + "'");

    face->bevel(d);

    app->tcl_->setResult(encodeFace(face));
  }
  else if (name == "inset") {
    double d;
    if (! stringToReal(args[2], d))
      return tclErrorMsg("Invalid inset '" + args[2] + "'");

    face->inset(d);

    app->tcl_->setResult(encodeFace(face));
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
getEdgeValueProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "getEdgeValueProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CGeomEdge3D *edge;
  if (! app->decodeEdge(args[0], edge))
    return tclErrorMsg("Invalid edge id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "start") {
    auto start = edge->getStart();

    app->tcl_->setResult(encodeObjectVertexId(edge->getObject(), start));
  }
  else if (name == "end") {
    auto end = edge->getEnd();

    app->tcl_->setResult(encodeObjectVertexId(edge->getObject(), end));
  }
  else if (name == "vertices") {
    const auto &vertexIds = edge->getVertices();

    StringList vertices1;

    for (const auto &vertexId : vertexIds) {
      auto vertexId1 = encodeObjectVertexId(edge->getObject(), vertexId);

      vertices1.push_back(vertexId1);
    }

    app->tcl_->setResult(vertices1);
  }
  else if (name == "normal") {
    auto normal = edge->calcNormal();

    app->tcl_->setResult(pointToRealArray(normal.point()));
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setEdgeValueProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "setEdgeValueProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CGeomEdge3D *edge;
  if (! app->decodeEdge(args[0], edge))
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
    app->tcl_->splitList(args[2], strs);

    CVector3D d;
    if (! app->stringToVector(args[2], d))
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

    app->tcl_->setResult(ids);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
getLineValueProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "getLineValueProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CGeomLine3D *line;
  if (! app->decodeLine(args[0], line))
    return tclErrorMsg("Invalid line id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "start") {
    auto start = line->getStartInd();

    app->tcl_->setResult(encodeObjectVertexId(line->getObject(), start));
  }
  else if (name == "end") {
    auto end = line->getEndInd();

    app->tcl_->setResult(encodeObjectVertexId(line->getObject(), end));
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setLineValueProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "setLineValueProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CGeomLine3D *line;
  if (! app->decodeLine(args[0], line))
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

    line->setColor(CRGBName::toRGBA(args[2]));
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
getVertexValueProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "getVertexValueProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() < 2)
    return tclErrorMsg("Invalid args");

  CGeomVertex3D *vertex;
  if (! app->decodeVertex(args[0], vertex))
    return tclErrorMsg("Invalid vertex id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "model") {
    auto p = vertex->getModel();

    app->tcl_->setResult(pointToRealArray(p));
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setVertexValueProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "setVertexValueProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() < 3)
    return tclErrorMsg("Invalid args");

  CGeomVertex3D *vertex;
  if (! app->decodeVertex(args[0], vertex))
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
    if (! app->stringToPoint(args[2], p))
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
setMaterialValueProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "setMaterialValueProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() < 3)
    return tclErrorMsg("Invalid args");

  int materialId;
  if (! decodeMaterialId(args[0], materialId))
    return tclErrorMsg("Invalid material id '" + args[2] + "'");

  auto *material = app->scene_->getMaterialById(materialId);
  if (! material)
    return tclErrorMsg("Invalid material id " + std::to_string(materialId));

  auto name = args[1];

  if      (name == "diffuse") {
    if (args.size() != 3)
      return tclErrorMsg("Invalid args");

    material->setDiffuse(CRGBName::toRGBA(args[2]));
  }
  else if (name == "diffuse_texture") {
    int textureId;
    if (! decodeTextureId(args[2], textureId))
      return tclErrorMsg("Invalid texture id '" + args[2] + "'");

    auto *texture = app->scene_->getTextureById(textureId);
    if (! texture)
      return tclErrorMsg("Invalid texture id " + std::to_string(textureId));

    material->setDiffuseTexture(texture);
  }
  else
    return tclErrorMsg("Invalid value name '" + name + "'");

  app->tcl_->setResult(encodeMaterial(material));

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
intersectObjectsProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "intersectObjects\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  StringList strs;
  app->tcl_->splitList(args[0], strs);

  std::vector<CGeomObject3D *> objects;

  for (const auto &str : strs) {
    CGeomObject3D *object;
    if (! app->decodeObject(str, object))
      return tclErrorMsg("Invalid object id '" + str + "'");

    objects.push_back(object);
  }

  auto *object = app->scene_->intersectObjects(objects);

  app->scene_->addObject(object);

  app->tcl_->setResult(encodeObject(object));

  return TCL_OK;
}

int
App::
inverseObjectProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "inverseObject\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  CGeomObject3D *object;
  if (! app->decodeObject(args[0], object))
    return tclErrorMsg("Invalid object id '" + args[0] + "'");

  auto *object1 = app->scene_->inverseObject(object);

  app->scene_->addObject(object1);

  app->tcl_->setResult(encodeObject(object1));

  return TCL_OK;
}

int
App::
unionObjectsProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "unionObjectsProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  StringList strs;
  app->tcl_->splitList(args[0], strs);

  std::vector<CGeomObject3D *> objects;

  for (const auto &str : strs) {
    CGeomObject3D *object;
    if (! app->decodeObject(str, object))
      return tclErrorMsg("Invalid object id '" + str + "'");

    objects.push_back(object);
  }

  auto *object = app->scene_->unionObjects(objects);

  app->scene_->addObject(object);

  app->tcl_->setResult(encodeObject(object));

  return TCL_OK;
}

int
App::
subtractObjectsProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "subtractObjectsProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  StringList strs;
  app->tcl_->splitList(args[0], strs);

  std::vector<CGeomObject3D *> objects;

  for (const auto &str : strs) {
    CGeomObject3D *object;
    if (! app->decodeObject(str, object))
      return tclErrorMsg("Invalid object id '" + str + "'");

    objects.push_back(object);
  }

  auto *object = app->scene_->subtractObjects(objects);

  app->scene_->addObject(object);

  app->tcl_->setResult(encodeObject(object));

  return TCL_OK;
}

int
App::
extrudeFacesProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "extrudeFacesProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 2)
    return tclErrorMsg("Invalid args");

  StringList fstrs;
  app->tcl_->splitList(args[0], fstrs);

  if (fstrs.size() < 1)
    return tclErrorMsg("Invalid args");

  std::vector<CGeomFace3D *> faces;

  if (! app->decodeFaces(fstrs, faces))
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

    app->tcl_->setResult(faceIds);
  }

  return TCL_OK;
}

int
App::
extrudeEdgesProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "extrudeEdgesProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 2)
    return tclErrorMsg("Invalid args");

  CGeomEdge3D *edge;
  if (! app->decodeEdge(args[0], edge))
    return tclErrorMsg("Invalid edge id '" + args[0] + "'");

  double d;
  if (! stringToReal(args[1], d))
    return tclErrorMsg("Invalid delta '" + args[1] + "'");

  auto *face1 = edge->extrude(d);

  auto faceId1 = encodeFace(face1);

  app->tcl_->setResult(faceId1);

  return TCL_OK;
}

int
App::
mergeEdgeProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "mergeVerticesProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  CGeomEdge3D *edge;
  if (! app->decodeEdge(args[0], edge))
    return tclErrorMsg("Invalid edge id '" + args[0] + "'");

  auto vind = edge->getObject()->mergeEdge(edge->getInd());

  auto vertexId = encodeObjectVertexId(edge->getObject(), vind);

  app->tcl_->setResult(vertexId);

  return TCL_OK;
}

int
App::
duplicateFaceProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "duplicateFaceProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  CGeomFace3D *face;
  if (! app->decodeFace(args[0], face))
    return tclErrorMsg("Invalid face id '" + args[0] + "'");

  auto *face1 = face->duplicate();

  face->getObject()->addFace(face1);

  app->tcl_->setResult(encodeFace(face1));

  return TCL_OK;
}

int
App::
separateFaceProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "separateFaceProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  CGeomFace3D *face;
  if (! app->decodeFace(args[0], face))
    return tclErrorMsg("Invalid face id '" + args[0] + "'");

  auto *object1 = face->getObject()->separateFace(face);

  app->tcl_->setResult(encodeObject(object1));

  return TCL_OK;
}

int
App::
separateEdgeProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "separateEdgeProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  CGeomEdge3D *edge;
  if (! app->decodeEdge(args[0], edge))
    return tclErrorMsg("Invalid edge id '" + args[0] + "'");

  auto *object1 = edge->getObject()->separateEdge(edge);

  app->tcl_->setResult(encodeObject(object1));

  return TCL_OK;
}

int
App::
mirrorObjectProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "mirrorObjectProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 2)
    return tclErrorMsg("Invalid args");

  CGeomObject3D *object;
  if (! app->decodeObject(args[0], object))
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

  auto c = app->cursor();

  const auto &objects = object->mirror(CGeomObject3D::MirrorDir(mirrorDir), c);

  StringList objectIds1;

  for (auto *object1 : objects) {
    auto name = "object." + std::to_string(app->scene_->getObjects().size() + 1);

    app->scene_->addObject(object1);

    object1->setName(name);

    object1->setInd(CGeometry3DInst->nextObjectId());

    auto objectId1 = encodeObject(object1);

    objectIds1.push_back(objectId1);
  }

  app->tcl_->setResult(objectIds1);

  return TCL_OK;
}

int
App::
fillVerticesProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "fillVerticesProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  std::vector<CGeomVertex3D *> vertices;

  if (! app->decodeVertices(args[0], vertices))
    return TCL_ERROR;

  auto *object = vertices[0]->getObject();

  object->fillVertices(vertices);

  app->tcl_->setResult(0);

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
deleteObjectsProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "deleteObjectsProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  StringList strs;
  app->tcl_->splitList(args[0], strs);

  std::vector<CGeomObject3D *> objects;

  for (const auto &str : strs) {
    CGeomObject3D *object;
    if (! app->decodeObject(str, object))
      return tclErrorMsg("Invalid object id '" + str + "'");

    objects.push_back(object);
  }

  for (auto *object : objects) {
    app->scene_->removeObject(object, /*force*/true);

    delete object;
  }

  app->tcl_->setResult(0);

  return TCL_OK;
}

int
App::
deleteFacesProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "deleteFacesProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  StringList strs;
  app->tcl_->splitList(args[0], strs);

  std::vector<CGeomFace3D *> faces;

  if (! app->decodeFaces(strs, faces))
    return TCL_ERROR;

  auto *object = faces[0]->getObject();

  for (auto *face : faces) {
    object->removeFace(face);

    delete face;
  }

  app->tcl_->setResult(0);

  return TCL_OK;
}

int
App::
deleteVerticesProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "deleteOVerticesroc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 1)
    return tclErrorMsg("Invalid args");

  StringList strs;
  app->tcl_->splitList(args[0], strs);

  std::vector<CGeomVertex3D *> vertices;

  for (const auto &str : strs) {
    CGeomVertex3D *vertex;
    if (! app->decodeVertex(str, vertex))
      return tclErrorMsg("Invalid object id '" + str + "'");

    vertices.push_back(vertex);
  }

  auto *object = vertices[0]->getObject();

  for (auto *vertex : vertices) {
    object->removeVertex(vertex);

    delete vertex;
  }

  app->tcl_->setResult(0);

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

  uint           numTop = 0;
  CGeomObject3D *topObj = nullptr;

  for (auto *object : scene->getObjects()) {
    if (! object->parent()) {
      ++numTop;
      topObj = object;
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

    topObj = parentObj;
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

  tcl_->setResult(encodeObject(topObj));

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

void
App::
timerSlot()
{
  for (auto *animData : animDatas_) {
    if (animData->isActive())
      animData->tick();
  }
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

}
