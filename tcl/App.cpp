#include <App.h>

#include <CTclUtil.h>
#include <CGeometry3D.h>
#include <CGeomEdge3D.h>

#include <CGeomCube3D.h>
#include <CGeomCone3D.h>
#include <CGeomCylinder3D.h>
#include <CGeomSphere3D.h>
#include <CGeomPlane3D.h>

#include <CImportObj.h>
#include <CRGBName.h>

#include <CSolidNoise.h>

namespace CTclGeometry3D {

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

std::string encodeObjectVertex(const CGeomVertex3D *vertex) {
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

std::string encodeObjectFace(CGeomFace3D *face) {
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

int errorMsg(const std::string &msg) {
  std::cerr << msg << "\n";
  return TCL_ERROR;
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

namespace CTclGeometry3D {

CTCL_DCL_OBJECT_PROC(App, readObj, readObjProc, this)

App::
App()
{
  scene_ = CGeometry3DInst->createScene3D();

  //---

  initTcl();
}

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
  tcl_->createObjCommand("addTerrain" , addTerrainProc , this);

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
  tcl_->createObjCommand("setMaterialValue", setMaterialValueProc, this);

  // operate on primitives
  tcl_->createObjCommand("intersectObjects", intersectObjectsProc, this);
  tcl_->createObjCommand("inverseObject"   , inverseObjectProc   , this);
  tcl_->createObjCommand("unionObjects"    , unionObjectsProc    , this);
  tcl_->createObjCommand("subtractObjects" , subtractObjectsProc , this);

  tcl_->createObjCommand("extrudeFace" , extrudeFaceProc , this);
  tcl_->createObjCommand("extrudeEdge" , extrudeEdgeProc , this);
  tcl_->createObjCommand("mergeEdge"   , mergeEdgeProc   , this);
  tcl_->createObjCommand("separateFace", separateFaceProc, this);
  tcl_->createObjCommand("separateEdge", separateEdgeProc, this);
  tcl_->createObjCommand("mirrorObject", mirrorObjectProc, this);

  tcl_->createObjCommand("deleteObjects", deleteObjectsProc, this);

  // import/export
//tcl_->createObjCommand("readObj" , readObjProc , this);
  tcl_->createObjCommand("writeObj", writeObjProc, this);

  CTCL_OBJECT_PROC(tcl_, readObj, App, this)
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
    return errorMsg("Invalid args");

  auto name = "object." + std::to_string(app->scene_->getObjects().size() + 1);

  auto *object = CGeometry3DInst->createObject3D(app->scene_, name);

  app->scene_->addObject(object);

  app->tcl_->setResult(encodeObject(object));

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
    return errorMsg("Invalid args");

  CGeomObject3D *object;
  if (! app->decodeObject(args[0], object))
    return errorMsg("Invalid object id '" + args[0] + "'");

  CPoint3D p;
  if (! app->stringToPoint(args[1], p))
    return errorMsg("Invalid point '" + args[1] + "'");

  auto vind = object->addVertex(p);

  app->tcl_->setResult(encodeObjectVertexId(object, vind));

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
    return errorMsg("Invalid args");

  CGeomObject3D *object;
  if (! app->decodeObject(args[0], object))
    return errorMsg("Invalid object id '" + args[0] + "'");

  std::vector<std::string> strs;
  app->tcl_->splitList(args[1], strs);

  std::vector<uint> vertices;

  for (const auto &str : strs) {
    int objId1, vertexId;
    if (! decodeObjectVertexId(str, objId1, vertexId) || objId1 != int(object->getInd()))
      return errorMsg("Invalid vertex '" + str + "'");

    vertices.push_back(vertexId);
  }

  if (vertices.size() < 3) {
    std::cerr << "Invalid number of faces\n";
    return TCL_ERROR;
  }

  auto faceId = object->addFace(vertices);

  app->tcl_->setResult(encodeObjectFaceId(object->getInd(), faceId));

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
    return errorMsg("Invalid args");

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
    return errorMsg("Invalid args");

  auto name = args[0];

  auto *texture = CGeometry3DInst->createTexture(name);

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
      return errorMsg("Invalid args");
  }
  else if (args.size() == 1) {
    if (! stringToReal(args[0], w))
      return errorMsg("Invalid args");

    h = w;
  }
  else if (! args.empty())
    return errorMsg("Invalid args");

  auto c = app->cursor();

  auto n = app->scene_->getObjects().size();
  auto name = "plane." + std::to_string(n + 1);

  auto *plane = new CGeomPlane3D(app->scene_, name, c, w, h);

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
      return errorMsg("Invalid args");
  }
  else if (! args.empty())
    return errorMsg("Invalid args");

  auto c = app->cursor();

  auto n = app->scene_->getObjects().size();
  auto name = "cube." + std::to_string(n + 1);

  auto *cube = new CGeomCube3D(app->scene_, name, c, r);

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
      return errorMsg("Invalid args");
  }
  else if (args.size() == 1) {
    if (! stringToReal(args[0], w))
      return errorMsg("Invalid args");

    h = w;
  }
  else if (! args.empty())
    return errorMsg("Invalid args");

  auto c = app->cursor();

  auto n = app->scene_->getObjects().size();
  auto name = "cone." + std::to_string(n + 1);

  auto *cone = new CGeomCone3D(app->scene_, name, c, w, h);

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
      return errorMsg("Invalid args");
  }
  else if (args.size() == 1) {
    if (! stringToReal(args[0], w))
      return errorMsg("Invalid args");

    h = w;
  }
  else if (! args.empty())
    return errorMsg("Invalid args");

  auto c = app->cursor();

  auto n = app->scene_->getObjects().size();
  auto name = "cylinder." + std::to_string(n + 1);

  auto *cylinder = new CGeomCylinder3D(app->scene_, name, c, w, h);

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
      return errorMsg("Invalid args");
  }
  else if (! args.empty())
    return errorMsg("Invalid args");

  auto c = app->cursor();

  auto n = app->scene_->getObjects().size();
  auto name = "sphere." + std::to_string(n + 1);

  auto *sphere = new CGeomSphere3D(app->scene_, name, c, r);

  sphere->setInd(CGeometry3DInst->nextObjectId());

  CGeomSphere3D::addTexturePoints(sphere);
  CGeomSphere3D::addNormals(sphere, 1.0);

  app->scene_->addObject(sphere);

  app->tcl_->setResult(encodeObject(sphere));

  return TCL_OK;
}

int
App::
addTerrainProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "addTerrainProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 3)
    return errorMsg("Invalid args");

  double width  = 1.0;
  double height = 1.0;
  double depth  = 1.0;

  if (! stringToReal(args[0], width) ||
      ! stringToReal(args[1], height) ||
      ! stringToReal(args[2], depth))
    return errorMsg("Invalid args");

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

int
App::
getAppValueProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "getAppValueProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() < 1)
    return errorMsg("Invalid args");

  auto name = args[0];

  if      (name == "cursor") {
    auto p = app->cursor();

    app->tcl_->setResult(pointToRealArray(p));
  }
  else if (name == "nearest_object") {
    if (args.size() < 2)
      return errorMsg("Invalid args");

    CPoint3D p;
    if (! app->stringToPoint(args[2], p))
      return errorMsg("Invalid point '" + args[2] + "'");

    auto *object = app->getNearestObject(p);

    app->tcl_->setResult(encodeObject(object));
  }
  else
    return errorMsg("Invalid value name '" + name + "'");

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
    return errorMsg("Invalid args");

  auto name = args[0];

  if      (name == "cursor") {
    CPoint3D p;
    if (! app->stringToPoint(args[1], p))
      return errorMsg("Invalid point '" + args[1] + "'");

    app->setCursor(p);
  }
  else
    return errorMsg("Invalid value name '" + name + "'");

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
    return errorMsg("Invalid args");

  CGeomObject3D *object;
  if (! app->decodeObject(args[0], object))
    return errorMsg("Invalid object id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "faces") {
    const auto &faces = object->getFaces();

    std::vector<std::string> faceIds1;

    for (auto *face : faces) {
      auto faceId1 = encodeObjectFace(face);

      faceIds1.push_back(faceId1);
    }

    app->tcl_->setResult(faceIds1);
  }
  else if (name == "edges") {
    const auto &edges = object->getEdges();

    std::vector<std::string> edgeIds1;

    for (auto *edge : edges) {
      auto edgeId1 = encodeObjectEdge(object, edge);

      edgeIds1.push_back(edgeId1);
    }

    app->tcl_->setResult(edgeIds1);
  }
  else if (name == "vertices") {
    const auto &vertices = object->getVertices();

    std::vector<std::string> vertices1;

    for (auto *vertex : vertices) {
      auto vertexId1 = encodeObjectVertex(vertex);

      vertices1.push_back(vertexId1);
    }

    app->tcl_->setResult(vertices1);
  }
  else if (name == "nearest_face") {
    if (args.size() < 2)
      return errorMsg("Invalid args");

    CPoint3D p;
    if (! app->stringToPoint(args[2], p))
      return errorMsg("Invalid point '" + args[2] + "'");

    auto *face = app->getNearestFace(object, p);

    app->tcl_->setResult(encodeObjectFace(face));
  }
  else if (name == "nearest_edge") {
    if (args.size() < 2)
      return errorMsg("Invalid args");

    CPoint3D p;
    if (! app->stringToPoint(args[2], p))
      return errorMsg("Invalid point '" + args[2] + "'");

    auto *edge = app->getNearestEdge(object, p);

    app->tcl_->setResult(encodeObjectEdge(object, edge));
  }
  else if (name == "nearest_vertex") {
    if (args.size() < 2)
      return errorMsg("Invalid args");

    CPoint3D p;
    if (! app->stringToPoint(args[2], p))
      return errorMsg("Invalid point '" + args[2] + "'");

    auto *vertex = app->getNearestVertex(object, p);

    app->tcl_->setResult(encodeObjectVertex(vertex));
  }
  else
    return errorMsg("Invalid value name '" + name + "'");

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
    return errorMsg("Invalid args");

  CGeomObject3D *object;
  if (! app->decodeObject(args[0], object))
    return errorMsg("Invalid object id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "name") {
    object->setName(args[2]);
  }
  else if (name == "material") {
    int materialId;
    if (! decodeMaterialId(args[2], materialId))
      return errorMsg("Invalid material id '" + args[2] + "'");

    auto *material = app->scene_->getMaterialById(materialId);
    if (! material)
      return errorMsg("Invalid material id " + std::to_string(materialId));

    object->setMaterialP(material);
  }
  else if (name == "translate") {
    std::vector<std::string> strs;
    app->tcl_->splitList(args[2], strs);

    double tx, ty, tz;

    if      (strs.size() == 3) {
      if (! stringToReal(strs[0], tx) ||
          ! stringToReal(strs[1], ty) ||
          ! stringToReal(strs[2], tz))
        return errorMsg("Invalid translate value");
    }
    else if (strs.size() == 1) {
      if (! stringToReal(strs[0], tx))
        return errorMsg("Invalid translate value");

      ty = tx;
      tz = tx;
    }
    else
      return errorMsg("Invalid translate value");

    object->translate(tx, ty, tz);
  }
  else if (name == "scale") {
    std::vector<std::string> strs;
    app->tcl_->splitList(args[2], strs);

    double sx, sy, sz;

    if      (strs.size() == 3) {
      if (! stringToReal(strs[0], sx) ||
          ! stringToReal(strs[1], sy) ||
          ! stringToReal(strs[2], sz))
        return errorMsg("Invalid scale value");
    }
    else if (strs.size() == 1) {
      if (! stringToReal(strs[0], sx))
        return errorMsg("Invalid scale value");

      sy = sx;
      sz = sx;
    }
    else
      return errorMsg("Invalid scale value");

    object->scale(sx, sy, sz);
  }
  else if (name == "rotate") {
    std::vector<std::string> strs;
    app->tcl_->splitList(args[2], strs);

    double ax, ay, az;

    if      (strs.size() == 3) {
      if (! stringToReal(strs[0], ax) ||
          ! stringToReal(strs[1], ay) ||
          ! stringToReal(strs[2], az))
        return errorMsg("Invalid rotate value");
    }
    else if (strs.size() == 1) {
      if (! stringToReal(strs[0], ax))
        return errorMsg("Invalid rotate value");

      ay = ax;
      az = ax;
    }
    else
      return errorMsg("Invalid rotate value");

    object->rotateModelX(degToRad(ax));
    object->rotateModelY(degToRad(ay));
    object->rotateModelZ(degToRad(az));
  }
  else
    return errorMsg("Invalid value name '" + name + "'");

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
    return errorMsg("Invalid args");

  CGeomFace3D *face;
  if (! app->decodeObjectFace(args[0], face))
    return errorMsg("Invalid face id '" + args[0] + "'");

  auto *object = face->getObject();

  auto name = args[1];

  if      (name == "color") {
    auto color = face->getColor();

    app->tcl_->setResult(color.stringEncode());
  }
  else if (name == "vertices") {
    const auto &vertexIds = face->getVertices();

    std::vector<std::string> vertices1;

    for (const auto &vertexId : vertexIds) {
      auto vertexId1 = encodeObjectVertexId(object, vertexId);

      vertices1.push_back(vertexId1);
    }

    app->tcl_->setResult(vertices1);
  }
  else if (name == "edges") {
    const auto &edges = face->getEdges();

    std::vector<std::string> edges1;

    for (const auto &edge : edges) {
      auto edgeId1 = encodeObjectEdge(object, edge);

      edges1.push_back(edgeId1);
    }

    app->tcl_->setResult(edges1);
  }
  else if (name == "bbox") {
    CBBox3D bbox;
    face->getModelBBox(bbox);

    app->tcl_->setResult(bboxToRealArrays(bbox));
  }
  else
    return errorMsg("Invalid value name '" + name + "'");

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
    return errorMsg("Invalid args");

  CGeomFace3D *face;
  if (! app->decodeObjectFace(args[0], face))
    return errorMsg("Invalid face id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "color") {
    if (args.size() != 3)
      return errorMsg("Invalid args");

    face->setColor(CRGBName::toRGBA(args[2]));
  }
  else if (name == "normal") {
    if (args.size() != 3)
      return errorMsg("Invalid args");

    CPoint3D p;
    if (! app->stringToPoint(args[2], p))
      return errorMsg("Invalid point '" + args[2] + "'");

    face->setNormal(CVector3D(p));
  }
  else if (name == "material") {
    int materialId;
    if (! decodeMaterialId(args[2], materialId))
      return errorMsg("Invalid material id '" + args[2] + "'");

    auto *material = app->scene_->getMaterialById(materialId);
    if (! material)
      return errorMsg("Invalid material id " + std::to_string(materialId));

    face->setMaterialP(material);
  }
  else if (name == "translate") {
    std::vector<std::string> strs;
    app->tcl_->splitList(args[2], strs);

    double tx, ty, tz;

    if      (strs.size() == 3) {
      if (! stringToReal(strs[0], tx) ||
          ! stringToReal(strs[1], ty) ||
          ! stringToReal(strs[2], tz))
        return errorMsg("Invalid translate value");
    }
    else if (strs.size() == 1) {
      if (! stringToReal(strs[0], tx))
        return errorMsg("Invalid translate value");

      ty = tx;
      tz = tx;
    }
    else
      return errorMsg("Invalid translate value");

    face->moveBy(CVector3D(tx, ty, tz));
  }
  else if (name == "scale") {
    std::vector<std::string> strs;
    app->tcl_->splitList(args[2], strs);

    double sx, sy, sz;

    if      (strs.size() == 3) {
      if (! stringToReal(strs[0], sx) ||
          ! stringToReal(strs[1], sy) ||
          ! stringToReal(strs[2], sz))
        return errorMsg("Invalid scale value");
    }
    else if (strs.size() == 1) {
      if (! stringToReal(strs[0], sx))
        return errorMsg("Invalid scale value");

      sy = sx;
      sz = sx;
    }
    else
      return errorMsg("Invalid scale value");

    face->scale(sx, sy, sz);
  }
  else if (name == "rotate") {
    std::vector<std::string> strs;
    app->tcl_->splitList(args[2], strs);

    double ax, ay, az;

    if      (strs.size() == 3) {
      if (! stringToReal(strs[0], ax) ||
          ! stringToReal(strs[1], ay) ||
          ! stringToReal(strs[2], az))
        return errorMsg("Invalid rotate value");
    }
    else if (strs.size() == 1) {
      if (! stringToReal(strs[0], ax))
        return errorMsg("Invalid rotate value");

      ay = ax;
      az = ax;
    }
    else
      return errorMsg("Invalid rotate value");

    face->rotateModelX(degToRad(ax));
    face->rotateModelY(degToRad(ay));
    face->rotateModelZ(degToRad(az));
  }
  else if (name == "bevel") {
    double d;
    if (! stringToReal(args[2], d))
      return errorMsg("Invalid bevel '" + args[2] + "'");

    face->bevel(d);
  }
  else if (name == "inset") {
    double d;
    if (! stringToReal(args[2], d))
      return errorMsg("Invalid inset '" + args[2] + "'");

    face->inset(d);
  }
  else
    return errorMsg("Invalid value name '" + name + "'");

  app->tcl_->setResult(encodeObjectFace(face));

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
    return errorMsg("Invalid args");

  CGeomEdge3D *edge;
  if (! app->decodeObjectEdge(args[0], edge))
    return errorMsg("Invalid edge id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "start") {
    auto start = edge->getStart();

    app->tcl_->setResult(encodeObjectVertexId(edge->getObject(), start));
  }
  else if (name == "end") {
    auto end = edge->getEnd();

    app->tcl_->setResult(encodeObjectVertexId(edge->getObject(), end));
  }
  else if (name == "normal") {
    auto normal = edge->calcNormal();

    app->tcl_->setResult(pointToRealArray(normal.point()));
  }
  else
    return errorMsg("Invalid value name '" + name + "'");

  return TCL_OK;
}

int
App::
setEdgeValueProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "setEdgeValueProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() < 3)
    return errorMsg("Invalid args");

  CGeomEdge3D *edge;
  if (! app->decodeObjectEdge(args[0], edge))
    return errorMsg("Invalid edge id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "move") {
    std::vector<std::string> strs;
    app->tcl_->splitList(args[2], strs);

    double x, y, z;

    if      (strs.size() == 3) {
      if (! stringToReal(strs[0], x) ||
          ! stringToReal(strs[1], y) ||
          ! stringToReal(strs[2], z))
        return errorMsg("Invalid delta '" + args[2] + "'");
    }
    else
      return errorMsg("Invalid delta '" + args[2] + "'");

    edge->moveBy(CVector3D(x, y, z));
  }
  else if (name == "scale") {
    double s;
    if (! stringToReal(args[2], s))
      return errorMsg("Invalid scale '" + args[2] + "'");

    edge->scale(s);
  }
  else if (name == "bevel") {
    double s;
    if (! stringToReal(args[2], s))
      return errorMsg("Invalid bevel '" + args[2] + "'");

    edge->bevel(s);
  }
  else
    return errorMsg("Invalid value name '" + name + "'");

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
    return errorMsg("Invalid args");

  CGeomVertex3D *vertex;
  if (! app->decodeObjectVertex(args[0], vertex))
    return errorMsg("Invalid vertex id '" + args[0] + "'");

  auto name = args[1];

  if      (name == "model") {
    auto p = vertex->getModel();

    app->tcl_->setResult(pointToRealArray(p));
  }
  else
    return errorMsg("Invalid value name '" + name + "'");

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
    return errorMsg("Invalid args");

  int materialId;
  if (! decodeMaterialId(args[0], materialId))
    return errorMsg("Invalid material id '" + args[2] + "'");

  auto *material = app->scene_->getMaterialById(materialId);
  if (! material)
    return errorMsg("Invalid material id " + std::to_string(materialId));

  auto name = args[1];

  if      (name == "diffuse") {
    if (args.size() != 3)
      return errorMsg("Invalid args");

    material->setDiffuse(CRGBName::toRGBA(args[2]));
  }
  else if (name == "diffuse_texture") {
    int textureId;
    if (! decodeMaterialId(args[2], textureId))
      return errorMsg("Invalid text id '" + args[2] + "'");

    auto *texture = app->scene_->getTextureById(textureId);
    if (! texture)
      return errorMsg("Invalid texture id " + std::to_string(textureId));

    material->setDiffuseTexture(texture);
  }
  else
    return errorMsg("Invalid value name '" + name + "'");

  app->tcl_->setResult(encodeMaterial(material));

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
    return errorMsg("Invalid args");

  std::vector<std::string> strs;
  app->tcl_->splitList(args[0], strs);

  std::vector<CGeomObject3D *> objects;

  for (const auto &str : strs) {
    CGeomObject3D *object;
    if (! app->decodeObject(str, object))
      return errorMsg("Invalid object id '" + str + "'");

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
    return errorMsg("Invalid args");

  CGeomObject3D *object;
  if (! app->decodeObject(args[0], object))
    return errorMsg("Invalid object id '" + args[0] + "'");

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
    return errorMsg("Invalid args");

  std::vector<std::string> strs;
  app->tcl_->splitList(args[0], strs);

  std::vector<CGeomObject3D *> objects;

  for (const auto &str : strs) {
    CGeomObject3D *object;
    if (! app->decodeObject(str, object))
      return errorMsg("Invalid object id '" + str + "'");

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
    return errorMsg("Invalid args");

  std::vector<std::string> strs;
  app->tcl_->splitList(args[0], strs);

  std::vector<CGeomObject3D *> objects;

  for (const auto &str : strs) {
    CGeomObject3D *object;
    if (! app->decodeObject(str, object))
      return errorMsg("Invalid object id '" + str + "'");

    objects.push_back(object);
  }

  auto *object = app->scene_->subtractObjects(objects);

  app->scene_->addObject(object);

  app->tcl_->setResult(encodeObject(object));

  return TCL_OK;
}

int
App::
extrudeFaceProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "extrudeFaceProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 2)
    return errorMsg("Invalid args");

  CGeomFace3D *face;
  if (! app->decodeObjectFace(args[0], face))
    return errorMsg("Invalid face id '" + args[0] + "'");

  double d;
  if (! stringToReal(args[1], d))
    return errorMsg("Invalid delta '" + args[1] + "'");

  auto *face1 = face->extrude(d);

  auto faceId2 = encodeObjectFace(face1);

  app->tcl_->setResult(faceId2);

  return TCL_OK;
}

int
App::
extrudeEdgeProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "extrudeEdgeProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 2)
    return errorMsg("Invalid args");

  CGeomEdge3D *edge;
  if (! app->decodeObjectEdge(args[0], edge))
    return errorMsg("Invalid edge id '" + args[0] + "'");

  double d;
  if (! stringToReal(args[1], d))
    return errorMsg("Invalid delta '" + args[1] + "'");

  auto *face1 = edge->extrude(d);

  auto faceId1 = encodeObjectFace(face1);

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
    return errorMsg("Invalid args");

  CGeomEdge3D *edge;
  if (! app->decodeObjectEdge(args[0], edge))
    return errorMsg("Invalid edge id '" + args[0] + "'");

  auto vind = edge->getObject()->mergeEdge(edge->getInd());

  auto vertexId = encodeObjectVertexId(edge->getObject(), vind);

  app->tcl_->setResult(vertexId);

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
    return errorMsg("Invalid args");

  CGeomFace3D *face;
  if (! app->decodeObjectFace(args[0], face))
    return errorMsg("Invalid face id '" + args[0] + "'");

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
    return errorMsg("Invalid args");

  CGeomEdge3D *edge;
  if (! app->decodeObjectEdge(args[0], edge))
    return errorMsg("Invalid edge id '" + args[0] + "'");

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
    return errorMsg("Invalid args");

  CGeomObject3D *object;
  if (! app->decodeObject(args[0], object))
    return errorMsg("Invalid edge id '" + args[0] + "'");

  uint mirrorDir = 0;

  for (int i = 0; args[1][i] != '\0'; ++i)
    if      (args[1][i] == 'x' || args[1][i] == 'X')
       mirrorDir |= uint(CGeomObject3D::MirrorDir::X);
    else if (args[1][i] == 'y' || args[1][i] == 'Y')
       mirrorDir |= uint(CGeomObject3D::MirrorDir::Y);
    else if (args[1][i] == 'z' || args[1][i] == 'Z')
       mirrorDir |= uint(CGeomObject3D::MirrorDir::Z);
    else {
      return errorMsg("Invalid mirror direction '" + args[1] + "'");
  }

  auto c = app->cursor();

  const auto &objects = object->mirror(CGeomObject3D::MirrorDir(mirrorDir), c);

  std::vector<std::string> objectIds1;

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
deleteObjectsProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "deleteObjectsProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 1)
    return errorMsg("Invalid args");

  std::vector<std::string> strs;
  app->tcl_->splitList(args[0], strs);

  std::vector<CGeomObject3D *> objects;

  for (const auto &str : strs) {
    CGeomObject3D *object;
    if (! app->decodeObject(str, object))
      return errorMsg("Invalid object id '" + str + "'");

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
readObjProc(const CTclUtil::StringList &args)
{
  if (args.size() != 1)
    return errorMsg("Invalid args");

  auto filename = args[0];

  auto format = CImportBase::filenameToType(filename);

  auto *im = CImportBase::createModel(format, filename);

  if (! im)
    return errorMsg("File format not recognised for '" + filename + "'");

  CFile file(filename);

  if (! im->read(file)) {
    delete im;
    return errorMsg("Failed to read model for '" + filename + "'");
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
writeObjProc(ClientData clientData, Tcl_Interp* /*interp*/, int objc, Tcl_Obj * const *objv)
{
  //std::cerr << "writeObjProc\n";

  auto *app = reinterpret_cast<App *>(clientData);

  auto args = CTclUtil::getObjArgs(objc, objv);
  if (args.size() != 1)
    return errorMsg("Invalid args");

  CFile file(args[0]);

  CImportObj obj;

  obj.write(&file, app->scene_);

  return TCL_OK;
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
decodeObjectFace(const std::string &arg, CGeomFace3D* &face) const
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
decodeObjectEdge(const std::string &arg, CGeomEdge3D* &edge) const
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
decodeObjectVertex(const std::string &arg, CGeomVertex3D* &vertex) const
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
    auto dist = face->distanceTo(p);

    if (! minFace || dist < minDist) {
      minFace = face;
      minDist = dist;
    }
  }

  return minFace;
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

CGeomVertex3D *
App::
getNearestVertex(CGeomObject3D *object, const CPoint3D &p) const
{
  CGeomVertex3D *minVertex { nullptr };
  double         minDist   { 0.0 };

  for (auto *vertex : object->getVertices()) {
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
  std::vector<std::string> strs;
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

}
