#include <CGeomObject3D.h>
#include <CGeomEdge3D.h>
#include <CGeometry3D.h>
#include <CGeomScene3D.h>
#include <CGeomZBuffer.h>
#include <CGeomNodeData.h>
#include <CGeomAnimationData.h>

#include <CFuncs.h>
#include <CTransform2D.h>
#include <CImagePtr.h>
#include <CQuaternion.h>

CGeomObject3D::
CGeomObject3D(CGeomScene3D *pscene, const std::string &name) :
 pscene_(pscene), name_(name)
{
  mesh_     = new CGeomMesh;
  skeleton_ = new CGeomSkeleton;
}

CGeomObject3D::
CGeomObject3D(const CGeomObject3D &object) :
 pscene_          (object.pscene_),
 name_            (object.name_),
 id_              (object.id_),
 meshName_        (object.meshName_),
 selected_        (object.selected_),
 visible_         (object.visible_),
 shadowed_        (object.shadowed_),
 drawPosition_    (object.drawPosition_),
 jointed_         (object.jointed_),
 coordFrame_      (object.coordFrame_),
 position_        (object.position_),
 diffuseTexture_  (object.diffuseTexture_),
 specularTexture_ (object.specularTexture_),
 normalTexture_   (object.normalTexture_),
 emissiveTexture_ (object.emissiveTexture_),
 materialP_       (object.materialP_),
 groups_          (object.groups_),
 viewMatrix_      (object.viewMatrix_),
 transformData_   (object.transformData_),
 animData_        (object.animData_),
 dv_              (object.dv_),
 da_              (object.da_)
{
  // copy mesh
  mesh_ = new CGeomMesh;

  for (auto *face : object.getFaces()) {
    auto *face1 = face->dup();

    addFace(face1);
  }

  for (auto *line : object.getLines()) {
    auto *line1 = line->dup();

    addLine(line1);
  }

  for (auto *vertex : object.getVertices()) {
    auto *vertex1 = vertex->dup();

    addVertex(vertex1);
  }

  auto *mesh1 = object.mesh();

  mesh_->vertexFaceList   = mesh1->vertexFaceList;
  mesh_->vertexFaceNormal = mesh1->vertexFaceNormal;
  mesh_->texturePoints    = mesh1->texturePoints;
  mesh_->normals          = mesh1->normals;

  //---

  // copy skeleton
  skeleton_ = new CGeomSkeleton;

  auto *skeleton1 = object.skeleton();

  skeleton_->nodes    = skeleton1->nodes;
  skeleton_->nodeIds  = skeleton1->nodeIds;
  skeleton_->meshNode = skeleton1->meshNode;
  skeleton_->rootNode = skeleton1->rootNode;

  //---

  // children done by hierDup

  updatePosition();

  // fix nodes object
  for (auto &pn : skeleton_->nodes) {
    auto &nodeData = pn.second;

    nodeData.setAnimObject(this);
    nodeData.setObject(nullptr);
  }
}

CGeomObject3D::
~CGeomObject3D()
{
  auto refObjects = this->refObjects();

  if (refObject()) {
    ObjectList refObjects1;

    for (auto *obj : refObjects) {
      if (obj != this)
        refObjects1.push_back(obj);
    }

    std::swap(refObjects_, refObjects1);
  }
  else {
    delete mesh_;
    delete skeleton_;

    for (auto *obj : refObjects)
      delete obj;

    refObjects_.clear();
  }

  // TODO: delete (or detach) children
}

//---

CGeomObject3D *
CGeomObject3D::
hierDup() const
{
  auto *obj = dup();

  obj->setInd(CGeometry3DInst->nextObjectId());

  for (auto *child : children()) {
    auto *child1 = child->hierDup();

    obj->addChild(child1);
  }

  return obj;
}

CGeomObject3D *
CGeomObject3D::
dup() const
{
  return new CGeomObject3D(*this);
}

//---

CGeomObject3D *
CGeomObject3D::
createRef() const
{
  auto *th = const_cast<CGeomObject3D *>(this);

  auto *obj = dup();

  obj->refObject_ = th;

  delete obj->mesh_;
  delete obj->skeleton_;

  obj->mesh_     = nullptr;
  obj->skeleton_ = nullptr;

  th->refObjects_.push_back(obj);

#if 1
  for (auto *child : children()) {
    auto *child1 = child->createRef();

    obj->addChild(child1);
  }
#endif

  return obj;
}

//---

void
CGeomObject3D::
clearGeometry(bool destroy)
{
  if (! refObject())
    mesh_->clear(destroy);

  if (! refObject())
    skeleton_->clear();

  groups_.clear();
}

//---

CMatrix3D
CGeomObject3D::
getTransform() const
{
  if (transformData_.global)
    return transformData_.transform;
  else
    return transformData_.translate*transformData_.rotate*transformData_.scale;
}

void
CGeomObject3D::
setTransform(const CMatrix3D &m)
{
  transformData_.global = true;

  transformData_.transform = m;
}

CMatrix3D
CGeomObject3D::
getTranslate() const
{
  if (transformData_.global)
    return CMatrix3D::identity();
  else
    return transformData_.translate;
}

void
CGeomObject3D::
setTranslate(double x, double y, double z)
{
  auto m = CMatrix3D::translation(x, y, z);

  setTranslate(m);
}

void
CGeomObject3D::
setTranslate(const CMatrix3D &m)
{
  transformData_.global    = false;
  transformData_.translate = m;
}

CMatrix3D
CGeomObject3D::
getRotate() const
{
  if (transformData_.global)
    return CMatrix3D::identity();

  return transformData_.rotate;
}

void
CGeomObject3D::
setRotate(double angle, const CVector3D &axis)
{
  auto m = CMatrix3D::rotation(angle, axis);

  setRotate(m);
}

void
CGeomObject3D::
setRotate(const CMatrix3D &m)
{
  transformData_.global = false;
  transformData_.rotate = m;
}

CMatrix3D
CGeomObject3D::
getScale() const
{
  if (transformData_.global)
    return CMatrix3D::identity();
  else
    return transformData_.scale;
}

void
CGeomObject3D::
setScale(double x, double y, double z)
{
  auto m = CMatrix3D::scale(x, y, z);

  setTranslate(m);
}

void
CGeomObject3D::
setScale(const CMatrix3D &m)
{
  transformData_.global = false;
  transformData_.scale  = m;
}

CMatrix3D
CGeomObject3D::
getHierTransform() const
{
  auto transform = getTransform();

  if (parent())
    transform = parent()->getHierTransform()*transform;

  return transform;
}

//---

bool
CGeomObject3D::
getHierSelected() const
{
  if (getSelected())
    return true;

  if (parent())
    return parent()->getHierSelected();

  return false;
}

void
CGeomObject3D::
setHierVisible(bool b)
{
  setVisible(b);

  for (auto *child : children())
    child->setHierVisible(b);
}

//---

void
CGeomObject3D::
setIsPrimitive(bool b, bool hier)
{
  isPrimitive_ = b;

  if (hier) {
    for (auto *child : children())
      child->setIsPrimitive(b, hier);
  }

#if 0
  if (isPrimitive_) {
    assert(! pscene_->getObjectByInd(getInd()));
  }
#endif
}

//---

CGeomObject3D *
CGeomObject3D::
parent() const
{
#if 0
  if (refObject())
    return refObject()->parent();
#endif

  return parent_;
}

const CGeomObject3D::ObjectList &
CGeomObject3D::
children() const
{
#if 0
  if (refObject())
    return refObject()->children();
#endif

  return children_;
}

CGeomObject3D::ObjectList
CGeomObject3D::
hierChildren() const
{
#if 0
  if (refObject())
    return refObject()->hierChildren();
#endif

  auto children = this->children();

  for (auto *child : this->children()) {
    auto children1 = child->hierChildren();

    for (auto *child1 : children1)
      children.push_back(child1);
  }

  return children;
}

void
CGeomObject3D::
addChild(CGeomObject3D *child)
{
#if 0
  assert(! refObject());
#endif

  child->parent_ = this;

  children_.push_back(child);
}

void
CGeomObject3D::
resetHier()
{
#if 0
  assert(! refObject());
#endif

  parent_ = nullptr;

  resetChildren();
}

void
CGeomObject3D::
resetChildren()
{
#if 0
  assert(! refObject());
#endif

  children_.clear();
}

//-----------

CPoint3D
CGeomObject3D::
getModelCenter() const
{
  CBBox3D bbox;
  getModelBBox(bbox);

  return bbox.getCenter();
}

void
CGeomObject3D::
setModelCenter(const CPoint3D &point)
{
  auto c = getModelCenter();

  CVector3D d(c, point);

  moveBy(d.point());
}

CVector3D
CGeomObject3D::
getModelSize() const
{
  CBBox3D bbox;
  getModelBBox(bbox);

  return bbox.getSize();
}

//-----------

void
CGeomObject3D::
validatePObject()
{
  for (auto *face : getFaces())
    assert(face->getObject() == this);

  for (auto *line : getLines())
    assert(line->getObject() == this);

  for (auto *vertex : getVertices())
    assert(vertex->getObject() == this);
}

//-----------

void
CGeomObject3D::
setTexture(CGeomTexture *texture)
{
  if (refObject())
    return refObject()->setTexture(texture);

  for (auto *face : getFaces())
    face->setTexture(texture);
}

void
CGeomObject3D::
setTexture(CImagePtr image)
{
  auto *texture = CGeometry3DInst->createTexture(image);

  setTexture(texture);
}

void
CGeomObject3D::
mapTexture(CGeomTexture *texture)
{
  setTexture(texture);
}

void
CGeomObject3D::
mapTexture(CImagePtr image)
{
  auto *texture = CGeometry3DInst->createTexture(image);

  mapTexture(texture);
}

void
CGeomObject3D::
setMask(CGeomMask *mask)
{
  if (refObject())
    return refObject()->setMask(mask);

  for (auto *face : getFaces())
    face->setMask(mask);
}

void
CGeomObject3D::
setMask(CImagePtr image)
{
  auto *mask = CGeometry3DInst->createMask(image);

  setMask(mask);
}

void
CGeomObject3D::
mapMask(CGeomMask *mask)
{
  setMask(mask);
}

void
CGeomObject3D::
mapMask(CImagePtr image)
{
  auto *mask = CGeometry3DInst->createMask(image);

  mapMask(mask);
}

void
CGeomObject3D::
setFaceFlags(uint flags)
{
  if (refObject())
    return refObject()->setFaceFlags(flags);

  for (auto *face : getFaces())
    face->setFlags(flags);
}

void
CGeomObject3D::
unsetFaceFlags(uint flags)
{
  if (refObject())
    return refObject()->unsetFaceFlags(flags);

  for (auto *face : getFaces())
    face->unsetFlags(flags);
}

bool
CGeomObject3D::
findVertex(const CPoint3D &point, uint *ind)
{
  if (refObject())
    return refObject()->findVertex(point, ind);

  uint i = 0;

  for (auto *vertex : getVertices()) {
    const auto &actual = vertex->getModel();

    if (point == actual) {
      *ind = i;
      return true;
    }

    ++i;
  }

  return false;
}

//---

uint
CGeomObject3D::
addVertex(const CPoint3D &point)
{
  auto *vertex = CGeometry3DInst->createVertex3D(this, point);

  return addVertex(vertex);
}

uint
CGeomObject3D::
addVertex(CGeomVertex3D *vertex)
{
  assert(! refObject());

  vertex->setObject(this);

  mesh_->vertices.push_back(vertex);

  auto ind = uint(mesh_->vertices.size() - 1);

  vertex->setInd(ind);

  mesh_->edgesValid = false;

  return ind;
}

void
CGeomObject3D::
addVertexFace(uint vertex_ind, uint face_ind)
{
  auto &faceList = mesh_->vertexFaceList[vertex_ind];

  faceList.push_back(face_ind);
}

uint
CGeomObject3D::
dupVertex(uint i)
{
  const auto &v = getVertex(i);

  return addVertex(v.getModel());
}

//---

const CGeomObject3D::TexturePoints &
CGeomObject3D::
getTexturePoints() const
{
  return mesh_->texturePoints;
}

uint
CGeomObject3D::
getNumTextuePoints() const
{
  return uint(mesh_->texturePoints.size());
}

uint
CGeomObject3D::
addTexturePoint(const CPoint2D &point)
{
  return addTexturePoint(CPoint3D(point.x, point.y, 0.0));
}

uint
CGeomObject3D::
addTexturePoint(const CPoint3D &point)
{
  mesh_->texturePoints.push_back(point);

  auto ind = uint(mesh_->texturePoints.size() - 1);

  return ind;
}

const CPoint3D &
CGeomObject3D::
texturePoint(uint i) const
{
  assert(i < mesh_->texturePoints.size());

  return mesh_->texturePoints[i];
}

//---

uint
CGeomObject3D::
getNumNormals() const
{
  return uint(mesh_->normals.size());
}

uint
CGeomObject3D::
addNormal(const CVector3D &v)
{
  mesh_->normals.push_back(v);

  auto ind = uint(mesh_->normals.size() - 1);

  return ind;
}

const CVector3D &
CGeomObject3D::
normal(uint i) const
{
  assert(i < mesh_->normals.size());

  return mesh_->normals[i];
}

//---

uint
CGeomObject3D::
addLine(uint start, uint end)
{
  auto *line = CGeometry3DInst->createLine3D(this, start, end);

  return addLine(line);
}

uint
CGeomObject3D::
addLine(CGeomLine3D *line)
{
  assert(! refObject());

  line->setObject(this);

  mesh_->lines.push_back(line);

  auto ind = uint(mesh_->lines.size() - 1);

  line->setInd(ind);

  return ind;
}

std::vector<uint>
CGeomObject3D::
addITriangles(uint *inds, uint num_inds)
{
  std::vector<uint> faceNums;

  for (uint i1 = 0, i2 = 1, i3 = 2; i1 < num_inds;
         i1 += 2, i2 = (i1 + 1) % num_inds, i3 = (i2 + 1) % num_inds) {
    uint faceNum = addITriangle(inds[i1], inds[i2], inds[i3]);

    faceNums.push_back(faceNum);
  }

  return faceNums;
}

uint
CGeomObject3D::
addITriangle(uint i1, uint i2, uint i3)
{
  VertexIList vertices;

  vertices.push_back(i1);
  vertices.push_back(i2);
  vertices.push_back(i3);

  return addFace(vertices);
}

uint
CGeomObject3D::
addIPolygon(uint *inds, uint num_inds)
{
  VertexIList vertices;

  for (uint i = 0; i < num_inds; ++i)
    vertices.push_back(inds[i]);

  return addFace(vertices);
}

uint
CGeomObject3D::
addFace(const VertexIList &vertices)
{
  auto *face = CGeometry3DInst->createFace3D(this, vertices);

  return addFace(face);
}

uint
CGeomObject3D::
addFace(CGeomFace3D *face)
{
  assert(! refObject());

  face->setObject(this);

  mesh_->faces.push_back(face);

  auto ind = uint(mesh_->faces.size() - 1);

  face->setInd(ind);

  mesh_->edgesValid = false;

  return ind;
}

void
CGeomObject3D::
removeFace(CGeomFace3D *face)
{
  assert(! refObject());

  face->setObject(nullptr);

  FaceList faces;

  for (auto *face1 : mesh_->faces) {
    if (face1 != face)
      faces.push_back(face1);
  }

  std::swap(mesh_->faces, faces);

  mesh_->edgesValid = false;
}

uint
CGeomObject3D::
addFaceSubFace(uint faceNum, const std::vector<uint> &vertices)
{
  auto *face = getFaceP(faceNum);

  return face->addSubFace(vertices);
}

uint
CGeomObject3D::
addFaceSubLine(uint faceNum, uint start, uint end)
{
  auto *face = getFaceP(faceNum);

  return face->addSubLine(start, end);
}

//---

const CGeomObject3D::VertexList &
CGeomObject3D::
getVertices() const
{
  if (refObject())
    return refObject()->getVertices();

  return mesh_->vertices;
}

uint
CGeomObject3D::
getNumVertices() const
{
  return uint(getVertices().size());
}

const CGeomVertex3D &
CGeomObject3D::
getVertex(uint i) const
{
  if (refObject())
    return refObject()->getVertex(i);

  return *mesh_->vertices[i];
}

CGeomVertex3D &
CGeomObject3D::
getVertex(uint i)
{
  if (refObject())
    return refObject()->getVertex(i);

  return *mesh_->vertices[i];
}

const CGeomVertex3D *
CGeomObject3D::
getVertexP(uint i) const
{
  if (refObject())
    return refObject()->getVertexP(i);

  return mesh_->vertices[i];
}

CGeomVertex3D *
CGeomObject3D::
getVertexP(uint i)
{
  if (refObject())
    return refObject()->getVertexP(i);

  return mesh_->vertices[i];
}

void
CGeomObject3D::
removeVertex(CGeomVertex3D *vertex)
{
  assert(! refObject());

  auto vind = vertex->getInd();

  vertex->setObject(nullptr);

  VertexList vertices;

  for (auto *vertex1 : mesh_->vertices) {
    if (vertex1 != vertex)
      vertices.push_back(vertex1);
    else
      vertices.push_back(nullptr);
  }

  std::swap(mesh_->vertices, vertices);

  // TODO: allow degenerate faces
  for (auto *face1 : mesh_->faces) {
    // TODO: fast check ?
    if (face1->hasVertex(vind))
      face1->removeVertex(vind);
  }

#if 0
  // TODO: allow degenerate lines
  for (auto *line1 : mesh_->lines) {
    // TODO: fast check ?
    line1->removeVertex(vind);
  }
#endif

  mesh_->edgesValid = false;
}

//---

const CGeomObject3D::LineList &
CGeomObject3D::
getLines() const
{
  if (refObject())
    return refObject()->getLines();

  return mesh_->lines;
}

CGeomObject3D::LineList &
CGeomObject3D::
getLines()
{
  if (refObject())
    return refObject()->getLines();

  return mesh_->lines;
}

uint
CGeomObject3D::
getNumLines() const
{
  return uint(getLines().size());
}

const CGeomLine3D &
CGeomObject3D::
getLine(uint i) const
{
  if (refObject())
    return refObject()->getLine(i);

  return *mesh_->lines[i];
}

CGeomLine3D *
CGeomObject3D::
getLineP(uint i) const
{
  if (refObject())
    return refObject()->getLineP(i);

  return mesh_->lines[i];
}

CGeomLine3D &
CGeomObject3D::
getLine(uint i)
{
  if (refObject())
    return refObject()->getLine(i);

  return *mesh_->lines[i];
}

//---

const CGeomObject3D::FaceList &
CGeomObject3D::
getFaces() const
{
  if (refObject())
    return refObject()->getFaces();

  return mesh_->faces;
}

CGeomObject3D::FaceList &
CGeomObject3D::
getFaces()
{
  if (refObject())
    return refObject()->getFaces();

  return mesh_->faces;
}

uint
CGeomObject3D::
getNumFaces() const
{
  return uint(getFaces().size());
}

const CGeomFace3D &
CGeomObject3D::
getFace(uint i) const
{
  if (refObject())
    return refObject()->getFace(i);

  return *mesh_->faces[i];
}

CGeomFace3D *
CGeomObject3D::
getFaceP(uint i) const
{
  if (refObject())
    return refObject()->getFaceP(i);

  return mesh_->faces[i];
}

CGeomFace3D &
CGeomObject3D::
getFace(uint i)
{
  if (refObject())
    return refObject()->getFace(i);

  return *mesh_->faces[i];
}

//---

CGeomObject3D::Group &
CGeomObject3D::
getGroup(const std::string &name)
{
  auto p = groups_.find(name);

  if (p == groups_.end()) {
    uint id = uint(groups_.size() + 1);

    p = groups_.insert(p, Groups::value_type(name, Group(name, id)));
  }

  return (*p).second;
}

CRGBA
CGeomObject3D::
getFaceColor() const
{
  if (getFaces().empty())
    return CRGBA::white();

  return getFaceColor(0);
}

void
CGeomObject3D::
setFaceColor(const CRGBA &rgba)
{
  if (refObject())
    return refObject()->setFaceColor(rgba);

  for (auto *face : mesh_->faces)
    face->setColor(rgba);
}

CRGBA
CGeomObject3D::
getFaceColor(uint faceNum) const
{
  auto *face = getFaceP(faceNum);

  return face->getColor();
}

void
CGeomObject3D::
setFaceColor(uint faceNum, const CRGBA &rgba)
{
  auto *face = getFaceP(faceNum);

  face->setColor(rgba);
}

void
CGeomObject3D::
setFaceDiffuse(const CRGBA &rgba)
{
  if (refObject())
    return refObject()->setFaceDiffuse(rgba);

  for (auto *face : mesh_->faces)
    face->setDiffuse(rgba);
}

void
CGeomObject3D::
setFaceSpecular(const CRGBA &rgba)
{
  if (refObject())
    return refObject()->setFaceSpecular(rgba);

  for (auto *face : mesh_->faces)
    face->setSpecular(rgba);
}

void
CGeomObject3D::
setFaceEmission(const CRGBA &rgba)
{
  if (refObject())
    return refObject()->setFaceEmission(rgba);

  for (auto *face : mesh_->faces)
    face->setEmission(rgba);
}

void
CGeomObject3D::
setFaceMaterial(uint faceNum, const CGeomMaterial &material)
{
  auto *face = getFaceP(faceNum);

  face->setMaterial(material);
}

void
CGeomObject3D::
setFaceTexture(uint faceNum, CGeomTexture *texture)
{
  setFaceDiffuseTexture(faceNum, texture);
}

void
CGeomObject3D::
setFaceDiffuseTexture(uint faceNum, CGeomTexture *texture)
{
  auto *face = getFaceP(faceNum);

  face->setDiffuseTexture(texture);
}

void
CGeomObject3D::
setFaceSpecularTexture(uint faceNum, CGeomTexture *texture)
{
  auto *face = getFaceP(faceNum);

  face->setSpecularTexture(texture);
}

void
CGeomObject3D::
setFaceNormalTexture(uint faceNum, CGeomTexture *texture)
{
  auto *face = getFaceP(faceNum);

  face->setNormalTexture(texture);
}

void
CGeomObject3D::
setFaceEmissiveTexture(uint faceNum, CGeomTexture *texture)
{
  auto *face = getFaceP(faceNum);

  face->setEmissiveTexture(texture);
}

void
CGeomObject3D::
setSubFaceColor(const CRGBA &rgba)
{
  if (refObject())
    return refObject()->setSubFaceColor(rgba);

  for (auto *face : mesh_->faces)
    face->setSubFaceColor(rgba);
}

void
CGeomObject3D::
setSubFaceColor(uint faceNum, const CRGBA &rgba)
{
  auto *face = getFaceP(faceNum);

  face->setSubFaceColor(rgba);
}

void
CGeomObject3D::
setSubFaceColor(uint faceNum, uint subFaceNum, const CRGBA &rgba)
{
  auto *face = getFaceP(faceNum);

  face->setSubFaceColor(subFaceNum, rgba);
}

void
CGeomObject3D::
setSubFaceMaterialP(CGeomMaterial *material)
{
  if (refObject())
    return refObject()->setSubFaceMaterialP(material);

  for (auto *face : mesh_->faces)
    face->setSubFaceMaterialP(material);
}

void
CGeomObject3D::
setLineColor(const CRGBA &rgba)
{
  for (auto *line : getLines())
    line->setColor(rgba);
}

void
CGeomObject3D::
setLineColor(uint lineNum, const CRGBA &rgba)
{
  getLineP(lineNum)->setColor(rgba);
}

void
CGeomObject3D::
setSubLineColor(uint faceNum, uint subLineNum, const CRGBA &rgba)
{
  auto *face = getFaceP(faceNum);

  face->setSubLineColor(subLineNum, rgba);
}

void
CGeomObject3D::
setVertexColor(uint i, const CRGBA &rgba)
{
  if (refObject())
    return refObject()->setVertexColor(i, rgba);

  mesh_->vertices[i]->setColor(rgba);
}

void
CGeomObject3D::
setVertexPixel(uint i, const CPoint3D &pixel)
{
  if (refObject())
    return refObject()->setVertexPixel(i, pixel);

  mesh_->vertices[i]->setPixel(pixel);
}

void
CGeomObject3D::
setVertexNormal(uint i, const CVector3D &n)
{
  if (refObject())
    return refObject()->setVertexNormal(i, n);

  mesh_->vertices[i]->setNormal(n);
}

void
CGeomObject3D::
setVertexTextureMap(uint i, const CPoint2D &p)
{
  if (refObject())
    return refObject()->setVertexTextureMap(i, p);

  mesh_->vertices[i]->setTextureMap(p);
}

void
CGeomObject3D::
setFrontMaterial(const CGeomMaterial &material)
{
  if (refObject())
    return refObject()->setFrontMaterial(material);

  for (auto *face : mesh_->faces)
    face->setFrontMaterial(material);
}

void
CGeomObject3D::
setBackMaterial(const CGeomMaterial &material)
{
  if (refObject())
    return refObject()->setBackMaterial(material);

  for (auto *face : mesh_->faces)
    face->setBackMaterial(material);
}

//---

CGeomObject3D *
CGeomObject3D::
getChildOfName(const std::string &name) const
{
  for (auto *child : children()) {
    if (child->getName() == name)
      return child;
  }

  for (auto *child : children()) {
    if (child->children().empty())
      continue;

    auto *child1 = child->getChildOfName(name);

    if (child1)
      return child1;
  }

  return nullptr;
}

//---

CGeomObject3D *
CGeomObject3D::
getAnimObject() const
{
  if (isAnimObject())
    return const_cast<CGeomObject3D *>(this);

  return (parent() ? parent()->getAnimObject() : nullptr);
}

bool
CGeomObject3D::
isAnimObject() const
{
  return (! skeleton_->nodes.empty());
}

bool
CGeomObject3D::
hasNode(int i) const
{
  if (i < 0) return false;

  auto pn = skeleton_->nodes.find(i);

  return (pn != skeleton_->nodes.end());
}

void
CGeomObject3D::
addNode(int i, const CGeomNodeData &data)
{
  assert(i >= 0);

#if 1
  auto pn = skeleton_->nodes.find(i);
  assert(pn == skeleton_->nodes.end());

  pn = skeleton_->nodes.insert(pn, NodeDatas::value_type(i, data));

  auto &data1 = (*pn).second;
#else
  skeleton_->nodes[i] = data;

  auto &data1 = skeleton_->nodes[i];
#endif

  data1.setInd    (i);
  data1.resetIndex();
  data1.setValid  (true);

  data1.setAnimObject(this);

  skeleton_->nodeIds.push_back(i);
}

const CGeomObject3D::NodeDatas &
CGeomObject3D::
getNodes() const
{
  return skeleton_->nodes;
}

const CGeomObject3D::NodeIds &
CGeomObject3D::
getNodeIds() const
{
  return skeleton_->nodeIds;
}

// map bone id into position in globalBoneTransform array
int
CGeomObject3D::
mapNodeId(int id) const
{
  for (size_t i = 0; i < skeleton_->nodeIds.size(); ++i) {
    if (skeleton_->nodeIds[i] == id)
      return int(i);
  }

  return -1;
}

int
CGeomObject3D::
mapNodeIndex(int index) const
{
  int i = 0;

  for (auto &pn : skeleton_->nodes) {
    auto &nodeData = pn.second;

    if (nodeData.index() == index)
      return i;

    ++i;
  }

  return -1;
}

const CGeomNodeData &
CGeomObject3D::
getNode(int i) const
{
  static CGeomNodeData noData;

  if (i >= 0) {
    auto pn = skeleton_->nodes.find(i);

    if (pn != skeleton_->nodes.end())
      return (*pn).second;
  }

  return noData;
}

CGeomNodeData &
CGeomObject3D::
editNode(int i)
{
  return const_cast<CGeomNodeData &>(getNode(i));
}

CGeomNodeData *
CGeomObject3D::
getNodeByInd(int ind) const
{
  for (auto &pn : skeleton_->nodes) {
    auto &nodeData = pn.second;

    if (nodeData.ind() == ind)
      return const_cast<CGeomNodeData *>(&nodeData);
  }

  return nullptr;
}

//---

CGeomObject3D *
CGeomObject3D::
getMeshObject() const
{
  int meshNodeId = getMeshNode();

  if (meshNodeId >= 0)
    return const_cast<CGeomObject3D *>(this);

  for (auto *child : children()) {
    auto *object = child->getMeshObject();

    if (object)
      return object;
  }

  return const_cast<CGeomObject3D *>(this);
}

int
CGeomObject3D::
getMeshNode() const
{
  if (refObject())
    return refObject()->getMeshNode();

  return skeleton_->meshNode;
}

void
CGeomObject3D::
setMeshNode(int id)
{
  if (refObject())
    return refObject()->setMeshNode(id);

  skeleton_->meshNode = id;
}

//---

CGeomObject3D *
CGeomObject3D::
getRootObject() const
{
  auto *rootObject = this;

  while (rootObject && rootObject->parent())
    rootObject = rootObject->parent();

  return const_cast<CGeomObject3D *>(rootObject);
}

CMatrix3D
CGeomObject3D::
getMeshGlobalTransform() const
{
#if 0
  auto *animObject = getAnimObject();

  int meshNodeId = getMeshNode();

  if (meshNodeId < 0)
    meshNodeId = animObject->getMeshNode();

  const auto &meshNodeData = animObject->getNode(meshNodeId);

  return meshNodeData.globalTransform();
#else
  auto *meshObject = this;
  int   meshNodeId = meshObject->getMeshNode();

  while (meshObject && meshNodeId < 0) {
    meshObject = meshObject->parent();

    if (meshObject)
      meshNodeId = meshObject->getMeshNode();
  }

  if (meshNodeId < 0)
    return CMatrix3D::identity();

  auto *animObject = getAnimObject();

  if (! animObject)
    return CMatrix3D::identity();

  const auto &meshNodeData = animObject->getNode(meshNodeId);

  return meshNodeData.globalTransform();
#endif
}

CMatrix3D
CGeomObject3D::
getMeshLocalTransform() const
{
  auto *animObject = getAnimObject();

  if (! animObject)
    return CMatrix3D::identity();

  int meshNodeId = getMeshNode();

  if (meshNodeId < 0)
    meshNodeId = animObject->getMeshNode();

  const auto &meshNodeData = animObject->getNode(meshNodeId);

  return meshNodeData.calcLocalTransform();
}

int
CGeomObject3D::
getRootNode() const
{
  if (skeleton_->rootNode < 0)
    return (! skeleton_->nodeIds.empty() ? skeleton_->nodeIds[0] : -1);
  else
    return skeleton_->rootNode;
}

void
CGeomObject3D::
setRootNode(int id)
{
  skeleton_->rootNode = id;
}

void
CGeomObject3D::
setNodeLocalTransforms(int i, const CTranslate3D &translation, const CRotate3D &rotation,
                       const CScale3D &scale)
{
  assert(i >= 0);

  auto pn = skeleton_->nodes.find(i);
  assert(pn != skeleton_->nodes.end());

  auto &nodeData = (*pn).second;

  nodeData.setLocalTranslation(translation);
  nodeData.setLocalRotation   (rotation);
  nodeData.setLocalScale      (scale);
}

#if 0
void
CGeomObject3D::
setNodeLocalTransform(int i, const CMatrix3D &m)
{
  assert(i >= 0);

  auto pn = skeleton_->nodes.find(i);
  assert(pn != skeleton_->nodes.end());

  auto &nodeData = (*pn).second;

  nodeData.localTransform = m;
}
#endif

void
CGeomObject3D::
setNodeGlobalTransform(int i, const CMatrix3D &m)
{
  assert(i >= 0);

  auto pn = skeleton_->nodes.find(i);
  assert(pn != skeleton_->nodes.end());

  auto &nodeData = (*pn).second;

  nodeData.setGlobalTransform(m);
}

CMatrix3D
CGeomObject3D::
getNodeHierTransform(const CGeomNodeData &nodeData) const
{
#if 0
  auto m = nodeData.localTranslation().matrix()*
           nodeData.localRotation().matrix()*
           nodeData.localScale().matrix();
#else
  auto m = nodeData.calcLocalTransform();
#endif

  int parentId = nodeData.parent();

  if (parentId >= 0) {
    const auto &pnode = getNode(parentId);

    if (pnode.isValid()) {
      m = getNodeHierTransform(pnode)*m;
    }
    else {
      auto *animObject = getAnimObject();

      if (animObject != this) {
        const auto &pnode1 = animObject->getNode(parentId);

        if (pnode1.isValid())
          m = animObject->getNodeHierTransform(pnode1)*m;
      }
    }
  }

  return m;
}

CMatrix3D
CGeomObject3D::
getNodeAnimHierTransform(const CGeomNodeData &nodeData, const std::string &animName, double t) const
{
  auto m = getNodeAnimTransform(nodeData, animName, t);

  int parentId = nodeData.parent();

  if (parentId >= 0) {
    auto &pnode = const_cast<CGeomNodeData &>(getNode(parentId));

    if (pnode.isValid()) {
      m = getNodeAnimHierTransform(pnode, animName, t)*m;
    }
    else {
      auto *animObject = getAnimObject();

      if (animObject != this) {
        const auto &pnode1 = animObject->getNode(parentId);

        if (pnode1.isValid())
          m = animObject->getNodeAnimHierTransform(pnode1, animName, t)*m;
      }
    }
  }

  return m;
}

CMatrix3D
CGeomObject3D::
getNodeAnimTransform(const CGeomNodeData &nodeData, const std::string &animName, double t) const
{
  auto *th = const_cast<CGeomObject3D *>(this);

  auto &nodeData1 = const_cast<CGeomNodeData &>(nodeData);

  CMatrix3D m;
  bool      mSet { false };

  if (th->updateNodeAnimationData(nodeData1, animName, t)) {
    auto &animationData = nodeData1.getAnimationData(animName);

#if 0
    m = animationData.animTranslation().matrix()*
        animationData.animRotation   ().matrix()*
        animationData.animScale      ().matrix();
#else
    m = animationData.animMatrix();
#endif

    mSet = true;
  }

  if (! mSet) {
#if 0
    m = nodeData.localTranslation().matrix()*
        nodeData.localRotation().matrix()*
        nodeData.localScale().matrix();
#else
    m = nodeData.calcLocalTransform();
#endif
  }

  return m;
}

double
CGeomObject3D::
calcTimeStep() const
{
  double tmin, tmax;

  if (getAnimationTranslationRange(animName(), tmin, tmax)) {
    int nt = animTimeFrames();

    if (nt > 1)
      return (tmax - tmin)/(nt - 1);
    else
      return (tmax - tmin);
  }

  return 0.1;
}

void
CGeomObject3D::
stepAnimTime()
{
  double tmin, tmax;

  if (getAnimationTranslationRange(animName(), tmin, tmax)) {
    auto d = animTimeStep();

    animData_.time += d;

    if (animData_.time > tmax) {
      if (isAnimRepeat())
        animData_.time = tmin;
      else
        animData_.time = tmax;
    }
  }
}

void
CGeomObject3D::
setNodeAnimationData(int i, const std::string &name, const CGeomAnimationData &data)
{
  assert(i >= 0);

  auto pn = skeleton_->nodes.find(i);
  assert(pn != skeleton_->nodes.end());

  auto &nodeData = (*pn).second;

  nodeData.setAnimationData(name, data);
}

void
CGeomObject3D::
setNodeAnimationTransformData(int i, const std::string &name, const Transform &transform,
                              const CGeomAnimationData &data)
{
  assert(i >= 0);

  auto pn = skeleton_->nodes.find(i);
  assert(pn != skeleton_->nodes.end());

  auto &nodeData = (*pn).second;

  auto &animData = nodeData.getAnimationData(name, data);

  if      (transform == Transform::TRANSLATION) {
    animData.setTranslationRange        (data.translationRange());
    animData.setTranslationRangeMin     (data.translationRangeMin());
    animData.setTranslationRangeMax     (data.translationRangeMax());
    animData.setTranslationInterpolation(data.translationInterpolation());
    animData.setTranslations            (data.translations());
  }
  else if (transform == Transform::ROTATION) {
    animData.setRotationRange        (data.rotationRange());
    animData.setRotationRangeMin     (data.rotationRangeMin());
    animData.setRotationRangeMax     (data.rotationRangeMax());
    animData.setRotationInterpolation(data.rotationInterpolation());
    animData.setRotations            (data.rotations());
  }
  else if (transform == Transform::SCALE) {
    animData.setScaleRange        (data.scaleRange());
    animData.setScaleRangeMin     (data.scaleRangeMin());
    animData.setScaleRangeMax     (data.scaleRangeMax());
    animData.setScaleInterpolation(data.scaleInterpolation());
    animData.setScales            (data.scales());
  }
  else if (transform == Transform::TRANSFORM) {
    animData.setTransformRange        (data.transformRange());
    animData.setTransformRangeMin     (data.transformRangeMin());
    animData.setTransformRangeMax     (data.transformRangeMax());
    animData.setTransformInterpolation(data.transformInterpolation());
    animData.setTransforms            (data.transforms());
  }
}

bool
CGeomObject3D::
updateNodesAnimationData(const std::string &name, double t)
{
  // update animation data for all nodes (skeleton/joints) of object
  bool rc = false;

  for (auto &pn : skeleton_->nodes) {
    auto &nodeData = pn.second;

    if (updateNodeAnimationData(nodeData, name, t))
      rc = true;
  }

  //---

//int rootId = getRootNode();

  // calc hierarchical animation matrix for all nodes (joints)
  for (auto &pn : skeleton_->nodes) {
    auto &nodeData = pn.second;

    auto hierAnimMatrix = nodeData.animMatrix();

    int parentId = nodeData.parent();

    while (parentId >= 0) {
      const auto &pnode = getNode(parentId);

      if (! pnode.isValid())
        break;

      hierAnimMatrix = pnode.animMatrix()*hierAnimMatrix;

      parentId = pnode.parent();
    }

    nodeData.setHierAnimMatrix(hierAnimMatrix);
  }

  // TODO: calc node matrices

  return rc;
}

bool
CGeomObject3D::
updateNodeAnimationData(int i, const std::string &name, double t)
{
  assert(i >= 0);

  auto pn = skeleton_->nodes.find(i);
  if (pn == skeleton_->nodes.end()) return false;

  return updateNodeAnimationData((*pn).second, name, t);
}

bool
CGeomObject3D::
updateHierNodeAnimationData(CGeomNodeData &nodeData, const std::string &name, double t)
{
  bool rc = true;

  if (! updateNodeAnimationData(nodeData, name, t)) {
    if (nodeData.parent()) {
      const auto &parentNode = getNode(nodeData.parent());

      nodeData.setAnimMatrix(parentNode.animMatrix()*nodeData.animMatrix());
    }
    else
      rc = false;
  }

  for (const auto &childId : nodeData.children()) {
    auto &childNode = const_cast<CGeomNodeData &>(getNode(childId));

    if (! updateHierNodeAnimationData(childNode, name, t))
      rc = false;
  }

  return rc;
}

bool
CGeomObject3D::
updateNodeAnimationData(CGeomNodeData &nodeData, const std::string &name, double t)
{
  CMatrix3D anim_matrix;
  bool      animSet { false };

  if (nodeData.hasAnimationData(name)) {
    auto &animationData = nodeData.getAnimationData(name);

    if (updateAnimationData(nodeData, animationData, t)) {
      anim_matrix = animationData.animMatrix();

      animSet = true;
    }
  }

  if (! animSet) {
#if 0
    const auto &anim_translation = nodeData.localTranslation();
    const auto &anim_rotation    = nodeData.localRotation();
    const auto &anim_scale       = nodeData.localScale();

    anim_matrix = anim_translation.matrix()*anim_rotation.matrix()*anim_scale.matrix();
#else
#if 0
    anim_matrix = nodeData.localTranslation().matrix()*
                  nodeData.localRotation().matrix()*
                  nodeData.localScale().matrix();
#else
    anim_matrix = nodeData.calcLocalTransform();
#endif
#endif
  }

  nodeData.setAnimMatrix(anim_matrix);

  return animSet;
}

bool
CGeomObject3D::
updateAnimationData(CGeomNodeData &nodeData, CGeomAnimationData &animationData, double t) const
{
  if (animationData.translations().empty() &&
      animationData.rotations   ().empty() &&
      animationData.scales      ().empty() &&
      animationData.transforms  ().empty())
    return false;

  CTranslate3D anim_translation;
  CRotate3D    anim_rotation;
  CScale3D     anim_scale;
  CMatrix3D    anim_transform;

  //---

  // translation
  if (! animationData.translations().empty()) {
    if (animationData.translationRange().empty()) {
      std::cerr << "Invalid range for animation translation\n";
      return false;
    }

    auto iv = CMathGen::mapIntoRangeSet<double>(t, animationData.translationRange());

    if (iv.first < 0) {
      std::cerr << "Invalid range for animation translation\n";
      return false;
    }

    auto ii = iv.first;
    auto fi = iv.second;

    //if (isDebug())
    //  std::cerr << "    sampler.output ind: " << ii << " " << fi << "\n";

    //---

    auto translationInterpolation = animationData.translationInterpolation();

    if      (translationInterpolation == CGeomAnimationData::Interpolation::LINEAR) {
      auto ov = CMathGen::interpRangeSet<CVector3D>(ii, fi, animationData.translations());

      if (! ov.first) {
        std::cerr << "Invalid values for animation translation\n";
        return false;
      }

      //if (isDebug())
      //  std::cerr << "    translation: " << ov.second << "\n";

      anim_translation = CTranslate3D(ov.second.x(), ov.second.y(), ov.second.z());
    }
    else if (translationInterpolation == CGeomAnimationData::Interpolation::STEP) {
      const auto &translation = animationData.translations()[ii];

      anim_translation = CTranslate3D(translation.x(), translation.y(), translation.z());
    }
    else {
      std::cerr << "Invalid interpolation for animation translation\n";
      return false;
    }
  }
  else
    anim_translation = nodeData.localTranslation();

  //---

  // rotation
  if (! animationData.rotations().empty()) {
    if (animationData.rotationRange().empty()) {
      std::cerr << "Invalid Range for animation rotation\n";
      return false;
    }

    auto iv = CMathGen::mapIntoRangeSet<double>(t, animationData.rotationRange());

    if (iv.first < 0) {
      std::cerr << "Invalid Range for animation rotation\n";
      return false;
    }

    auto ii = iv.first;
    auto fi = iv.second;

    //if (isDebug())
    //  std::cerr << "    sampler.output ind: " << ii << " " << fi << "\n";

    //---

    auto rotationInterpolation = animationData.rotationInterpolation();

    if      (rotationInterpolation == CGeomAnimationData::Interpolation::LINEAR) {
      auto ov = CMathGen::interpRangeSet<CQuaternion>(ii, fi, animationData.rotations());

      if (! ov.first) {
        std::cerr << "Invalid values for animation rotation\n";
        return false;
      }

      //if (isDebug())
      //  std::cerr << "    rotate: " << ov.second << "\n";

      anim_rotation = CRotate3D(ov.second);
    }
    else if (rotationInterpolation == CGeomAnimationData::Interpolation::STEP) {
      const auto &rotation = animationData.rotations()[ii];

      anim_rotation = CRotate3D(rotation);
    }
    else {
      std::cerr << "Invalid interpolation for animation rotation\n";
      return false;
    }
  }
  else
    anim_rotation = nodeData.localRotation();

  //---

  // scale
  if (! animationData.scales().empty()) {
    if (animationData.scaleRange().empty()) {
      std::cerr << "Invalid Range for animation scale\n";
      return false;
    }

    auto iv = CMathGen::mapIntoRangeSet<double>(t, animationData.scaleRange());

    if (iv.first < 0) {
      std::cerr << "Invalid Range for animation scale\n";
      return false;
    }

    auto ii = iv.first;
    auto fi = iv.second;

    //if (isDebug())
    //  std::cerr << "    sampler.output ind: " << ii << " " << fi << "\n";

    //---

    auto scaleInterpolation = animationData.scaleInterpolation();

    if      (scaleInterpolation == CGeomAnimationData::Interpolation::LINEAR) {
      auto ov = CMathGen::interpRangeSet<CVector3D>(ii, fi, animationData.scales());

      if (! ov.first) {
        std::cerr << "Invalid values for animation scale\n";
        return false;
      }

      //if (isDebug())
      //  std::cerr << "    scale: " << ov.second << "\n";

      anim_scale = CScale3D(ov.second.x(), ov.second.y(), ov.second.z());
    }
    else if (scaleInterpolation == CGeomAnimationData::Interpolation::STEP) {
      const auto &scale = animationData.scales()[ii];

      anim_scale = CScale3D(scale.x(), scale.y(), scale.z());
    }
    else {
      std::cerr << "Invalid interpolation for animation scale\n";
      return false;
    }
  }
  else
    anim_scale = nodeData.localScale();

  //---

  // transform
  if (! animationData.transforms().empty()) {
    if (animationData.transformRange().empty()) {
      std::cerr << "Invalid Range for animation transform\n";
      return false;
    }

    auto iv = CMathGen::mapIntoRangeSet<double>(t, animationData.transformRange());

    if (iv.first < 0) {
      std::cerr << "Invalid Range for animation transform\n";
      return false;
    }

    auto ii = iv.first;
    auto fi = iv.second;

    //if (isDebug())
    //  std::cerr << "    sampler.output ind: " << ii << " " << fi << "\n";

    //---

    auto transformInterpolation = animationData.transformInterpolation();

    if      (transformInterpolation == CGeomAnimationData::Interpolation::LINEAR) {
      auto ov = CMathGen::interpRangeSetDiscreet<CMatrix3D>(ii, fi, animationData.transforms());

      if (! ov.first) {
        std::cerr << "Invalid values for animation transform\n";
        return false;
      }

      //if (isDebug())
      //  std::cerr << "    transform: " << ov.second << "\n";

      anim_transform = ov.second;
    }
    else if (transformInterpolation == CGeomAnimationData::Interpolation::STEP) {
      const auto &transform = animationData.transforms()[ii];

      anim_transform = transform;
    }
    else {
      std::cerr << "Invalid interpolation for animation transform\n";
      return false;
    }
  }
  else
    anim_transform = nodeData.localTransform();

  //---

  if (! animationData.translations().empty() ||
      ! animationData.rotations   ().empty() ||
      ! animationData.scales      ().empty()) {
    animationData.setAnimTranslation(anim_translation);
    animationData.setAnimRotation   (anim_rotation);
    animationData.setAnimScale      (anim_scale);
  }
  else {
    animationData.setAnimTransform  (anim_transform);
  }

  return true;
}

void
CGeomObject3D::
getAnimationNames(std::vector<std::string> &names) const
{
  std::set<std::string> nameSet;

  for (const auto &pn : skeleton_->nodes) {
    const auto &nodeData = pn.second;

    for (const auto &pn1 : nodeData.animationDatas()) {
      auto pn2 = nameSet.find(pn1.first);

      if (pn2 == nameSet.end()) {
        nameSet.insert(pn1.first);

        names.push_back(pn1.first);
      }
    }
  }
}

bool
CGeomObject3D::
getAnimationTranslationRange(const std::string &name, double &min, double &max) const
{
  auto pt = animationRangeMap_.find(name);

  if (pt == animationRangeMap_.end()) {
    AnimationRange range;

    range.min = 0.0;
    range.max = 1.0;

    for (const auto &pn : skeleton_->nodes) {
      const auto &nodeData = pn.second;

      if (! nodeData.hasAnimationData(name))
        continue;

      auto &animationData = const_cast<CGeomNodeData &>(nodeData).getAnimationData(name);

      if (animationData.translationRangeMin() && animationData.translationRangeMax()) {
        range.min = animationData.translationRangeMin().value();
        range.max = animationData.translationRangeMax().value();
        range.set = true;
        break;
      }
    }

    auto *th = const_cast<CGeomObject3D *>(this);

    pt = th->animationRangeMap_.insert(pt, AnimationRangeMap::value_type(name, range));
  }

  const auto &range = (*pt).second;

  min = range.min;
  max = range.max;

  return range.set;
}

CGeomAnimationData &
CGeomObject3D::
getNodeAnimationData(int i, const std::string &name)
{
  static CGeomAnimationData noAnimationData;

  assert(i >= 0);

  auto pn = skeleton_->nodes.find(i);
  if (pn == skeleton_->nodes.end()) return noAnimationData;

  auto &nodeData = (*pn).second;

  if (! nodeData.hasAnimationData(name))
    return noAnimationData;

  return nodeData.getAnimationData(name);
}

void
CGeomObject3D::
addBodyRev(double *x, double *y, uint num_xy, uint num_patches, const BodyRevData &data)
{
  std::vector<double> c, s;

  c.resize(num_patches);
  s.resize(num_patches);

  // rotate from angle (angleStart -> angleStart + angleDelta)
  double theta           = data.angleStart;
  double theta_increment = data.angleDelta/num_patches;

  for (uint i = 0; i < num_patches; ++i) {
    c[i] = std::cos(theta);
    s[i] = std::sin(theta);

    theta += theta_increment;
  }

  //---

  std::vector<uint> index1, index2;

  index1.resize(num_patches + 1);
  index2.resize(num_patches + 1);

  uint *pindex1 = &index1[0];
  uint *pindex2 = &index2[0];

  //---

//std::vector<uint> addedVertices;
  std::vector<uint> addedFaces;

  auto tagVertex = [&](uint ind, uint i) {
    auto &v = getVertex(ind);
    v.setTag(i + 1);
//  addedVertices.push_back(ind);
  };

  auto uniquifyVertex = [&](uint ind, uint i) {
    if (! data.uniquify) return ind;
    auto ind1 = dupVertex(ind);
    tagVertex(ind1, i);
    return ind1;
  };

  //---

  // add bottom circle points
  if (fabs(x[0]) < CMathGen::EPSILON_E6) {
    CPoint3D p(0.0, y[0], 0.0);

    auto ind = addVertex(p);
    tagVertex(ind, 0);

    for (uint i = 0; i <= num_patches; ++i)
      pindex1[i] = ind;
  }
  else {
    for (uint i = 0; i < num_patches; ++i) {
      CPoint3D p(x[0]*c[i], y[0], -x[0]*s[i]);

      auto ind = addVertex(p);
      tagVertex(ind, 0);

      pindex1[i] = ind;
    }

    pindex1[num_patches] = pindex1[0];
  }

  // next circle (top of face)
  for (uint j = 1; j < num_xy; ++j) {
    // add top circle points
    if (fabs(x[j]) < CMathGen::EPSILON_E6) {
      CPoint3D p(0.0, y[j], 0.0);

      auto ind = addVertex(p);
      tagVertex(ind, j);

      for (uint i = 0; i <= num_patches; ++i)
        pindex2[i] = ind;
    }
    else {
      for (uint i = 0; i < num_patches; ++i) {
        CPoint3D p(x[j]*c[i], y[j], -x[j]*s[i]);

        auto ind = addVertex(p);
        tagVertex(ind, j);

        pindex2[i] = ind;
      }

      pindex2[num_patches] = pindex2[0];
    }

    //---

#if 0
    auto getVertexTag = [&](uint iv) {
      const auto &v = getVertex(iv);
      return v.getTag();
    };
#endif

    auto addFace3 = [&](uint v1, uint v2, uint v3) {
      VertexIList vertices;

      vertices.push_back(v1);
      vertices.push_back(v2);
      vertices.push_back(v3);

      auto faceNum = addFace(vertices);

      addedFaces.push_back(faceNum);

//    assert(getVertexTag(v1) == getVertexTag(v2));
//    assert(getVertexTag(v1) == getVertexTag(v3));

      return faceNum;
    };

    auto addFace4 = [&](uint v1, uint v2, uint v3, uint v4) {
      VertexIList vertices;

      vertices.push_back(v1);
      vertices.push_back(v2);
      vertices.push_back(v3);
      vertices.push_back(v4);

      auto faceNum = addFace(vertices);

      addedFaces.push_back(faceNum);

//    assert(getVertexTag(v1) == getVertexTag(v2));
//    assert(getVertexTag(v1) == getVertexTag(v3));
//    assert(getVertexTag(v1) == getVertexTag(v4));

      return faceNum;
    };

    //---

    // add faces
    if (pindex1[0] != pindex1[1]) {
      // triangle if bottom different and top same
      if (pindex2[0] == pindex2[1]) {
        for (uint i = 0; i < num_patches; ++i) {
          auto v1 = pindex1[i    ];
          auto v2 = pindex1[i + 1];
          auto v3 = pindex2[i    ];

          addFace3(v2, v3, v1);
        }
      }
      // quad if bottom different and top different
      else {
        for (uint i = 0; i < num_patches; ++i) {
          auto v1 = uniquifyVertex(pindex1[i    ], j);
          auto v2 = uniquifyVertex(pindex1[i + 1], j);
          auto v3 = pindex2[i    ];
          auto v4 = pindex2[i + 1];

          addFace4(v2, v4, v3, v1);
        }
      }
    }
    else {
      // triangle if top different and bottom same
      if (pindex2[0] != pindex2[1]) {
        for (uint i = 0; i < num_patches; ++i) {
          auto v1 = uniquifyVertex(pindex1[i    ], j);
          auto v3 = pindex2[i    ];
          auto v4 = pindex2[i + 1];

          addFace3(v4, v3, v1);
        }
      }
    }

    // swap bottom indices
    std::swap(pindex1, pindex2);
  }

  //---

#if 0
  for (const auto &ind : addedVertices) {
    auto &v = getVertex(ind);

    auto ind1 = v.getTag();

    if (ind1 != 0) {
      auto pt = data.tagInds.find(ind1 - 1);

      if (pt != data.tagInds.end())
        v.setTag((*pt).second);
      else
        v.setTag(0);
    }
    else
      v.setTag(0);
  }
#else
  std::set<uint> processedVertices;

  for (const auto &ind : addedFaces) {
    const auto &face = getFace(ind);

    for (const auto &iv : face.getVertices()) {
      auto pp = processedVertices.find(iv);
      if (pp != processedVertices.end())
        continue;

      auto &v = getVertex(iv);

      auto ind1 = v.getTag();

      if (ind1 != 0) {
        auto pt = data.tagInds.find(ind1 - 1);

        if (pt != data.tagInds.end())
          v.setTag((*pt).second);
        else
          v.setTag(0);
      }
      else
        v.setTag(0);

      processedVertices.insert(iv);
    }
  }
#endif
}

//--------

void
CGeomObject3D::
moveTo(const CPoint3D &position)
{
  coordFrame_.setOrigin(position);

  updatePosition();
}

void
CGeomObject3D::
moveBy(const CPoint3D &offset)
{
  coordFrame_.setOrigin(coordFrame_.getOrigin() + CVector3D(offset));

  updatePosition();
}

void
CGeomObject3D::
moveX(double dx)
{
  coordFrame_.moveX(dx);

  updatePosition();
}

void
CGeomObject3D::
moveY(double dy)
{
  coordFrame_.moveY(dy);

  updatePosition();
}

void
CGeomObject3D::
moveZ(double dz)
{
  coordFrame_.moveZ(dz);

  updatePosition();
}

void
CGeomObject3D::
rotate(const CPoint3D &angle)
{
  coordFrame_.rotateAboutXYZ(angle.x, angle.y, angle.z);
}

void
CGeomObject3D::
rotateX(double dx)
{
  coordFrame_.rotateAboutX(dx);
}

void
CGeomObject3D::
rotateY(double dy)
{
  coordFrame_.rotateAboutY(dy);
}

void
CGeomObject3D::
rotateZ(double dz)
{
  coordFrame_.rotateAboutZ(dz);
}

CPoint3D
CGeomObject3D::
transformTo(const CPoint3D &p) const
{
  return coordFrame_.transformTo(p);
}

CVector3D
CGeomObject3D::
transformTo(const CVector3D &v) const
{
  return coordFrame_.transformTo(v);
}

void
CGeomObject3D::
setBasis(const CVector3D &right, const CVector3D &up, const CVector3D &dir)
{
  if (! CCoordFrame3D::validate(right, up, dir)) {
    std::cerr << "Invalid basis\n";
    return;
  }

  coordFrame_.setBasis(right, up, dir);
}

void
CGeomObject3D::
getBasis(CVector3D &right, CVector3D &up, CVector3D &dir)
{
  coordFrame_.getBasis(right, up, dir);
}

//---

void
CGeomObject3D::
scale(double x, double y, double z, bool hier)
{
  auto s = CMatrix3D::scale(x, y, z);

  transform(s, hier);
}

void
CGeomObject3D::
translate(double x, double y, double z, bool hier)
{
  auto m = CMatrix3D::translation(x, y, z);

  transform(m, hier);
}

void
CGeomObject3D::
transform(const CMatrix3D &matrix, bool hier)
{
  if (refObject())
    return refObject()->transform(matrix, hier);

  CPoint3D point;

  for (auto *vertex : mesh_->vertices) {
    matrix.multiplyPoint(vertex->getModel(), point);

    vertex->setModel(point);
  }

  if (hier) {
    for (auto *child : children())
      child->transform(matrix, hier);
  }
}

//---

void
CGeomObject3D::
getMeshBBox(CBBox3D &bbox, bool hier) const
{
  if (refObject())
    return refObject()->getMeshBBox(bbox, hier);

  auto m = getMeshGlobalTransform();

  for (auto *vertex : mesh_->vertices)
    bbox += m*vertex->getModel();

  if (hier) {
    for (auto *child : children()) {
      CBBox3D bbox1;
      child->getMeshBBox(bbox1, hier);

      bbox += bbox1;
    }
  }
}

void
CGeomObject3D::
getModelBBox(CBBox3D &bbox, bool hier) const
{
  if (refObject())
    return refObject()->getModelBBox(bbox, hier);

  for (auto *vertex : mesh_->vertices)
    bbox += vertex->getModel();

  if (hier) {
    for (auto *child : children()) {
      CBBox3D bbox1;
      child->getModelBBox(bbox1, hier);

      bbox += bbox1;
    }
  }
}

void
CGeomObject3D::
getTransformedModelBBox(CBBox3D &bbox, bool hier) const
{
  auto t = getTransform();

  for (auto *vertex : getVertices())
    bbox += t*vertex->getModel();

  if (hier) {
    for (auto *child : children()) {
      CBBox3D bbox1;
      child->getTransformedModelBBox1(t, bbox1);

      bbox += bbox1;
    }
  }
}

void
CGeomObject3D::
getTransformedModelBBox1(const CMatrix3D &t, CBBox3D &bbox, bool hier) const
{
  auto t1 = t*getTransform();

  for (auto *vertex : getVertices())
    bbox += t1*vertex->getModel();

  if (hier) {
    for (auto *child : children()) {
      CBBox3D bbox1;
      child->getTransformedModelBBox1(t1, bbox1, hier);

      bbox += bbox1;
    }
  }
}

//---

void
CGeomObject3D::
reset()
{
  coordFrame_.reset();

  updatePosition();

  resetSpin();
}

CPoint3D
CGeomObject3D::
verticesModelMidPoint(const VertexIList &ivertices) const
{
  CPoint3D mid_point(0, 0, 0);

  double n1 = 1.0/double(ivertices.size());

  for (const auto &iv : ivertices) {
    auto *v = getVertexP(iv);

    mid_point += n1*v->getModel();
  }

  return mid_point;
}

CPoint3D
CGeomObject3D::
verticesViewedMidPoint(const VertexIList &ivertices) const
{
  CPoint3D mid_point(0, 0, 0);

  double n1 = 1.0/double(ivertices.size());

  for (const auto &iv : ivertices) {
    auto *v = getVertexP(iv);

    mid_point += n1*v->getViewed();
  }

  return mid_point;
}

CVector3D
CGeomObject3D::
verticesModelNormal(const VertexIList &vertices) const
{
  if (vertices.size() < 3)
    return CVector3D(0, 1, 0);

  auto *v1 = getVertexP(vertices[0]);
  auto *v2 = getVertexP(vertices[1]);
  auto *v3 = getVertexP(vertices[2]);

  CVector3D diff1(v1->getModel(), v2->getModel());
  CVector3D diff2(v2->getModel(), v3->getModel());

  return diff1.crossProduct(diff2).normalized();
}

CVector3D
CGeomObject3D::
verticesViewedNormal(const VertexIList &vertices) const
{
  if (vertices.size() < 3)
    return CVector3D(0, 1, 0);

  auto *v1 = getVertexP(vertices[0]);
  auto *v2 = getVertexP(vertices[1]);
  auto *v3 = getVertexP(vertices[2]);

  CVector3D diff1(v1->getViewed(), v2->getViewed());
  CVector3D diff2(v2->getViewed(), v3->getViewed());

  return diff1.crossProduct(diff2).normalized();
}

CVector3D
CGeomObject3D::
getVertexFaceNormal(uint ind) const
{
  auto p = mesh_->vertexFaceNormal.find(ind);

  if (p != mesh_->vertexFaceNormal.end())
    return (*p).second;

  auto *th = const_cast<CGeomObject3D *>(this);

  auto &faces = th->mesh_->vertexFaceList[ind];

  assert(! faces.empty());

  CVector3D n(0, 0, 0);

  CVector3D fn;

  for (auto pf = faces.begin(); pf != faces.end(); ++pf) {
    auto *face = getFaceP(*pf);

    face->calcViewedNormal(fn);

    n += fn;
  }

  th->mesh_->vertexFaceNormal[ind] = n;

  return n;
}

void
CGeomObject3D::
drawSolid(const CGeomCamera3D &camera, CGeom3DRenderer *renderer)
{
  if (! getVisible())
    return;

  drawSolidFaces(renderer);

  // drawSubLines(renderer);

  // if (isDrawPosition())
  //   drawPosition(renderer);

  if (getSelected())
    drawBBox(camera, renderer);
}

void
CGeomObject3D::
drawSolid(const CGeomCamera3D &camera, CGeomZBuffer *zbuffer)
{
  if (! getVisible())
    return;

  drawSolidFaces(zbuffer);

  drawSubLines(zbuffer);

  if (isDrawPosition())
    drawPosition(zbuffer);

  if (getSelected())
    drawBBox(camera, zbuffer);
}

void
CGeomObject3D::
drawWireframe(const CGeomCamera3D &camera, CGeom3DRenderer *renderer)
{
  if (! getVisible())
    return;

  drawLineFaces(renderer);

  //drawSubLines(zbuffer);

  //if (isDrawPosition())
  //  drawPosition(zbuffer);

  if (getSelected())
    drawBBox(camera, renderer);
}

void
CGeomObject3D::
drawWireframe(const CGeomCamera3D &camera, CGeomZBuffer *zbuffer)
{
  if (! getVisible())
    return;

  drawLineFaces(zbuffer);

  drawSubLines(zbuffer);

  if (isDrawPosition())
    drawPosition(zbuffer);

  if (getSelected())
    drawBBox(camera, zbuffer);
}

//---

void
CGeomObject3D::
modelToPixel(const CGeomCamera3D &camera)
{
  position_.currentToPixel(camera);

  for (auto *vertex : getVertices())
    vertex->modelToPixel(coordFrame_, camera);
}

void
CGeomObject3D::
toCurrent(const CGeomCamera3D &)
{
  for (auto *vertex : getVertices())
    vertex->setCurrent(coordFrame_.transformFrom(vertex->getModel()));
}

void
CGeomObject3D::
toView(const CGeomCamera3D &camera)
{
  for (auto *vertex : getVertices())
    vertex->setViewed(camera.transformTo(vertex->getCurrent()));
}

void
CGeomObject3D::
toView(CGeom3DRenderer *renderer)
{
  createViewMatrix(renderer, viewMatrix_);

  for (auto *vertex : getVertices())
    vertex->view(viewMatrix_);
}

void
CGeomObject3D::
createViewMatrix(CGeom3DRenderer *renderer, CMatrix3D &matrix)
{
  // transform model -> to fit renderer position/size
  int s  = int(std::min(renderer->getWidth(), renderer->getHeight())*0.9);
  int w2 = renderer->getWidth ()/2;
  int h2 = renderer->getHeight()/2;

  CBBox3D bbox;
  getModelBBox(bbox);

  if (! bbox.isSet())
    return;

  auto center = bbox.getCenter();
  auto size   = bbox.getSize  ();

  double scale = std::min(s/size.getX(), std::min(s/size.getY(), s/size.getZ()));

  matrix.setScaleTranslation(scale,
    w2 - center.getX(), h2 - center.getY(), h2 - center.getZ());
}

void
CGeomObject3D::
project(const CGeomCamera3D &camera)
{
  for (auto *vertex : getVertices())
    vertex->project(camera);
}

void
CGeomObject3D::
toPixel(const CGeomCamera3D &camera)
{
  for (auto *vertex : getVertices())
    vertex->toPixel(camera);
}

void
CGeomObject3D::
drawSolidFaces(CGeom3DRenderer *renderer)
{
  for (auto *face : getFaces())
    face->drawSolid(renderer);
}

void
CGeomObject3D::
drawSolidFaces(CGeomZBuffer *zbuffer)
{
  for (auto *face : getFaces())
    face->drawSolid(zbuffer);
}

void
CGeomObject3D::
drawLineFaces(CGeom3DRenderer *renderer)
{
  for (auto *face : getFaces())
    face->drawLines(renderer);
}

void
CGeomObject3D::
drawLineFaces(CGeomZBuffer *zbuffer)
{
  for (auto *face : getFaces())
    face->drawLines(zbuffer);
}

void
CGeomObject3D::
drawSubLines(CGeomZBuffer *zbuffer)
{
  for (auto *line : getLines())
    line->draw(zbuffer);
}

void
CGeomObject3D::
drawPosition(CGeomZBuffer *zbuffer)
{
  zbuffer->setForeground(CRGBA(1, 0, 0));

  zbuffer->drawOverlayZLine(int(position_.getPixel().x) - 2,
                            int(position_.getPixel().y),
                            int(position_.getPixel().x) + 2,
                            int(position_.getPixel().y));
  zbuffer->drawOverlayZLine(int(position_.getPixel().x),
                            int(position_.getPixel().y) - 2,
                            int(position_.getPixel().x),
                            int(position_.getPixel().y) + 2);
}

void
CGeomObject3D::
drawBBox(const CGeomCamera3D &camera, CGeomZBuffer *zbuffer)
{
  if (! pscene_)
    return;

  CBBox3D bbox;
  getModelBBox(bbox);

  bbox.scale(1.01);

  CPoint3D points[8] = {
    CPoint3D(bbox.getMin().x, bbox.getMin().y, bbox.getMin().z),
    CPoint3D(bbox.getMax().x, bbox.getMin().y, bbox.getMin().z),
    CPoint3D(bbox.getMax().x, bbox.getMax().y, bbox.getMin().z),
    CPoint3D(bbox.getMin().x, bbox.getMax().y, bbox.getMin().z),
    CPoint3D(bbox.getMin().x, bbox.getMin().y, bbox.getMax().z),
    CPoint3D(bbox.getMax().x, bbox.getMin().y, bbox.getMax().z),
    CPoint3D(bbox.getMax().x, bbox.getMax().y, bbox.getMax().z),
    CPoint3D(bbox.getMin().x, bbox.getMax().y, bbox.getMax().z),
  };

  CGeomVertex3D vertices[8] = {
    CGeomVertex3D(this, points[0]),
    CGeomVertex3D(this, points[1]),
    CGeomVertex3D(this, points[2]),
    CGeomVertex3D(this, points[3]),
    CGeomVertex3D(this, points[4]),
    CGeomVertex3D(this, points[5]),
    CGeomVertex3D(this, points[6]),
    CGeomVertex3D(this, points[7]),
  };

  for (int i = 0; i < 8; ++i)
    vertices[i].modelToPixel(coordFrame_, camera);

  zbuffer->setForeground(CRGBA(1, 0, 0));

  for (int i1 = 3, i2 = 0; i2 < 4; i1 = i2, ++i2) {
    const auto &point1 = vertices[i1].getPixel();
    const auto &point2 = vertices[i2].getPixel();

    zbuffer->drawZLine(int(point1.x), int(point1.y), point1.z,
                       int(point2.x), int(point2.y), point2.z);
  }

  for (int i1 = 7, i2 = 4; i2 < 8; i1 = i2, ++i2) {
    const auto &point1 = vertices[i1].getPixel();
    const auto &point2 = vertices[i2].getPixel();

    zbuffer->drawZLine(int(point1.x), int(point1.y), point1.z,
                       int(point2.x), int(point2.y), point2.z);
  }

  for (int i1 = 0, i2 = 4; i1 < 4; ++i1, ++i2) {
    const auto &point1 = vertices[i1].getPixel();
    const auto &point2 = vertices[i2].getPixel();

    zbuffer->drawZLine(int(point1.x), int(point1.y), point1.z,
                       int(point2.x), int(point2.y), point2.z);
  }
}

void
CGeomObject3D::
drawBBox(const CGeomCamera3D &, CGeom3DRenderer *)
{
#if 0
  CBBox3D bbox;
  getModelBBox(bbox);

  renderer->drawLine(bbox->getMin().x, bbox->getMin().y, bbox->getMin().z,
                     bbox->getMax().x, bbox->getMin().y, bbox->getMin().z);
  renderer->drawLine(bbox->getMax().x, bbox->getMin().y, bbox->getMin().z,
                     bbox->getMax().x, bbox->getMax().y, bbox->getMin().z);
  renderer->drawLine(bbox->getMax().x, bbox->getMax().y, bbox->getMin().z,
                     bbox->getMin().x, bbox->getMax().y, bbox->getMin().z);
  renderer->drawLine(bbox->getMin().x, bbox->getMax().y, bbox->getMin().z,
                     bbox->getMin().x, bbox->getMin().y, bbox->getMin().z);

  renderer->drawLine(bbox->getMin().x, bbox->getMin().y, bbox->getMax().z,
                     bbox->getMax().x, bbox->getMin().y, bbox->getMax().z);
  renderer->drawLine(bbox->getMax().x, bbox->getMin().y, bbox->getMax().z,
                     bbox->getMax().x, bbox->getMax().y, bbox->getMax().z);
  renderer->drawLine(bbox->getMax().x, bbox->getMax().y, bbox->getMax().z,
                     bbox->getMin().x, bbox->getMax().y, bbox->getMax().z);
  renderer->drawLine(bbox->getMin().x, bbox->getMax().y, bbox->getMax().z,
                     bbox->getMin().x, bbox->getMin().y, bbox->getMax().z);

  renderer->drawLine(bbox->getMin().x, bbox->getMin().y, bbox->getMin().z,
                     bbox->getMin().x, bbox->getMin().y, bbox->getMax().z);
  renderer->drawLine(bbox->getMax().x, bbox->getMin().y, bbox->getMin().z,
                     bbox->getMax().x, bbox->getMin().y, bbox->getMax().z);
  renderer->drawLine(bbox->getMax().x, bbox->getMax().y, bbox->getMin().z,
                     bbox->getMax().x, bbox->getMax().y, bbox->getMax().z);
  renderer->drawLine(bbox->getMin().x, bbox->getMax().y, bbox->getMin().z,
                     bbox->getMin().x, bbox->getMax().y, bbox->getMax().z);
#endif
}

//---

void
CGeomObject3D::
moveModel(const CPoint3D &d)
{
  CMatrix3D m;
  m.setTranslation(d.x, d.y, d.z);

  transform(m, /*hier*/false);
}

void
CGeomObject3D::
moveModelX(double dx)
{
  moveModel(CPoint3D(dx, 0, 0));
}

void
CGeomObject3D::
moveModelY(double dy)
{
  moveModel(CPoint3D(0, dy, 0));
}

void
CGeomObject3D::
moveModelZ(double dz)
{
  moveModel(CPoint3D(0, 0, dz));
}

void
CGeomObject3D::
rotateModelZ(double dz)
{
  CMatrix3D m;
  m.setRotation(CMathGen::Z_AXIS_3D, dz);

  transform(m, /*hier*/false);
}

void
CGeomObject3D::
rotateModelY(double dy)
{
  CMatrix3D m;
  m.setRotation(CMathGen::Y_AXIS_3D, dy);

  transform(m, /*hier*/false);
}

void
CGeomObject3D::
rotateModelX(double dx)
{
  CMatrix3D m;
  m.setRotation(CMathGen::X_AXIS_3D, dx);

  transform(m, /*hier*/false);
}

void
CGeomObject3D::
rotateModel(double angle, const CVector3D &axis, bool hier)
{
  CMatrix3D m;
  m.setRotation(angle, axis);

  transform(m, hier);
}

void
CGeomObject3D::
resizeModel(double factor)
{
  CMatrix3D m;
  m.setScale(factor, factor, factor);

  transform(m, /*hier*/false);
}

void
CGeomObject3D::
resizeModelX(double dx)
{
  CMatrix3D m;
  m.setScale(dx, 1.0, 1.0);

  transform(m, /*hier*/false);
}

void
CGeomObject3D::
resizeModelY(double dy)
{
  CMatrix3D m;
  m.setScale(1.0, dy, 1.0);

  transform(m, /*hier*/false);
}

void
CGeomObject3D::
resizeModelZ(double dz)
{
  CMatrix3D m;
  m.setScale(1.0, 1.0, dz);

  transform(m, /*hier*/false);
}

//---

void
CGeomObject3D::
swapXY()
{
#if 0
  for (auto *face : getFaces())
    face->swapXY();

  for (auto *line : getLines())
    line->swapXY();
#endif

  for (auto *vertex : getVertices())
    vertex->swapXY();

  for (auto *child : children())
    child->swapXY();
}

void
CGeomObject3D::
swapYZ()
{
#if 0
  for (auto *face : getFaces())
    face->swapYZ();

  for (auto *line : getLines())
    line->swapYZ();
#endif

  for (auto *vertex : getVertices())
    vertex->swapYZ();

  for (auto *child : children())
    child->swapYZ();
}

void
CGeomObject3D::
swapZX()
{
#if 0
  for (auto *face : getFaces())
    face->swapZX();

  for (auto *line : getLines())
    line->swapZX();
#endif

  for (auto *vertex : getVertices())
    vertex->swapZX();

  for (auto *child : children())
    child->swapZX();
}

void
CGeomObject3D::
invertX()
{
#if 0
  for (auto *face : getFaces())
    face->invertX();

  for (auto *line : getLines())
    line->invertX();
#endif

  for (auto *vertex : getVertices())
    vertex->invertX();

  for (auto *child : children())
    child->invertX();
}

void
CGeomObject3D::
invertY()
{
#if 0
  for (auto *face : getFaces())
    face->invertY();

  for (auto *line : getLines())
    line->invertY();
#endif

  for (auto *vertex : getVertices())
    vertex->invertY();

  for (auto *child : children())
    child->invertY();
}

void
CGeomObject3D::
invertZ()
{
#if 0
  for (auto *face : getFaces())
    face->invertZ();

  for (auto *line : getLines())
    line->invertZ();
#endif

  for (auto *vertex : getVertices())
    vertex->invertZ();

  for (auto *child : children())
    child->invertZ();
}

void
CGeomObject3D::
flipX(double x)
{
  for (auto *face : getFaces()) {
    if (face->getNormalSet()) {
      auto n = face->getNormal();
      n.scaleX(-1);
      face->setNormal(n);
    }
  }

#if 0
  for (auto *line : getLines())
    line->flipX(x);
#endif

  for (auto *vertex : getVertices())
    vertex->flipX(x);

  for (auto *child : children())
    child->flipX(x);
}

void
CGeomObject3D::
flipY(double y)
{
  for (auto *face : getFaces()) {
    if (face->getNormalSet()) {
      auto n = face->getNormal();
      n.scaleY(-1);
      face->setNormal(n);
    }
  }

#if 0
  for (auto *line : getLines())
    line->flipY(y);
#endif

  for (auto *vertex : getVertices())
    vertex->flipY(y);

  for (auto *child : children())
    child->flipY(y);
}

void
CGeomObject3D::
flipZ(double z)
{
  for (auto *face : getFaces()) {
    if (face->getNormalSet()) {
      auto n = face->getNormal();
      n.scaleZ(-1);
      face->setNormal(n);
    }
  }

#if 0
  for (auto *line : getLines())
    line->flipZ(z);
#endif

  for (auto *vertex : getVertices())
    vertex->flipZ(z);

  for (auto *child : children())
    child->flipZ(z);
}

//---

bool
CGeomObject3D::
lightPoint(const CPoint3D &point, const CVector3D &normal,
           const CGeomMaterial &material, CRGBA &rgba) const
{
  if (! pscene_)
    return false;

  return pscene_->lightPoint(point, normal, material, rgba);
}

//---

CGeomObject3D::FaceList
CGeomObject3D::
getMaterialFaces(CGeomMaterial *material) const
{
  FaceList faces;

  for (auto *face : getFaces()) {
    if (face->getMaterialP() == material)
      faces.push_back(face);
  }

  return faces;
}

//---

void
CGeomObject3D::
divideFace(CGeomFace3D *face, const CPoint3D &c)
{
  CVector3D normal;
  face->calcModelNormal(normal);

  auto ind = addVertex(c);

  setVertexNormal(ind, normal);

  const auto &vertices = face->getVertices();

  auto nv = vertices.size();
  assert(nv >= 3);

  std::vector<uint> vertices1;

  vertices1.resize(3);

  auto v1 = vertices[nv - 1];

  for (uint i = 0; i < nv; ++i) {
    auto v2 = vertices[i];

    vertices1[0] = v1;
    vertices1[1] = v2;
    vertices1[2] = ind;

    if (i == nv - 1)
      face->setVertices(vertices1);
    else {
      auto face1 = face->dup();

      face1->setVertices(vertices1);

      addFace(face1);
    }

    v1 = v2;
  }
}

//---

bool
CGeomObject3D::
triangulate()
{
  auto faces = getFaces();

  for (auto *face : faces)
    face->triangulate();

  for (auto *child : children())
    child->triangulate();

  return true;
}

bool
CGeomObject3D::
splitFacesByMaterial(std::vector<CGeomObject3D *> &newObjects) const
{
  assert(! refObject());

  using Faces         = std::vector<CGeomFace3D *>;
  using MaterialFaces = std::map<CGeomMaterial *, Faces>;

  MaterialFaces materialFaces;

  for (auto *face : mesh_->faces) {
    materialFaces[face->getMaterialP()].push_back(face);
  }

  if (materialFaces.size() < 2) {
    if (materialFaces.size() == 1) {
      auto *th = const_cast<CGeomObject3D *>(this);

      th->setMaterialP(materialFaces.begin()->first);
    }

    return false;
  }

  for (const auto &pm : materialFaces) {
    auto *material = pm.first;

    std::set<uint> vertexSet;

    for (auto *face : pm.second) {
      for (const auto &iv : face->getVertices()) {
        vertexSet.insert(iv);
      }
    }

    auto *newObject = CGeometry3DInst->createObject3D(pscene_, material->name());

    newObject->setMaterialP(material);

    std::map<uint, uint> vertexMap;

    for (const auto &iv : vertexSet) {
      auto *v = getVertexP(iv);

      auto *v1 = v->dup();

      auto iv1 = newObject->addVertex(v1);

      vertexMap[iv] = iv1;
    }

    for (auto *face : pm.second) {
      VertexIList faceVertices;

      for (const auto &iv : face->getVertices())
        faceVertices.push_back(vertexMap[iv]);

      auto newFaceId = newObject->addFace(faceVertices);

      auto *newFace = newObject->getFaceP(newFaceId);

      newFace->setTexturePoints(face->getTexturePoints());
    }

    newObjects.push_back(newObject);
  }

  return true;
}

//---

void
CGeomObject3D::
updateMaterials()
{
  auto *objectMaterial = getMaterialP();
  if (objectMaterial) return;

  using MaterialSet = std::set<CGeomMaterial *>;

  MaterialSet materialSet;

  auto getMaterial = [&](const CRGBA &diffuse) {
    for (auto *m : materialSet) {
      if (m->getDiffuse() == diffuse)
        return m;
    }

    return static_cast<CGeomMaterial *>(nullptr);
  };

  uint materialNum = 1;

  for (auto *face : getFaces()) {
    auto *faceMaterial = face->getMaterialP();
    if (faceMaterial) continue;

    if (! face->color())
      face->setColor(CRGBA::white());

    auto diffuse = face->color().value();

    faceMaterial = getMaterial(diffuse);

    if (! faceMaterial) {
      faceMaterial = CGeometry3DInst->createMaterial();

      faceMaterial->setName("_material" + std::to_string(materialNum++));

      faceMaterial->setDiffuse(diffuse);

      materialSet.insert(faceMaterial);
    }

    face->setMaterialP(faceMaterial);
  }
}

//---

bool
CGeomObject3D::
edgesValid() const
{
  return mesh_->edgesValid;
}

const CGeomObject3D::EdgeList &
CGeomObject3D::
getEdges() const
{
  if (refObject())
    return refObject()->getEdges();

  if (! mesh_->edgesValid) {
    auto *th = const_cast<CGeomObject3D *>(this);

    th->mesh_->edges    .clear();
    th->mesh_->edgeFaces.clear();

    // add edges to faces
    for (auto *face : th->mesh_->faces) {
      const auto &vertices = face->getVertices();

      auto nv = vertices.size();

      size_t i1 = nv - 1;

      for (size_t i2 = 0; i2 < nv; i1 = i2++) {
        auto *edge = getVertexVertexEdge(vertices[i1], vertices[i2]);

        if (! edge)
          edge = th->addVertexVertexEdge(vertices[i1], vertices[i2]);

        auto pe = th->mesh_->edgeFaces.find(edge);

        if (pe == th->mesh_->edgeFaces.end())
          pe = th->mesh_->edgeFaces.insert(pe, EdgeFaces::value_type(edge, FaceList()));

        (*pe).second.push_back(face);
      }
    }

    std::map<uint, CGeomEdge3D *> sortedEdges;

    for (auto &pe : th->mesh_->edgeFaces)
      sortedEdges[pe.first->getInd()] = pe.first;

    for (auto &pe : sortedEdges)
      th->mesh_->edges.push_back(pe.second);

    th->mesh_->edgesValid = true;
  }

  return mesh_->edges;
}

const CGeomEdge3D *
CGeomObject3D::
getEdgeP(uint edgeId) const
{
  const auto &edges = getEdges();

  for (auto *edge : edges) {
    if (edge->getInd() == edgeId)
      return edge;
  }

  return nullptr;
}

CGeomObject3D::EdgeList
CGeomObject3D::
getFaceEdges(const CGeomFace3D *face) const
{
  (void) getEdges();

  EdgeList edges;

  for (const auto &pe : mesh_->edgeFaces) {
    bool found = false;

    for (auto *face1 : pe.second) {
      if (face1 == face) {
        found = true;
        break;
      }
    }

    if (found)
      edges.push_back(pe.first);
  }

  return edges;
}

CGeomObject3D::FaceList
CGeomObject3D::
getEdgeFaces(const CGeomEdge3D *edge) const
{
  FaceList faces;

  (void) getEdges();

  auto pe = mesh_->edgeFaces.find(const_cast<CGeomEdge3D *>(edge));

  if (pe != mesh_->edgeFaces.end())
    faces = (*pe).second;

  return faces;
}

std::vector<CGeomEdge3D *>
CGeomObject3D::
getVertexEdges(const CGeomVertex3D *v) const
{
  std::set<CGeomEdge3D *> edgeSet;

  (void) getEdges();

  for (auto &edge : mesh_->edges) {
    if (! edge->hasVertex(v->getInd()))
      continue;

    edgeSet.insert(edge);
  }

  std::vector<CGeomEdge3D *> edges;

  for (auto *edge : edgeSet)
    edges.push_back(edge);

  return edges;
}

std::vector<CGeomFace3D *>
CGeomObject3D::
getVertexFaces(const CGeomVertex3D *v) const
{
  std::set<CGeomFace3D *> faceSet;

  for (auto &face : getFaces()) {
    if (! face->hasVertex(v->getInd()))
      continue;

    faceSet.insert(face);
  }

  std::vector<CGeomFace3D *> faces;

  for (auto *face : faceSet)
    faces.push_back(face);

  return faces;
}

CGeomEdge3D *
CGeomObject3D::
getVertexVertexEdge(uint v1, uint v2) const
{
  if (v1 < v2) std::swap(v1, v2);

  auto p1 = mesh_->vertexVertexEdgeMap.find(v1);
  if (p1 == mesh_->vertexVertexEdgeMap.end())
    return nullptr;

  auto p2 = (*p1).second.find(v2);
  if (p2 == (*p1).second.end())
    return nullptr;

  return (*p2).second;
}

CGeomEdge3D *
CGeomObject3D::
addVertexVertexEdge(uint v1, uint v2)
{
  if (v1 < v2) std::swap(v1, v2);

  auto *edge = CGeometry3DInst->createEdge3D(this, v1, v2);

  edge->setInd(++mesh_->edgeInd);

  mesh_->vertexVertexEdgeMap[v1][v2] = edge;

  return edge;
}

//---

uint
CGeomObject3D::
mergeEdge(uint edgeId)
{
  auto *edge = getEdgeP(edgeId);
  assert(edge);

  return edge->getObject()->mergeVertices(edge->getStart(), edge->getEnd());
}

uint
CGeomObject3D::
mergeVertices(uint vind1, uint vind2)
{
  auto *v1 = const_cast<CGeomVertex3D *>(getVertexP(vind1));
  auto *v2 = const_cast<CGeomVertex3D *>(getVertexP(vind2));

  // set new (merged) point
  auto p = (v1->getModel() + v2->getModel())/2.0;

  // move one vertex to point
  v1->setModel(p);

  // remove other vertex
  removeMergedVertex(vind2, vind1);

  mesh_->edgesValid = false;

  return vind1;
}

void
CGeomObject3D::
removeMergedVertex(uint oldInd, uint newInd)
{
  assert(! refObject());

  for (auto *face : mesh_->faces) {
    auto b1 = face->hasVertex(oldInd);
    if (! b1) continue;

    auto b2 = face->hasVertex(newInd);

    if (! b2)
      face->replaceVertex(oldInd, newInd);
    else
      face->removeVertex(oldInd);
  }
}

CGeomObject3D *
CGeomObject3D::
separateFace(const CGeomFace3D *face) const
{
  auto name = "object." + std::to_string(pscene_->getObjects().size() + 1);

  auto *newObject = CGeometry3DInst->createObject3D(pscene_, name);

  auto *face1 = face->dup();

  std::vector<uint> vertices1;

  for (const auto &vind : face->getVertices()) {
    auto *v  = getVertexP(vind);
    auto *v1 = v->dup();

    auto vind1 = newObject->addVertex(v1);

    vertices1.push_back(vind1);
  }

  face1->setVertices(vertices1);

  newObject->addFace(face1);

  pscene_->addObject(newObject);

  return newObject;
}

CGeomObject3D *
CGeomObject3D::
separateEdge(const CGeomEdge3D *edge) const
{
  auto name = "object." + std::to_string(pscene_->getObjects().size() + 1);

  auto *newObject = CGeometry3DInst->createObject3D(pscene_, name);

  auto *v1 = getVertexP(edge->getStart());
  auto *v2 = getVertexP(edge->getEnd  ());

  auto *vv1 = v1->dup();
  auto *vv2 = v2->dup();
  auto *vv3 = v2->dup();
  auto *vv4 = v1->dup();

  std::vector<uint> vertices1;

  vertices1.push_back(newObject->addVertex(vv1));
  vertices1.push_back(newObject->addVertex(vv2));
  vertices1.push_back(newObject->addVertex(vv3));
  vertices1.push_back(newObject->addVertex(vv4));

  auto *face1 = CGeometry3DInst->createFace3D(newObject, vertices1);

  newObject->addFace(face1);

  pscene_->addObject(newObject);

  return newObject;
}

std::vector<CGeomObject3D *>
CGeomObject3D::
mirror(MirrorDir dir, const CPoint3D &c) const
{
  std::vector<CGeomObject3D *> objects;

  objects.push_back(const_cast<CGeomObject3D *>(this));

  if (uint(dir) & uint(MirrorDir::X)) {
    std::vector<CGeomObject3D *> objects1;

    for (auto *object : objects) {
      objects1.push_back(object);

      auto *object1 = object->dup();

      object1->flipX(c.x);

      objects1.push_back(object1);
    }

    std::swap(objects, objects1);
  }

  if (uint(dir) & uint(MirrorDir::Y)) {
    std::vector<CGeomObject3D *> objects1;

    for (auto *object : objects) {
      objects1.push_back(object);

      auto *object1 = object->dup();

      object1->flipY(c.y);

      objects1.push_back(object1);
    }

    std::swap(objects, objects1);
  }

  if (uint(dir) & uint(MirrorDir::Z)) {
    std::vector<CGeomObject3D *> objects1;

    for (auto *object : objects) {
      objects1.push_back(object);

      auto *object1 = object->dup();

      object1->flipZ(c.z);

      objects1.push_back(object1);
    }

    std::swap(objects, objects1);
  }

  return objects;
}

bool
CGeomObject3D::
scaleFaces(const std::vector<CGeomFace3D *> &faces, const CPoint3D &c, const CVector3D &s)
{
  // get unique vertices
  std::set<uint> vertices;

  for (auto *face : faces) {
    for (const auto &iv : face->getVertices())
      vertices.insert(iv);
  }

  // scale by distance to center
  for (const auto &iv : vertices) {
    auto *v = getVertexP(iv);

    auto pv = v->getModel();

    auto n1 = CVector3D(c, pv);

    pv = c + s*n1;

    v->setModel(pv);
  }

  return true;
}

// duplicate face and move along normal
bool
CGeomObject3D::
extrudeFaces(const std::vector<CGeomFace3D *> &faces, double d)
{
  // unique set of vertices for all faces
  std::set<uint> vertices;

  for (auto *face : faces) {
    for (const auto &iv : face->getVertices())
      vertices.insert(iv);
  }

  //---

  // add new vertices for extruded faces
  std::map<uint, uint> newInds;

  for (auto &ind : vertices) {
    auto *v = getVertexP(ind);

    auto newInd = addVertex(v->getModel());

    newInds[ind] = newInd;
  }

  //---

  for (auto *face : faces) {
    const auto &fvertices = face->getVertices();

    // get face normal
    CVector3D normal;

    if (face->getNormalSet())
      normal = face->getNormal();
    else
      face->calcModelNormal(normal);

    // move new vertices along face normal
    std::vector<uint> faceInds;

    for (auto &ind : fvertices) {
      auto newInd = newInds[ind];

      auto *v = getVertexP(newInd);

      v->setModel(v->getModel() + d*normal);

      faceInds.push_back(newInd);
    }

    // add new extruded face
    auto *topFace = face->dup();
    topFace->setVertices(faceInds);
    addFace(topFace);

    topFace->setNormal(normal);
  }

  //---

  // add side faces
  for (auto *face : faces) {
    const auto &fvertices = face->getVertices();

    auto nv = fvertices.size();

    uint i1 = uint(nv - 1);

    for (uint i2 = 0; i2 < nv; i1 = i2++) {
      std::vector<uint> inds1;

      inds1.push_back(fvertices[i1]);
      inds1.push_back(fvertices[i2]);
      inds1.push_back(newInds [fvertices[i1]]);
      inds1.push_back(newInds [fvertices[i2]]);

      auto *face2 = face->dup();
      face2->setVertices(inds1);
      addFace(face2);

      CVector3D normal2;
      face2->calcModelNormal(normal2);

      face2->setNormal(normal2);
    }
  }

  return true;
}

bool
CGeomObject3D::
circularizeFaces(const std::vector<CGeomFace3D *> &faces)
{
  // get faces associated with each vertex and unique set of vertices for all faces
  std::map<uint, std::set<CGeomFace3D *>> vertexFaces;
  std::set<uint>                          vertices;

  for (auto *face : faces) {
    for (const auto &iv : face->getVertices()) {
      vertexFaces[iv].insert(face);

      vertices.insert(iv);
    }
  }

  CGeomVertex3D *center = nullptr;

  for (const auto &pv : vertexFaces) {
    if (pv.second.size() == faces.size()) {
      center = getVertexP(pv.first);
      break;
    }
  }

  if (! center || vertices.size() < 2)
    return false;

  auto pc = center->getModel();

  double d = 0.0;

  for (const auto &iv : vertices) {
    if (iv == center->getInd())
      continue;

    auto *v = getVertexP(iv);

    auto pv = v->getModel();

    d += pc.distanceTo(pv);
  }

  d /= double(vertices.size() - 1);

  for (const auto &iv : vertices) {
    if (iv == center->getInd())
      continue;

    auto *v = getVertexP(iv);

    auto pv = v->getModel();

    auto n1 = CVector3D(pc, pv).normalized();

    pv = pc + d*n1;

    v->setModel(pv);
  }

  return true;
}

CGeomFace3D *
CGeomObject3D::
fillVertices(const std::vector<CGeomVertex3D *> &vertices)
{
  std::set<uint> vinds;

  for (auto *vertex : vertices)
    vinds.insert(vertex->getInd());

  VertexIList vertices1;

  for (auto &vind : vinds)
    vertices1.push_back(vind);

  auto faceId = addFace(vertices1);

  auto *face = getFaceP(faceId);

  return face;
}
