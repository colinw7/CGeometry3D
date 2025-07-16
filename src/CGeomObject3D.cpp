#include <CGeomObject3D.h>
#include <CGeometry3D.h>
#include <CGeomScene3D.h>
#include <CGeomZBuffer.h>
#include <CFuncs.h>
#include <CTransform2D.h>
#include <CImagePtr.h>
#include <CQuaternion.h>

CGeomObject3D::
CGeomObject3D(const CGeomObject3D &object) :
 pscene_          (object.pscene_),
 name_            (object.name_),
 selected_        (object.selected_),
 visible_         (object.visible_),
 draw_position_   (object.draw_position_),
 coord_frame_     (object.coord_frame_),
 position_        (object.position_),
 diffuseTexture_  (object.diffuseTexture_),
 specularTexture_ (object.specularTexture_),
 normalTexture_   (object.normalTexture_),
 vertexFaceList_  (object.vertexFaceList_),
 vertexFaceNormal_(object.vertexFaceNormal_),
 texturePoints_   (object.texturePoints_),
 normals_         (object.normals_),
 view_matrix_     (object.view_matrix_),
 dv_              (object.dv_),
 da_              (object.da_)
{
  // copy faces
  for (auto *face : object.faces_) {
    auto *face1 = face->dup();

    face1->setObject(this);

    faces_.push_back(face1);

    auto ind = uint(faces_.size() - 1);

    face1->setInd(ind);
  }

  // copy lines
  for (auto *line : object.lines_) {
    auto *line1 = line->dup();

    line1->setObject(this);

    lines_.push_back(line1);

    auto ind = uint(lines_.size() - 1);

    line1->setInd(ind);
  }

  // copy vertices
  for (auto *vertex : object.vertices_) {
    auto *vertex1 = vertex->dup();

    vertex1->setObject(this);

    vertices_.push_back(vertex1);

    auto ind = uint(vertices_.size() - 1);

    vertex1->setInd(ind);
  }

  updatePosition();
}

CGeomObject3D::
~CGeomObject3D()
{
}

CGeomObject3D *
CGeomObject3D::
dup() const
{
  return new CGeomObject3D(*this);
}

//-----------

void
CGeomObject3D::
addChild(CGeomObject3D *child)
{
  child->parent_ = this;

  children_.push_back(child);
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
  for (auto *face : faces_)
    assert(face->getObject() == this);

  for (auto *line : lines_)
    assert(line->getObject() == this);

  for (auto *vertex : vertices_)
    assert(vertex->getObject() == this);
}

//-----------

void
CGeomObject3D::
setTexture(CGeomTexture *texture)
{
  for (auto *face : faces_)
    face->setTexture(texture);
}

void
CGeomObject3D::
setTexture(CImagePtr image)
{
  auto *texture = CGeometryInst->createTexture(image);

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
  auto *texture = CGeometryInst->createTexture(image);

  mapTexture(texture);
}

void
CGeomObject3D::
setMask(CGeomMask *mask)
{
  for (auto *face : faces_)
    face->setMask(mask);
}

void
CGeomObject3D::
setMask(CImagePtr image)
{
  auto *mask = CGeometryInst->createMask(image);

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
  auto *mask = CGeometryInst->createMask(image);

  mapMask(mask);
}

void
CGeomObject3D::
setFaceFlags(uint flags)
{
  for (auto *face : faces_)
    face->setFlags(flags);
}

void
CGeomObject3D::
unsetFaceFlags(uint flags)
{
  for (auto *face : faces_)
    face->unsetFlags(flags);
}

bool
CGeomObject3D::
findVertex(const CPoint3D &point, uint *ind)
{
  uint i = 0;

  for (auto *vertex : vertices_) {
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
  auto *vertex = CGeometryInst->createVertex3D(this, point);

  vertices_.push_back(vertex);

  auto ind = uint(vertices_.size() - 1);

  vertex->setInd(ind);

  return ind;
}

void
CGeomObject3D::
addVertexFace(uint vertex_ind, uint face_ind)
{
  FaceIList &face_list = vertexFaceList_[vertex_ind];

  face_list.push_back(face_ind);
}

//---

uint
CGeomObject3D::
addTexturePoint(const CPoint3D &point)
{
  texturePoints_.push_back(point);

  auto ind = uint(texturePoints_.size() - 1);

  return ind;
}

const CPoint3D &
CGeomObject3D::
texturePoint(uint i) const
{
  assert(i < texturePoints_.size());

  return texturePoints_[i];
}

//---

uint
CGeomObject3D::
addNormal(const CVector3D &v)
{
  normals_.push_back(v);

  auto ind = uint(normals_.size() - 1);

  return ind;
}

const CVector3D &
CGeomObject3D::
normal(uint i) const
{
  assert(i < normals_.size());

  return normals_[i];
}

//---

uint
CGeomObject3D::
addLine(uint start, uint end)
{
  auto *line = CGeometryInst->createLine3D(this, start, end);

  lines_.push_back(line);

  auto ind = uint(lines_.size() - 1);

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
    uint face_num = addITriangle(inds[i1], inds[i2], inds[i3]);

    faceNums.push_back(face_num);
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
  auto *face = CGeometryInst->createFace3D(this, vertices);

  faces_.push_back(face);

  auto ind = uint(faces_.size() - 1);

  face->setInd(ind);

  return ind;
}

uint
CGeomObject3D::
addFaceSubFace(uint face_num, const std::vector<uint> &vertices)
{
  return faces_[face_num]->addSubFace(vertices);
}

uint
CGeomObject3D::
addFaceSubLine(uint face_num, uint start, uint end)
{
  return faces_[face_num]->addSubLine(start, end);
}

void
CGeomObject3D::
setFaceColor(const CRGBA &rgba)
{
  for (auto *face : faces_)
    face->setColor(rgba);
}

void
CGeomObject3D::
setFaceColor(uint face_num, const CRGBA &rgba)
{
  faces_[face_num]->setColor(rgba);
}

void
CGeomObject3D::
setFaceDiffuse(const CRGBA &rgba)
{
  for (auto *face : faces_)
    face->setDiffuse(rgba);
}

void
CGeomObject3D::
setFaceSpecular(const CRGBA &rgba)
{
  for (auto *face : faces_)
    face->setSpecular(rgba);
}

void
CGeomObject3D::
setFaceMaterial(uint face_num, const CMaterial &material)
{
  faces_[face_num]->setMaterial(material);
}

void
CGeomObject3D::
setFaceTexture(uint face_num, CGeomTexture *texture)
{
  setFaceDiffuseTexture(face_num, texture);
}

void
CGeomObject3D::
setFaceDiffuseTexture(uint face_num, CGeomTexture *texture)
{
  faces_[face_num]->setDiffuseTexture(texture);
}

void
CGeomObject3D::
setFaceSpecularTexture(uint face_num, CGeomTexture *texture)
{
  faces_[face_num]->setSpecularTexture(texture);
}

void
CGeomObject3D::
setFaceNormalTexture(uint face_num, CGeomTexture *texture)
{
  faces_[face_num]->setNormalTexture(texture);
}

void
CGeomObject3D::
setFaceEmissiveTexture(uint face_num, CGeomTexture *texture)
{
  faces_[face_num]->setEmissiveTexture(texture);
}

void
CGeomObject3D::
setSubFaceColor(const CRGBA &rgba)
{
  for (auto *face : faces_)
    face->setSubFaceColor(rgba);
}

void
CGeomObject3D::
setSubFaceColor(uint face_num, const CRGBA &rgba)
{
  faces_[face_num]->setSubFaceColor(rgba);
}

void
CGeomObject3D::
setSubFaceColor(uint face_num, uint sub_face_num, const CRGBA &rgba)
{
  faces_[face_num]->setSubFaceColor(sub_face_num, rgba);
}

void
CGeomObject3D::
setLineColor(const CRGBA &rgba)
{
  for (auto *line : lines_)
    line->setColor(rgba);
}

void
CGeomObject3D::
setLineColor(uint line_num, const CRGBA &rgba)
{
  lines_[line_num]->setColor(rgba);
}

void
CGeomObject3D::
setSubLineColor(uint face_num, uint sub_line_num, const CRGBA &rgba)
{
  faces_[face_num]->setSubLineColor(sub_line_num, rgba);
}

void
CGeomObject3D::
setVertexColor(uint i, const CRGBA &rgba)
{
  vertices_[i]->setColor(rgba);
}

void
CGeomObject3D::
setVertexPixel(uint i, const CPoint3D &pixel)
{
  vertices_[i]->setPixel(pixel);
}

void
CGeomObject3D::
setVertexNormal(uint i, const CVector3D &n)
{
  vertices_[i]->setNormal(n);
}

void
CGeomObject3D::
setVertexTextureMap(uint i, const CPoint2D &p)
{
  vertices_[i]->setTextureMap(p);
}

void
CGeomObject3D::
setFrontMaterial(const CMaterial &material)
{
  for (auto *face : faces_)
    face->setFrontMaterial(material);
}

void
CGeomObject3D::
setBackMaterial(const CMaterial &material)
{
  for (auto *face : faces_)
    face->setBackMaterial(material);
}

bool
CGeomObject3D::
hasNode(int i) const
{
  auto pn = nodes_.find(i);

  return (pn != nodes_.end());
}

void
CGeomObject3D::
addNode(int i, const NodeData &data)
{
  nodes_[i] = data;
  nodes_[i].ind   = i;
  nodes_[i].valid = true;

  nodeIds_.push_back(i);
}

int
CGeomObject3D::
mapNodeId(int id) const
{
  for (size_t i = 0; i < nodeIds_.size(); ++i)
    if (nodeIds_[i] == id)
      return int(i);

  return -1;
}

const CGeomObject3D::NodeData &
CGeomObject3D::
getNode(int i) const
{
  static NodeData noData;

  auto pn = nodes_.find(i);

  return (pn != nodes_.end() ? (*pn).second : noData);
}

int
CGeomObject3D::
getMeshNode() const
{
  return meshNode_;
}

void
CGeomObject3D::
setMeshNode(int id)
{
  meshNode_ = id;
}

int
CGeomObject3D::
getRootNode() const
{
  if (rootNode_ < 0)
    return (! nodeIds_.empty() ? nodeIds_[0] : -1);
  else
    return rootNode_;
}

void
CGeomObject3D::
setRootNode(int id)
{
  rootNode_ = id;
}

void
CGeomObject3D::
setNodeLocalTransforms(int i, const CMatrix3D &translation, const CMatrix3D &rotation,
                       const CMatrix3D &scale)
{
  auto pn = nodes_.find(i);
  assert(pn != nodes_.end());

  auto &node = (*pn).second;

  node.localTranslation = translation;
  node.localRotation    = rotation;
  node.localScale       = scale;

  node.localTransform = node.localTranslation*node.localRotation*node.localScale;
}

void
CGeomObject3D::
setNodeLocalTransform(int i, const CMatrix3D &m)
{
  auto pn = nodes_.find(i);
  assert(pn != nodes_.end());

  auto &node = (*pn).second;

  node.localTransform = m;
}

void
CGeomObject3D::
setNodeGlobalTransform(int i, const CMatrix3D &m)
{
  auto pn = nodes_.find(i);
  assert(pn != nodes_.end());

  auto &node = (*pn).second;

  node.globalTransform = m;
}

void
CGeomObject3D::
setNodeAnimationData(int i, const std::string &name, const AnimationData &data)
{
  auto pn = nodes_.find(i);
  assert(pn != nodes_.end());

  auto &node = (*pn).second;

  node.animationDatas[name] = data;
}

void
CGeomObject3D::
setNodeAnimationTransformData(int i, const std::string &name, const Transform &transform,
                              const AnimationData &data)
{
  auto pn = nodes_.find(i);
  assert(pn != nodes_.end());

  auto &node = (*pn).second;

  auto pn1 = node.animationDatas.find(name);

  if (pn1 == node.animationDatas.end())
    pn1 = node.animationDatas.insert(pn1, AnimationDatas::value_type(name, data));

  AnimationData &nodeData = (*pn1).second;

  if      (transform == Transform::ROTATION)
    nodeData.rotation = data.rotation;
  else if (transform == Transform::TRANSLATION)
    nodeData.translation = data.translation;
  else if (transform == Transform::SCALE)
    nodeData.scale = data.scale;
}

bool
CGeomObject3D::
updateNodesAnimationData(const std::string &name, double t)
{
  bool rc = false;

  for (const auto &pn : nodes_) {
    const auto &node = pn.second;

    if (updateNodeAnimationData(node, name, t))
      rc = true;
  }

//int rootId = getRootNode();

  for (const auto &pn : nodes_) {
    const auto &node = pn.second;

    node.hierAnimMatrix = node.animMatrix;

    int parentId = node.parent;

    while (parentId >= 0) {
      const auto &pnode = getNode(parentId);

      node.hierAnimMatrix = pnode.animMatrix*node.hierAnimMatrix;

      parentId = pnode.parent;
    }
  }

  return rc;
}

bool
CGeomObject3D::
updateNodeAnimationData(int i, const std::string &name, double t)
{
  auto pn = nodes_.find(i);
  if (pn == nodes_.end()) return false;

  return updateNodeAnimationData((*pn).second, name, t);
}

bool
CGeomObject3D::
updateNodeAnimationData(const NodeData &node, const std::string &name, double t)
{
  auto anim_translation = node.localTranslation;
  auto anim_rotation    = node.localRotation;
  auto anim_scale       = node.localScale;

  CMatrix3D anim_matrix;

  auto pn = node.animationDatas.find(name);

  if (pn != node.animationDatas.end()) {
    const AnimationData &animationData = (*pn).second;

    auto iv = CMathGen::mapIntoRangeSet<double>(t, animationData.range);

    if (iv.first < 0) {
      std::cerr << "Invalid Range for sampler.input data\n";
      return false;
    }

    auto ii = iv.first;
    auto fi = iv.second;

    //if (isDebug())
    //  std::cerr << "    sampler.output ind: " << ii << " " << fi << "\n";

    //---

    if      (animationData.interpolation == AnimationInterpolation::LINEAR) {
      if (! animationData.translation.empty()) {
        auto ov = CMathGen::interpRangeSet<CVector3D>(ii, fi, animationData.translation);

        if (ov.first) {
          //if (isDebug())
          //  std::cerr << "    translation: " << ov.second << "\n";

          anim_translation = CMatrix3D::translation(ov.second.x(), ov.second.y(), ov.second.z());
        }
      }

      if (! animationData.rotation.empty()) {
        auto ov = CMathGen::interpRangeSet<CQuaternion>(ii, fi, animationData.rotation);

        if (ov.first) {
          //if (isDebug())
          //  std::cerr << "    rotate: " << ov.second << "\n";

          ov.second.toRotationMatrix(anim_rotation);
        }
      }

      if (! animationData.scale.empty()) {
        auto ov = CMathGen::interpRangeSet<CVector3D>(ii, fi, animationData.scale);

        if (ov.first) {
          //if (isDebug())
          //  std::cerr << "    scale: " << ov.second << "\n";

          anim_scale = CMatrix3D::scale(ov.second.x(), ov.second.y(), ov.second.z());
        }
      }
    }
    else if (animationData.interpolation == AnimationInterpolation::STEP) {
      if (! animationData.translation.empty()) {
        const auto &translation = animationData.translation[ii];

        anim_translation =
          CMatrix3D::translation(translation.x(), translation.y(), translation.z());
      }

      if (! animationData.rotation.empty()) {
        const auto &rotation = animationData.rotation[ii];

        rotation.toRotationMatrix(anim_rotation);
      }

      if (! animationData.scale.empty()) {
        const auto &scale = animationData.scale[ii];

        anim_scale = CMatrix3D::scale(scale.x(), scale.y(), scale.z());
      }
    }
    else
      return false;

    anim_matrix = anim_translation*anim_rotation*anim_scale;

    animationData.anim_translation = anim_translation;
    animationData.anim_rotation    = anim_rotation;
    animationData.anim_scale       = anim_scale;
    animationData.anim_matrix      = anim_matrix;
  }
  else {
    anim_matrix = anim_translation*anim_rotation*anim_scale;
  }

  //---

  node.animMatrix = anim_matrix;

  return true;
}

void
CGeomObject3D::
getAnimationNames(std::vector<std::string> &names) const
{
  std::set<std::string> nameSet;

  for (const auto &pn : nodes_) {
    for (const auto &pn1 : pn.second.animationDatas) {
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
getAnimationRange(const std::string &name, double &min, double &max) const
{
  min = 0.0;
  max = 1.0;

  for (const auto &pn : nodes_) {
    auto pn1 = pn.second.animationDatas.find(name);
    if (pn1 == pn.second.animationDatas.end()) continue;

    const auto &animationData = (*pn1).second;

    if (animationData.rangeMin && animationData.rangeMax) {
      min = animationData.rangeMin.value();
      max = animationData.rangeMax.value();
      return true;
    }
  }

  return false;
}

CGeomObject3D::AnimationData &
CGeomObject3D::
getNodeAnimationData(int i, const std::string &name)
{
  static AnimationData noAnimationData;

  auto pn = nodes_.find(i);
  if (pn == nodes_.end()) return noAnimationData;

  auto pn1 = (*pn).second.animationDatas.find(name);
  if (pn1 == (*pn).second.animationDatas.end()) return noAnimationData;

  return (*pn1).second;
}

void
CGeomObject3D::
addBodyRev(double *x, double *y, uint num_xy, uint num_patches)
{
  std::vector<double> c, s;

  c.resize(num_patches);
  s.resize(num_patches);

  double theta           = 0.0;
  double theta_increment = 2.0*M_PI/num_patches;

  for (uint i = 0; i < num_patches; ++i) {
    c[i] = cos(theta);
    s[i] = sin(theta);

    theta += theta_increment;
  }

  uint num_vertices = 0;

  std::vector<uint> index1, index2;

  index1.resize(num_patches + 1);
  index2.resize(num_patches + 1);

  uint *pindex1 = &index1[0];
  uint *pindex2 = &index2[0];

  if (fabs(x[0]) < CMathGen::EPSILON_E6) {
    CPoint3D p(0.0, y[0], 0.0);

    addVertex(p);

    for (uint i = 0; i <= num_patches; ++i)
      pindex1[i] = num_vertices;

    ++num_vertices;
  }
  else {
    for (uint i = 0; i < num_patches; ++i) {
      CPoint3D p(x[0]*c[i], y[0], -x[0]*s[i]);

      addVertex(p);

      pindex1[i] = num_vertices;

      ++num_vertices;
    }

    pindex1[num_patches] = pindex1[0];
  }

  for (uint j = 1; j < num_xy; ++j) {
    if (fabs(x[j]) < CMathGen::EPSILON_E6) {
      CPoint3D p(0.0,  y[j], 0.0);

      addVertex(p);

      for (uint i = 0; i <= num_patches; ++i)
        pindex2[i] = num_vertices;

      ++num_vertices;
    }
    else {
      for (uint i = 0; i < num_patches; ++i) {
        CPoint3D p(x[j]*c[i], y[j], -x[j]*s[i]);

        addVertex(p);

        pindex2[i] = num_vertices;

        ++num_vertices;
      }

      pindex2[num_patches] = pindex2[0];
    }

    if (pindex1[0] != pindex1[1]) {
      if (pindex2[0] == pindex2[1]) {
        for (uint i = 0; i < num_patches; ++i) {
          VertexIList vertices;

          vertices.push_back(pindex1[i + 1]);
          vertices.push_back(pindex2[i    ]);
          vertices.push_back(pindex1[i    ]);

          addFace(vertices);
        }
      }
      else {
        for (uint i = 0; i < num_patches; ++i) {
          VertexIList vertices;

          vertices.push_back(pindex1[i + 1]);
          vertices.push_back(pindex2[i + 1]);
          vertices.push_back(pindex2[i    ]);
          vertices.push_back(pindex1[i    ]);

          addFace(vertices);
        }
      }
    }
    else {
      if (pindex2[0] != pindex2[1]) {
        for (uint i = 0; i < num_patches; ++i) {
          VertexIList vertices;

          vertices.push_back(pindex2[i + 1]);
          vertices.push_back(pindex2[i    ]);
          vertices.push_back(pindex1[i    ]);

          addFace(vertices);
        }
      }
    }

    uint *pindex = pindex2;

    pindex2 = pindex1;
    pindex1 = pindex;
  }
}

//--------

void
CGeomObject3D::
moveTo(const CPoint3D &position)
{
  coord_frame_.setOrigin(position);

  updatePosition();
}

void
CGeomObject3D::
moveBy(const CPoint3D &offset)
{
  coord_frame_.setOrigin(coord_frame_.getOrigin() + CVector3D(offset));

  updatePosition();
}

void
CGeomObject3D::
setBasis(const CVector3D &right, const CVector3D &up, const CVector3D &dir)
{
  coord_frame_.setBasis(right, up, dir);
}

void
CGeomObject3D::
getBasis(CVector3D &right, CVector3D &up, CVector3D &dir)
{
  coord_frame_.getBasis(right, up, dir);
}

void
CGeomObject3D::
transform(const CMatrix3D &matrix)
{
  CPoint3D point;

  for (auto *vertex : vertices_) {
    matrix.multiplyPoint(vertex->getModel(), point);

    vertex->setModel(point);
  }
}

void
CGeomObject3D::
getModelBBox(CBBox3D &bbox) const
{
  for (auto *vertex : vertices_)
    bbox += vertex->getModel();
}

void
CGeomObject3D::
reset()
{
  coord_frame_.reset();

  updatePosition();

  resetSpin();
}

CPoint3D
CGeomObject3D::
verticesMidPoint(const VertexIList &ivertices) const
{
  CPoint3D mid_point(0,0,0);

  double n1 = 1.0/double(ivertices.size());

  for (const auto &iv : ivertices)
    mid_point += n1*vertices_[iv]->getViewed();

  return mid_point;
}

CVector3D
CGeomObject3D::
verticesNormal(const VertexIList &vertices) const
{
  auto *v1 = vertices_[vertices[0]];
  auto *v2 = vertices_[vertices[1]];
  auto *v3 = vertices_[vertices[2]];

  CVector3D diff1(v1->getViewed(), v2->getViewed());
  CVector3D diff2(v2->getViewed(), v3->getViewed());

  return diff1.crossProduct(diff2).normalized();
}

CVector3D
CGeomObject3D::
getVertexFaceNormal(uint ind) const
{
  auto p = vertexFaceNormal_.find(ind);

  if (p != vertexFaceNormal_.end())
    return (*p).second;

  auto *th = const_cast<CGeomObject3D *>(this);

  FaceIList &faces = th->vertexFaceList_[ind];

  assert(! faces.empty());

  CVector3D n(0,0,0);

  CVector3D fn;

  for (auto pf = faces.begin(); pf != faces.end(); ++pf) {
    auto *face = faces_[*pf];

    face->calcNormal(fn);

    n += fn;
  }

  th->vertexFaceNormal_[ind] = n;

  return n;
}

CPoint3D
CGeomObject3D::
transformTo(const CPoint3D &p) const
{
  return coord_frame_.transformTo(p);
}

CVector3D
CGeomObject3D::
transformTo(const CVector3D &v) const
{
  return coord_frame_.transformTo(v);
}

void
CGeomObject3D::
drawSolid(const CGeomCamera3D &camera, CGeom3DRenderer *renderer)
{
  if (! getVisible())
    return;

  drawSolidFaces(renderer);

  // drawSubLines(renderer);

  // if (draw_position_)
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

  if (draw_position_)
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

  //if (draw_position_)
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

  if (draw_position_)
    drawPosition(zbuffer);

  if (getSelected())
    drawBBox(camera, zbuffer);
}

void
CGeomObject3D::
modelToPixel(const CGeomCamera3D &camera)
{
  position_.currentToPixel(camera);

  for (auto *vertex : vertices_)
    vertex->modelToPixel(coord_frame_, camera);
}

void
CGeomObject3D::
toCurrent(const CGeomCamera3D &)
{
  for (auto *vertex : vertices_)
    vertex->setCurrent(coord_frame_.transformFrom(vertex->getModel()));
}

void
CGeomObject3D::
toView(const CGeomCamera3D &camera)
{
  for (auto *vertex : vertices_)
    vertex->setViewed(camera.transformTo(vertex->getCurrent()));
}

void
CGeomObject3D::
toView(CGeom3DRenderer *renderer)
{
  createViewMatrix(renderer, view_matrix_);

  for (auto *vertex : vertices_)
    vertex->view(view_matrix_);
}

void
CGeomObject3D::
createViewMatrix(CGeom3DRenderer *renderer, CMatrix3D &matrix)
{
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
  for (auto *vertex : vertices_)
    vertex->project(camera);
}

void
CGeomObject3D::
toPixel(const CGeomCamera3D &camera)
{
  for (auto *vertex : vertices_)
    vertex->toPixel(camera);
}

void
CGeomObject3D::
drawSolidFaces(CGeom3DRenderer *renderer)
{
  for (auto *face : faces_)
    face->drawSolid(renderer);
}

void
CGeomObject3D::
drawSolidFaces(CGeomZBuffer *zbuffer)
{
  for (auto *face : faces_)
    face->drawSolid(zbuffer);
}

void
CGeomObject3D::
drawLineFaces(CGeom3DRenderer *renderer)
{
  for (auto *face : faces_)
    face->drawLines(renderer);
}

void
CGeomObject3D::
drawLineFaces(CGeomZBuffer *zbuffer)
{
  for (auto *face : faces_)
    face->drawLines(zbuffer);
}

void
CGeomObject3D::
drawSubLines(CGeomZBuffer *zbuffer)
{
  for (auto *line : lines_)
    line->draw(zbuffer);
}

void
CGeomObject3D::
drawPosition(CGeomZBuffer *zbuffer)
{
  zbuffer->setForeground(CRGBA(1,0,0));

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
    vertices[i].modelToPixel(coord_frame_, camera);

  zbuffer->setForeground(CRGBA(1,0,0));

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

void
CGeomObject3D::
moveZ(double dz)
{
  coord_frame_.moveZ(dz);

  updatePosition();
}

void
CGeomObject3D::
moveY(double dy)
{
  coord_frame_.moveY(dy);

  updatePosition();
}

void
CGeomObject3D::
moveX(double dx)
{
  coord_frame_.moveX(dx);

  updatePosition();
}

void
CGeomObject3D::
moveModel(const CPoint3D &d)
{
  CMatrix3D m;

  m.setTranslation(d.x, d.y, d.z);

  CPoint3D model1;

  for (auto *vertex : vertices_) {
    auto model = vertex->getModel();

    m.multiplyPoint(model, model1);

    vertex->setModel(model1);
  }
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
rotate(const CPoint3D &angle)
{
  coord_frame_.rotateAboutXYZ(angle.x, angle.y, angle.z);
}

void
CGeomObject3D::
rotateX(double dx)
{
  coord_frame_.rotateAboutX(dx);
}

void
CGeomObject3D::
rotateY(double dy)
{
  coord_frame_.rotateAboutY(dy);
}

void
CGeomObject3D::
rotateZ(double dz)
{
  coord_frame_.rotateAboutZ(dz);
}

void
CGeomObject3D::
rotateModelZ(double dz)
{
  CMatrix3D m;

  m.setRotation(CMathGen::Z_AXIS_3D, dz);

  CPoint3D model1;

  for (auto *vertex : vertices_) {
    auto model = vertex->getModel();

    m.multiplyPoint(model, model1);

    vertex->setModel(model1);
  }
}

void
CGeomObject3D::
rotateModelY(double dy)
{
  CMatrix3D m;

  m.setRotation(CMathGen::Y_AXIS_3D, dy);

  CPoint3D model1;

  for (auto *vertex : vertices_) {
    auto model = vertex->getModel();

    m.multiplyPoint(model, model1);

    vertex->setModel(model1);
  }
}

void
CGeomObject3D::
rotateModelX(double dx)
{
  CMatrix3D m;

  m.setRotation(CMathGen::X_AXIS_3D, dx);

  CPoint3D model1;

  for (auto *vertex : vertices_) {
    auto model = vertex->getModel();

    m.multiplyPoint(model, model1);

    vertex->setModel(model1);
  }
}

void
CGeomObject3D::
resizeModel(double factor)
{
  for (auto *vertex : vertices_)
    vertex->setModel(vertex->getModel()*factor);
}

void
CGeomObject3D::
resizeModelX(double dx)
{
  for (auto *vertex : vertices_) {
    auto model = vertex->getModel();

    model.x *= dx;

    vertex->setModel(model);
  }
}

void
CGeomObject3D::
resizeModelY(double dy)
{
  for (auto *vertex : vertices_) {
    auto model = vertex->getModel();

    model.y *= dy;

    vertex->setModel(model);
  }
}

void
CGeomObject3D::
resizeModelZ(double dz)
{
  for (auto *vertex : vertices_) {
    auto model = vertex->getModel();

    model.z *= dz;

    vertex->setModel(model);
  }
}

bool
CGeomObject3D::
lightPoint(const CPoint3D &point, const CVector3D &normal,
           const CMaterial &material, CRGBA &rgba) const
{
  if (! pscene_)
    return false;

  return pscene_->lightPoint(point, normal, material, rgba);
}
