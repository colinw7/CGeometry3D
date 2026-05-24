#ifndef CQGLBuffer_H
#define CQGLBuffer_H

#include <CBBox3D.h>
#include <CPoint4D.h>
#include <CRGBA.h>

#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QColor>

#include <vector>
#include <optional>
#include <cassert>

class CQGLBuffer {
 public:
  struct Point {
    float x { 0.0 };
    float y { 0.0 };
    float z { 0.0 };

    Point() { }

    Point(float x, float y, float z) :
     x(x), y(y), z(z) {
    }

    CPoint3D point() const { return CPoint3D(x, y, z); }
  };

  struct Color {
    float r { 0.0 };
    float g { 0.0 };
    float b { 0.0 };

    Color() { }

    Color(float r, float g, float b) :
     r(r), g(g), b(b) {
    }

    CPoint3D point() const { return CPoint3D(r, g, b); }
  };

  struct TexturePoint {
    float x { 0.0 };
    float y { 0.0 };

    TexturePoint() { }

    TexturePoint(float x, float y) :
     x(x), y(y) {
    }

    CPoint2D point() const { return CPoint2D(x, y); }
  };

  struct Vector {
    float x { 0.0 };
    float y { 0.0 };
    float z { 0.0 };
    float w { 0.0 };

    Vector() { }

    Vector(float x, float y, float z, float w) :
     x(x), y(y), z(z), w(w) {
    }

    CPoint4D point() const { return CPoint4D(x, y, z, w); }
  };

  struct IVector {
    int x { 0 };
    int y { 0 };
    int z { 0 };
    int w { 0 };

    IVector() { }

    IVector(int x, int y, int z, int w) :
     x(x), y(y), z(z), w(w) {
    }

    CPoint4D point() const { return CPoint4D(x, y, z, w); }
  };

  using Points        = std::vector<Point>;
  using Colors        = std::vector<Color>;
  using TexturePoints = std::vector<TexturePoint>;
  using Indices       = std::vector<int>;
  using BoneIds       = std::vector<IVector>;
  using BoneWeights   = std::vector<Vector>;

 public:
  enum Parts {
    POINT   = (1<<0),
    NORMAL  = (1<<1),
    COLOR   = (1<<2),
    TEXTURE = (1<<3),
    BONE    = (1<<4)
  };

 public:
  CQGLBuffer(QOpenGLShaderProgram *program=nullptr) {
    data_.program = program;

    data_.vObj         = new QOpenGLVertexArrayObject;
    data_.vertexBuffer = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
    data_.indBuffer    = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);

    initIds();
  }

 ~CQGLBuffer() {
    term();
  }

  CQGLBuffer(const CQGLBuffer &buffer) {
    initFrom(buffer);
  }

  CQGLBuffer &operator=(const CQGLBuffer &buffer) {
    term();

    initFrom(buffer);

    return *this;
  }

  //---

  QOpenGLShaderProgram *program() const { return data_.program; }
  void setProgram(QOpenGLShaderProgram *p) { data_.program = p; }

  //---

  bool hasPointPart  () const { return (data_.types & static_cast<unsigned int>(Parts::POINT  )); }
  bool hasNormalPart () const { return (data_.types & static_cast<unsigned int>(Parts::NORMAL )); }
  bool hasColorPart  () const { return (data_.types & static_cast<unsigned int>(Parts::COLOR  )); }
  bool hasTexturePart() const { return (data_.types & static_cast<unsigned int>(Parts::TEXTURE)); }
  bool hasBonesPart  () const { return (data_.types & static_cast<unsigned int>(Parts::BONE   )); }

  void disableTexturePart() { data_.types &= ~static_cast<unsigned int>(Parts::TEXTURE); }

  //---

  void clearAll() {
    data_.types = 0;

    delete [] data_.data;
    delete [] data_.indData;

    data_.data    = nullptr;
    data_.numData = 0;

    data_.indData    = nullptr;
    data_.numIndData = 0;

    data_.span = 0;

    data_.dataValid = false;

    data_.inds         .clear();
    data_.points       .clear();
    data_.normals      .clear();
    data_.colors       .clear();
    data_.texturePoints.clear();
    data_.boneIds      .clear();
    data_.boneWeights  .clear();

    data_.indices.clear();
    data_.indicesSet = false;
  }

  void clearInds         () { data_.inds         .clear(); data_.dataValid = false; }
  void clearPoints       () { data_.points       .clear(); data_.dataValid = false; }
  void clearNormals      () { data_.normals      .clear(); data_.dataValid = false; }
  void clearColors       () { data_.colors       .clear(); data_.dataValid = false; }
  void clearTexturePoints() { data_.texturePoints.clear(); data_.dataValid = false; }
  void clearBoneIds      () { data_.boneIds      .clear(); data_.dataValid = false; }
  void clearBoneWeights  () { data_.boneWeights  .clear(); data_.dataValid = false; }

  void clearBuffers() {
    clearInds(); clearPoints(); clearNormals(); clearColors(); clearTexturePoints();
    clearBoneIds(); clearBoneWeights();
  }

  //---

  void addInd(uint ind) {
    data_.inds.push_back(ind);
  }

  uint numInds() const { return data_.inds.size(); }

  int mapInd(uint ind) const {
    int i = -1;

    for (auto &ind1 : data_.inds) {
      ++i;

      if (int(ind) == ind1)
        return i;
    }

    return -1;
  }

  void addPoint(float x, float y, float z) {
    addPoint(Point(x, y, z));
  }

  void addPoint(const Point &p) {
    data_.types |= static_cast<unsigned int>(Parts::POINT);

    data_.points.push_back(p);

    data_.dataValid = false;
  }

  uint numPoints() const { return data_.points.size(); }

  void addNormal(float x, float y, float z) {
    addNormal(Point(x, y, z));
  }

  void addNormal(const Point &p) {
    data_.types |= static_cast<unsigned int>(Parts::NORMAL);

    data_.normals.push_back(p);

    data_.dataValid = false;
  }

  void addColor(const CRGBA &c) {
    addColor(Color(c.getRedF(), c.getGreenF(), c.getBlueF()));
  }

  void addColor(const QColor &c) {
    addColor(Color(c.redF(), c.greenF(), c.blueF()));
  }

  void addColor(float r, float g, float b) {
    addColor(Color(r, g, b));
  }

  void addColor(const Color &c) {
    data_.types |= static_cast<unsigned int>(Parts::COLOR);

    data_.colors.push_back(c);

    data_.dataValid = false;
  }

  void addTexturePoint(float x, float y) {
    addTexturePoint(TexturePoint(x, y));
  }

  void addTexturePoint(const TexturePoint &p) {
    data_.types |= static_cast<unsigned int>(Parts::TEXTURE);

    data_.texturePoints.push_back(p);

    data_.dataValid = false;
  }

  void addBoneIds(int i1, int i2, int i3, int i4) {
    data_.types |= static_cast<unsigned int>(Parts::BONE);

    data_.boneIds.push_back(IVector(i1, i2, i3, i4));

    data_.dataValid = false;
  }

  void addBoneWeights(double w1, double w2, double w3, double w4) {
    data_.types |= static_cast<unsigned int>(Parts::BONE);

    data_.boneWeights.push_back(Vector(w1, w2, w3, w4));

    data_.dataValid = false;
  }

  void addIndex(int i) {
    data_.indices.push_back(i);

    data_.indicesSet = true;
  }

  //---

  struct PointData {
    std::optional<uint>         ind;
    std::optional<Point>        point;
    std::optional<Point>        normal;
    std::optional<Color>        color;
    std::optional<TexturePoint> texturePoint;
    std::optional<IVector>      boneId;
    std::optional<Vector>       boneWeight;
  };

  void getPointData(int i, PointData &data) const {
    auto np = data_.points.size();
    assert(i < int(np));

    if (numInds() == np) data.ind = data_.inds[i];

    if (hasPointPart  ()) data.point        = data_.points[i];
    if (hasNormalPart ()) data.normal       = data_.normals[i];
    if (hasColorPart  ()) data.color        = data_.colors[i];
    if (hasTexturePart()) data.texturePoint = data_.texturePoints[i];

    if (hasBonesPart()) {
      data.boneId     = data_.boneIds[i];
      data.boneWeight = data_.boneWeights[i];
    }
  }

  //---

  void load() {
    initData();

    // bind the Vertex Array Object first, then bind and set vertex buffer(s),
    // and then configure vertex attributes(s).
    data_.vObj->bind();

    // send geometry data to buffer
    data_.vertexBuffer->bind();
    data_.vertexBuffer->setUsagePattern(QOpenGLBuffer::StaticDraw);
    data_.vertexBuffer->allocate(data_.data, int(data_.numData*sizeof(float)));
    //data_.vertexBuffer->release();

    // send indices data to buffer
    if (data_.indicesSet) {
      data_.indBuffer->bind();
      data_.indBuffer->setUsagePattern(QOpenGLBuffer::StaticDraw);
      data_.indBuffer->allocate(data_.indData, int(data_.numIndData*sizeof(int)));
      //data_.indBuffer->release();
    }

    int  vid  = 0;
    uint span = 0;

    // store points in vertex array (location 0)
    if (hasPointPart()) {
      data_.program->setAttributeArray(vid, reinterpret_cast<float *>(span*sizeof(float)),
                                       3, int(data_.span*sizeof(float)));
      data_.program->enableAttributeArray(vid++);
      span += 3;
    }

    // store normals in vertex array (location 1)
    if (hasNormalPart()) {
      data_.program->setAttributeArray(vid, reinterpret_cast<float *>(span*sizeof(float)),
                                       3, int(data_.span*sizeof(float)));
      data_.program->enableAttributeArray(vid++);
      span += 3;
    }

    // store colors in vertex array (location 2)
    if (hasColorPart()) {
      data_.program->setAttributeArray(vid, reinterpret_cast<float *>(span*sizeof(float)),
                                       3, int(data_.span*sizeof(float)));
      data_.program->enableAttributeArray(vid++);
      span += 3;
    }

    // store texture points in vertex array (location 3)
    if (hasTexturePart()) {
      data_.program->setAttributeArray(vid, reinterpret_cast<float *>(span*sizeof(float)),
                                       2, int(data_.span*sizeof(float)));
      data_.program->enableAttributeArray(vid++);
      span += 2;
    }

    // store bone ids and weights in vertex arrays (location 4 and 5)
    if (hasBonesPart()) {
      data_.program->setAttributeArray(vid, reinterpret_cast<float *>(span*sizeof(float)),
                                       4, int(data_.span*sizeof(float)));
      data_.program->enableAttributeArray(vid++);
      span += 4;

      data_.program->setAttributeArray(vid, reinterpret_cast<float *>(span*sizeof(float)),
                                       4, int(data_.span*sizeof(float)));
      data_.program->enableAttributeArray(vid++);
      span += 4;
    }

    // note that this is allowed, the call to setAttributeBuffer registered VBO as the
    // vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    data_.vertexBuffer->release();
    if (data_.indicesSet)
      data_.indBuffer->release();

    // remember: do NOT unbind the EBO while a VAO is active as the bound element buffer object
    // IS stored in the VAO; keep the EBO bound.
    //data_.vertexBuffer->release();

    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO,
    // but this rarely happens. Modifying other VAOs requires a call to glBindVertexArray
    // anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    data_.vObj->release();
  }

  //---

  void bind() {
    assert(data_.dataValid);

    // seeing as we only have a single VAO there's no need to bind it every time,
    // but we'll do so to keep things a bit more organized
    data_.vObj->bind();

    if (data_.indicesSet)
      data_.indBuffer->bind();
  }

  void unbind() {
    if (data_.indicesSet)
      data_.indBuffer->release();

    data_.vObj->release();
  }

  //---

  void drawTriangles() {
    glDrawArrays(GL_TRIANGLES, 0, int(numPoints()));
  }

  //---

  CBBox3D getBBox() const {
    CBBox3D bbox;

    for (const auto &p : data_.points)
      bbox.add(CPoint3D(p.x, p.y, p.z));

    return bbox;
  }

 private:
  void term() {
    data_.vertexBuffer->destroy();
    data_.indBuffer   ->destroy();

    data_.vObj->destroy();

    delete data_.vObj;
    delete data_.vertexBuffer;
    delete data_.indBuffer;

    delete [] data_.data;
    delete [] data_.indData;

    data_ = Data();
  }

  void initFrom(const CQGLBuffer &buffer) {
    data_ = buffer.data_;

    if (buffer.data_.numData) {
      data_.data = new float [buffer.data_.numData];

      memcpy(data_.data, buffer.data_.data, buffer.data_.numData*sizeof(float));
    }
    else
      data_.data = nullptr;

    if (buffer.data_.numIndData) {
      data_.indData = new int [buffer.data_.numIndData];

      memcpy(data_.indData, buffer.data_.indData, buffer.data_.numIndData*sizeof(int));
    }
    else
      data_.indData = nullptr;

    initIds();
  }

  void initIds() {
    data_.vObj->create();

    data_.vertexBuffer->create();
    data_.indBuffer   ->create();
  }

  void initData() {
    if (! data_.dataValid) {
      delete [] data_.data;
      delete [] data_.indData;

      //---

      data_.numData = 0;
      data_.span    = 0;

      if (hasPointPart()) {
        data_.numData += data_.points.size()*3;
        data_.span += 3;
      }

      if (hasNormalPart()) {
        data_.numData += data_.normals.size()*3;
        data_.span += 3;
      }

      if (hasColorPart()) {
        assert(data_.colors.size() == numPoints());
        data_.numData += data_.colors.size()*3;
        data_.span += 3;
      }

      if (hasTexturePart()) {
        assert(data_.texturePoints.size() == numPoints());
        data_.numData += data_.texturePoints.size()*2;
        data_.span += 2;
      }

      if (hasBonesPart()) {
        assert(data_.boneIds.size() == numPoints());
        data_.numData += data_.boneIds.size()*4;
        data_.span += 4;

        assert(data_.boneWeights.size() == numPoints());
        data_.numData += data_.boneWeights.size()*4;
        data_.span += 4;
      }

      data_.data = new float [data_.numData];

      int  i  = 0;
      auto np = numPoints();

      for (size_t ip = 0; ip < np; ++ip) {
        if (hasPointPart()) {
          const auto &p = data_.points[ip];

          data_.data[i++] = p.x;
          data_.data[i++] = p.y;
          data_.data[i++] = p.z;
        }

        if (hasNormalPart()) {
          const auto &p = data_.normals[ip];

          data_.data[i++] = p.x;
          data_.data[i++] = p.y;
          data_.data[i++] = p.z;
        }

        if (hasColorPart()) {
          const auto &c = data_.colors[ip];

          data_.data[i++] = c.r;
          data_.data[i++] = c.g;
          data_.data[i++] = c.b;
        }

        if (hasTexturePart()) {
          const auto &p = data_.texturePoints[ip];

          data_.data[i++] = p.x;
          data_.data[i++] = p.y;
        }

        if (hasBonesPart()) {
          const auto &p = data_.boneIds[ip];

          data_.data[i++] = p.x;
          data_.data[i++] = p.y;
          data_.data[i++] = p.z;
          data_.data[i++] = p.w;

          const auto &w = data_.boneWeights[ip];

          data_.data[i++] = w.x;
          data_.data[i++] = w.y;
          data_.data[i++] = w.z;
          data_.data[i++] = w.w;
        }
      }

      //---

      data_.numIndData = uint(data_.indices.size());

      data_.indData = new int [data_.numIndData];

      i = 0;

      for (const auto &ind : data_.indices)
        data_.indData[i++] = ind;

      //---

      data_.dataValid = true;
    }
  }

 private:
  struct Data {
    QOpenGLShaderProgram *program { nullptr };

    QOpenGLVertexArrayObject* vObj         { nullptr };
    QOpenGLBuffer*            vertexBuffer { nullptr };
    QOpenGLBuffer*            indBuffer    { nullptr };

    unsigned int  types      { 0 };
    float*        data       { nullptr };
    unsigned int  numData    { 0 };
    int*          indData    { nullptr };
    unsigned int  numIndData { 0 };
    unsigned int  span       { 0 };
    bool          dataValid  { false };
    Indices       inds;                    // vertex inds
    Points        points;                  // vertex point
    Points        normals;                 // vertex normal
    Colors        colors;                  // vertex color
    TexturePoints texturePoints;           // vertex texture point
    BoneIds       boneIds;                 // vertex bone id
    BoneWeights   boneWeights;             // vertex bone weight
    Indices       indices;                 // vertex point indices
    bool          indicesSet { false };    // is vertex point indices set
  };

  Data data_;
};

#endif
