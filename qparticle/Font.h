#ifndef Font_H
#define Font_H

#include <CGLVector2D.h>
#include <CGLVector3D.h>
#include <CMatrix3DH.h>
#include <CBBox3D.h>

#include <QString>

#include <vector>
#include <cstdint>

class CQGLBuffer;

namespace CQTclParticle3D {

class  Canvas;
struct FontData;
class  ShaderProgram;

class Font {
 public:
  struct GlyphInfo {
    CGLVector3D positions[4];
    CGLVector2D uvs[4];
    float       offsetX { 0 };
    float       offsetY { 0 };
  };

  struct Color {
    Color() = default;

    Color(float r_, float g_, float b_) :
     r(r_), g(g_), b(b_) {
    }

    float r { 0.0f };
    float g { 0.0f };
    float b { 0.0f };
  };

  struct RotatingLabel {
    uint  vao = 0;
    uint  vertexBuffer = 0;
    uint  uvBuffer = 0;
    uint  colBuffer = 0;
    uint  indexBuffer = 0;
    uint  indexElementCount = 0;
    float angle { 0.0f };
  };

 public:
  Font(Canvas *canvas);

  void init();

  const QString &fontName() { return name_; }
  bool setFontName(const QString &name);

  double aspect() const { return aspect_; }
  void setAspect(double r) { aspect_ = r; }

  double size() const { return size_; }
  void setSize(double s);

  bool isBillboard() const { return billboard_; }
  void setBillboard(bool b) { billboard_ = b; }

  GlyphInfo makeGlyphInfo(uint32_t character, float offsetX, float offsetY) const;

  Canvas *canvas() const { return canvas_; }

  ShaderProgram *shaderProgram() const { return shaderProgram_; }

  bool bindTexture();

  int textureId() const;

 private:
  bool updateFontData();

  bool createFontTexture(uint *texture, int w, int h, uchar *data);

  bool readFile(const char *path, std::vector<uint8_t> &bytes) const;

 private:
  Canvas*        canvas_        { nullptr };
  QString        name_;
  double         size_          { 40 };
  double         aspect_        { 0.1 };
  ShaderProgram* shaderProgram_ { nullptr };
  FontData*      fontData_      { nullptr };
  bool           billboard_     { true };
};

//---

class Text {
 public:
  using Color = Font::Color;

 public:
  Text(const QString &text="");

  const QString &text() const { return text_; }
  void setText(const QString &s) { text_ = s; }

  const Font *font() const { return font_; }
  void setFont(Font *p) { font_ = p; }

  const Color &color() const { return color_; }
  void setColor(const Color &v) { color_ = v; }

  const CGLVector3D &position() const { return position_; }
  void setPosition(const CGLVector3D &v) { position_ = v; }

  const CGLVector3D &angle() const { return angle_; }
  void setAngle(const CGLVector3D &v) { angle_ = v; }

  double size() const { return size_; }
  void setSize(double r) { size_ = r; }

  bool isOverlay() const { return overlay_; }
  void setOverlay(bool b) { overlay_ = b; }

  const Qt::Alignment &align() const { return align_; }
  void setAlign(const Qt::Alignment &v) { align_ = v; }

  void updateText();

  void render(Canvas *canvas);

 private:
  void initBuffer();

  CMatrix3DH getModelMatrix() const;

 private:
//using RotatingLabel = Font::RotatingLabel;

  QString       text_;
  Font*         font_    { nullptr };
  Color         color_   { 1, 1, 1 };
  CGLVector3D   position_;
  CGLVector3D   angle_   { 0, 0, 0 };
  double        size_    { 0.1 };
  bool          overlay_ { false };
  Qt::Alignment align_   { Qt::AlignLeft | Qt::AlignVCenter };

  CQGLBuffer *buffer_ { nullptr };

  mutable CBBox3D bbox_;

#if 0
  std::vector<CGLVector3D> vertices_;
  std::vector<CGLVector2D> uvs_;
  std::vector<Color>       colors_;
  std::vector<uint16_t>    indexes_;
#endif

//RotatingLabel rotatingLabel_;

//int indexElementCount_ { 0 };
};

}

#endif
