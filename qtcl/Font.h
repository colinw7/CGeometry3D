#ifndef Font_H
#define Font_H

#include <CGLVector2D.h>
#include <CGLVector3D.h>
#include <CMatrix3DH.h>
#include <CBBox3D.h>
#include <CRGBA.h>

#include <vector>
#include <cstdint>

using uchar = unsigned char;

class CQGLBuffer;

namespace CQTclModel3DView {

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

#if 0
  struct RotatingLabel {
    uint  vao = 0;
    uint  vertexBuffer = 0;
    uint  uvBuffer = 0;
    uint  colBuffer = 0;
    uint  indexBuffer = 0;
    uint  indexElementCount = 0;
    float angle { 0.0f };
  };
#endif

 public:
  Font(Canvas *canvas);

  void init();

  const std::string &fontName() { return name_; }
  bool setFontName(const std::string &name);

  double aspect() const { return aspect_; }
  void setAspect(double r) { aspect_ = r; }

  double size() const { return size_; }
  void setSize(double s);

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
  std::string    name_;
  double         size_          { 40 };
  double         aspect_        { 1.0 };
  ShaderProgram* shaderProgram_ { nullptr };
  FontData*      fontData_      { nullptr };
};

//---

class Text {
 public:
  enum class HAlign {
    LEFT,
    RIGHT,
    CENTER
  };

  enum class VAlign {
    BOTTOM,
    TOP,
    CENTER
  };

  Text(const std::string &text="");

  const uint &id() const { return id_; }
  void setId(const uint &v) { id_ = v; }

  const std::string &text() const { return text_; }
  void setText(const std::string &s) { text_ = s; }

  const Font *font() const { return font_; }
  void setFont(Font *p) { font_ = p; }

  const CRGBA &color() const { return color_; }
  void setColor(const CRGBA &v) { color_ = v; }

  const CGLVector3D &position() const { return position_; }
  void setPosition(const CGLVector3D &v) { position_ = v; }

  const CGLVector3D &angle() const { return angle_; }
  void setAngle(const CGLVector3D &v) { angle_ = v; }

  double size() const { return size_; }
  void setSize(double r) { size_ = r; }

  bool isOverlay() const { return overlay_; }
  void setOverlay(bool b) { overlay_ = b; }

  bool isBillboard() const { return billboard_; }
  void setBillboard(bool b) { billboard_ = b; }

  int viewport() const { return viewport_; }
  void setViewport(int i) { viewport_ = i; }

  const HAlign &halign() const { return halign_; }
  void setHAlign(const HAlign &a) { halign_ = a; }

  const VAlign &valign() const { return valign_; }
  void setVAlign(const VAlign &a) { valign_ = a; }

  bool isVisible() const { return visible_; }
  void setVisible(bool b) { visible_ = b; }

  void updateText();

  void render(Canvas *canvas);

 private:
  void initBuffer();

  CMatrix3DH getModelMatrix() const;

 private:
//using RotatingLabel = Font::RotatingLabel;

  uint        id_        { 0 };
  std::string text_;
  Font*       font_      { nullptr };
  CRGBA       color_     { 1, 1, 1 };
  CGLVector3D position_;
  CGLVector3D angle_     { 0, 0, 0 };
  double      size_      { 0.1 };
  bool        overlay_   { false };
  bool        billboard_ { true };
  bool        visible_   { true };
  int         viewport_  { -1 };
  HAlign      halign_    { HAlign::LEFT };
  VAlign      valign_    { VAlign::CENTER };

  CQGLBuffer *buffer_ { nullptr };

  mutable CBBox3D bbox_;

#if 0
  std::vector<CGLVector3D> vertices_;
  std::vector<CGLVector2D> uvs_;
  std::vector<CRGBA>       colors_;
  std::vector<uint16_t>    indexes_;
#endif

//RotatingLabel rotatingLabel_;

//int indexElementCount_ { 0 };
};

}

#endif
