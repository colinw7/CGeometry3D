#include <Font.h>
#include <Canvas.h>
#include <ShaderProgram.h>
#include <Camera.h>
#include <Canvas.h>

#include <CQGLBuffer.h>
#include <CQGLUtil.h>
#include <CGLVector2D.h>
#include <CGLVector3D.h>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>

#include <memory>

#define STB_TRUETYPE_IMPLEMENTATION
#include <stb/stb_truetype.h>

#include <fstream>

namespace CQTclParticle3D {

struct FontData {
  uint32_t                            size = 40;
  uint32_t                            atlasWidth = 1024;
  uint32_t                            atlasHeight = 1024;
  uint32_t                            oversampleX = 2;
  uint32_t                            oversampleY = 2;
  uint32_t                            firstChar = ' ';
  uint32_t                            charCount = '~' - ' ';
  std::unique_ptr<stbtt_packedchar[]> charInfo;
  GLuint                              texture = 0;
};

//----

Font::
Font(Canvas *canvas) :
 canvas_(canvas)
{
}

void
Font::
init()
{
  shaderProgram_ = new ShaderProgram(canvas_->app());

  shaderProgram_->addShaders("font.vs", "font.fs");
}

bool
Font::
setFontName(const QString &name)
{
  name_ = name;

  return updateFontData();
}

bool
Font::
updateFontData()
{
  if (name_ == "")
    return true;

  //---

  delete fontData_;

  fontData_ = new FontData;

  fontData_->size = size_;

  //---

  // get true type font data
  auto path = canvas_->app()->buildDir() + "/fonts/" + name_;

  std::vector<uint8_t> fontData;

  if (! readFile(path.toLatin1().constData(), fontData))
    return false;

  auto atlasData = std::make_unique<uint8_t[]>(fontData_->atlasWidth*fontData_->atlasHeight);

  fontData_->charInfo = std::make_unique<stbtt_packedchar[]>(fontData_->charCount);

  stbtt_pack_context context;
  if (! stbtt_PackBegin(&context, atlasData.get(), fontData_->atlasWidth,
                        fontData_->atlasHeight, 0, 1, nullptr))
    assert(false);

  stbtt_PackSetOversampling(&context, fontData_->oversampleX, fontData_->oversampleY);

  if (! stbtt_PackFontRange(&context, fontData.data(), 0, size(), fontData_->firstChar,
                            fontData_->charCount, fontData_->charInfo.get()))
    assert(false);

  stbtt_PackEnd(&context);

  //---

#if 0
//glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  // allocate texture id
  glGenTextures(1, &fontData_->texture);
  if (! checkError("glGenTextures")) return false;

  // set texture type
  glBindTexture(GL_TEXTURE_2D, fontData_->texture);
  if (! checkError("glBindTexture")) return false;

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  if (! checkError("glTexParameteri")) return false;

#if 0
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST_MIPMAP_NEAREST);
#else
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
#endif
  if (! checkError("glTexParameteri")) return false;

  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 8);

  glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
  if (! checkError("glTexParameteri")) return false;

  glHint(GL_GENERATE_MIPMAP_HINT, GL_NICEST);
  if (! checkError("glHint")) return false;

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, fontData_->atlasWidth, fontData_->atlasHeight, 0,
               GL_RED, GL_UNSIGNED_BYTE, atlasData.get());
  if (! checkError("glTexImage2D")) return false;

  canvas_->glGenerateMipmap(GL_TEXTURE_2D);
  if (! checkError("glGenerateMipmap")) return false;
#else
  if (! createFontTexture(&fontData_->texture,
                          fontData_->atlasWidth, fontData_->atlasHeight,
                          atlasData.get()))
    return false;
#endif

  return true;
}

bool
Font::
createFontTexture(uint *texture, int w, int h, uchar *data)
{
  //glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  // allocate texture id
  glGenTextures(1, texture);
  //if (! checkError("glGenTextures")) return false;

  // set texture type
  glBindTexture(GL_TEXTURE_2D, *texture);
  //if (! checkError("glBindTexture")) return false;

#if 0
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  if (! checkError("glTexParameteri")) return false;

#if 0
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST_MIPMAP_NEAREST);
#else
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
#endif
  if (! checkError("glTexParameteri")) return false;

  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 8);

  glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
  if (! checkError("glTexParameteri")) return false;
#else
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  //if (! checkError("glPixelStorei")) return false;
#endif

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RED, GL_UNSIGNED_BYTE, data);
  //if (! checkError("glTexImage2D")) return false;

  glHint(GL_GENERATE_MIPMAP_HINT, GL_NICEST);
  //if (! checkError("glHint")) return false;

  canvas_->glGenerateMipmap(GL_TEXTURE_2D);
  //if (! checkError("glGenerateMipmap")) return false;

  //---

  #if 0
  // dump font to png
  auto src = CImageNoSrc();

  auto image = CImageMgrInst->createImage(src);

  image->setDataSize(w, h);

  int ii = 0;

  for (int iy = 0; iy < h; ++iy) {
    for (int ix = 0; ix < w; ++ix, ++ii) {
      image->setRGBAPixel(ix, iy, 1.0, 1.0, 1.0, data[ii]/255.0);
    }
  }

  CFile file("texture.png");

  image->writePNG(&file);
#endif

  return true;
}

void
Font::
setSize(double s)
{
  size_ = s;

  (void) updateFontData();
}

Font::GlyphInfo
Font::
makeGlyphInfo(uint32_t character, float offsetX, float offsetY) const
{
  stbtt_aligned_quad quad;

  stbtt_GetPackedQuad(fontData_->charInfo.get(), fontData_->atlasWidth, fontData_->atlasHeight,
                      character - fontData_->firstChar, &offsetX, &offsetY, &quad, 1);

  const auto xmin =  quad.x0;
  const auto ymin = -quad.y1;
  const auto xmax =  quad.x1;
  const auto ymax = -quad.y0;

  GlyphInfo info{};

  info.offsetX = offsetX;
  info.offsetY = offsetY;

  info.positions[0] = CGLVector3D(xmin, ymin, 0.0f);
  info.positions[1] = CGLVector3D(xmin, ymax, 0.0f);
  info.positions[2] = CGLVector3D(xmax, ymax, 0.0f);
  info.positions[3] = CGLVector3D(xmax, ymin, 0.0f);

#if 1
  info.uvs[0] = CGLVector2D(quad.s0, quad.t1);
  info.uvs[1] = CGLVector2D(quad.s0, quad.t0);
  info.uvs[2] = CGLVector2D(quad.s1, quad.t0);
  info.uvs[3] = CGLVector2D(quad.s1, quad.t1);
#else
  info.uvs[0] = CGLVector2D(0.0, 1.0);
  info.uvs[1] = CGLVector2D(0.0, 0.0);
  info.uvs[2] = CGLVector2D(1.0, 0.0);
  info.uvs[3] = CGLVector2D(1.0, 1.0);
#endif

  return info;
}

bool
Font::
readFile(const char *path, std::vector<uint8_t> &bytes) const
{
  std::ifstream file(path, std::ios::binary | std::ios::ate);

  if (! file.is_open())
    return false;

  const auto size = file.tellg();

  file.seekg(0, std::ios::beg);
  bytes = std::vector<uint8_t>(size);
  file.read(reinterpret_cast<char *>(&bytes[0]), size);
  file.close();

  return true;
}

bool
Font::
bindTexture()
{
  glEnable(GL_TEXTURE_2D);
  //if (! checkError("glEnable")) return false;

  glBindTexture(GL_TEXTURE_2D, textureId());
  //if (! checkError("glBindTexture")) return false;

#if 0
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST_MIPMAP_NEAREST);
#else
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
#endif
  //if (! checkError("glTexParameteri")) return false;

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  //if (! checkError("glTexParameteri")) return false;

  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 8);
  //if (! checkError("glTexParameterf")) return false;

  glActiveTexture(GL_TEXTURE0);
  //if (! checkError("glActiveTexture")) return false;

  return true;
}

int
Font::
textureId() const
{
  return fontData_->texture;
}

//---

Text::
Text(const QString &text) :
 text_(text)
{
}

void
Text::
updateText()
{
  initBuffer();

  //---

//uint16_t lastIndex = 0;

  float offsetX = 0, offsetY = 0;

  double a = font_->aspect();
  double f = 1.0/font_->size();

  for (const auto &c : text_) {
    const auto glyphInfo = font_->makeGlyphInfo(c.toLatin1(), offsetX, offsetY);

    offsetX = glyphInfo.offsetX;
    offsetY = glyphInfo.offsetY;

    auto addPos = [&](int i) {
      // x, y, z
      auto pos = glyphInfo.positions[i];

      pos.scaleX(f/a);
      pos.scaleY(f);

      //pos += position();

      buffer_->addPoint(pos.x(), pos.y(), pos.z());

      // u, v
      buffer_->addTexturePoint(glyphInfo.uvs[i].x(), glyphInfo.uvs[i].y());

      // color
      buffer_->addColor(color().r, color().g, color().b);

      bbox_.add(pos.point());
    };

#if 0
    addPos(0);
    addPos(1);
    addPos(2);
    addPos(3);

    //---

    buffer_->addIndex(lastIndex);
    buffer_->addIndex(lastIndex + 1);
    buffer_->addIndex(lastIndex + 2);

    buffer_->addIndex(lastIndex);
    buffer_->addIndex(lastIndex + 2);
    buffer_->addIndex(lastIndex + 3);

    lastIndex += 4;
#else
    addPos(0);
    addPos(1);
    addPos(2);

    addPos(0);
    addPos(2);
    addPos(3);
#endif
  }

  buffer_->load();

//indexElementCount_ = 6*text_.length();
}

void
Text::
initBuffer()
{
  if (! buffer_) {
    auto *program = font_->shaderProgram();

    buffer_ = program->createBuffer();
  }

  buffer_->clearAll();
}

void
Text::
render(Canvas *canvas)
{
  glDepthFunc(GL_LEQUAL);
  //(void) checkError("glDepthFunc");

  glEnable(GL_BLEND);
  //glBlendFunc(GL_SRC_ALPHA, GL_ONE);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  //if (! checkError("glBlendFunc GL_SRC_ALPHA, GL_ONE")) return;
  //glDepthMask(GL_FALSE);

  //---

  auto *program = font_->shaderProgram();

  program->bind();

  buffer_->bind();

  //---

//auto mm1 = canvas->getModelMatrix();
  auto mm1 = CMatrix3DH::identity();
  auto mm2 = getModelMatrix();

  //---

  CVector3D up, right;

  program->setUniformValue("billboard", font_->isBillboard());

  if (! isOverlay()) {
    auto *camera = canvas->camera();

    // camera projection
    auto projectionMatrix = camera->worldMatrix();
    program->setUniformValue("projection", CQGLUtil::toQMatrix(projectionMatrix));

    // camera/view transformation
    auto viewMatrix = camera->viewMatrix();
    program->setUniformValue("view", CQGLUtil::toQMatrix(viewMatrix));

    program->setUniformValue("viewPos", CQGLUtil::toVector(camera->position()));

    up    = camera->up   ();
    right = camera->right();
  }
  else {
    // camera projection
    auto projectionMatrix = CMatrix3DH::identity();
    program->setUniformValue("projection", CQGLUtil::toQMatrix(projectionMatrix));

    // camera/view transformation
    auto viewMatrix = CMatrix3DH::identity();
    program->setUniformValue("view", CQGLUtil::toQMatrix(viewMatrix));

    auto viewPos = CVector3D(0, 0, 0);
    program->setUniformValue("viewPos", CQGLUtil::toVector(viewPos));

    up    = CVector3D(0, 1, 0);
    right = CVector3D(1, 0, 0);
  }

  program->setUniformValue("cameraUp"   , CQGLUtil::toVector(up   ));
  program->setUniformValue("cameraRight", CQGLUtil::toVector(right));

  //---

  auto pos = position().vector();

  if      (align_ & Qt::AlignHCenter)
    pos -= (size()*bbox_.getXSize()/2.0)*right;
  else if (align_ & Qt::AlignRight)
    pos -= (size()*bbox_.getXSize())*right;

  if      (align_ & Qt::AlignVCenter)
    pos += (size()*bbox_.getYSize()/2.0)*up;
  else if (align_ & Qt::AlignBottom)
    pos += (size()*bbox_.getYSize())*up;

  // model matrix
  program->setUniformValue("model", CQGLUtil::toQMatrix(mm1*mm2));

  program->setUniformValue("center", CQGLUtil::toVector(pos));
  program->setUniformValue("size", float(size()));

  program->setUniformValue("overlay", isOverlay());

  //---

  font_->bindTexture();

//program->setUniformValue("mainTex", font_->textureId());
  program->setUniformValue("mainTex", 0);

//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

//glDrawElements(GL_TRIANGLES, indexElementCount_, GL_UNSIGNED_SHORT, nullptr);
  glDrawArrays(GL_TRIANGLES, 0, int(buffer_->numPoints()));

  //---

  buffer_->unbind();

  program->release();

  glDisable(GL_BLEND);
  //glDepthMask(GL_TRUE);
}

CMatrix3DH
Text::
getModelMatrix() const
{
  auto modelMatrix = CMatrix3DH::identity();

  if (! font_->isBillboard()) {
    modelMatrix.translated(position().x(), position().y(), position().z());

    modelMatrix.rotated(angle().x(), CVector3D(1.0, 0.0, 0.0));
    modelMatrix.rotated(angle().y(), CVector3D(0.0, 1.0, 0.0));
    modelMatrix.rotated(angle().z(), CVector3D(0.0, 0.0, 1.0));

    modelMatrix.scaled(size(), size(), 1.0);
  }

  return modelMatrix;
}

}
