#include <CQGLTexture.h>
#include <CQImage.h>
#include <CMathGen.h>

#include <QFileInfo>
#include <QImageReader>
#include <iostream>

#if 0
#include <glad/glad.h>
#endif
#include <GL/glut.h>

namespace {

bool checkError(const char *msg) {
  // check texture generated
  GLenum err = glGetError();

  if (err != GL_NO_ERROR) {
    std::cerr << "OpenGL Error: " << gluErrorString(err) << "(" << msg << ")\n";
    return false;
  }

  return true;
}

}

//---

CQGLTexture::
CQGLTexture()
{
}

CQGLTexture::
CQGLTexture(const QImage &image)
{
  setImage(image);
}

CQGLTexture::
CQGLTexture(const CImagePtr &image)
{
  setImage(image);
}

CQGLTexture::
~CQGLTexture()
{
  if (valid_ && glIsTexture(id_))
    glDeleteTextures(1, &id_);
}

bool
CQGLTexture::
load(const QString &fileName, bool flip)
{
  QFileInfo fi(fileName);

  if (! fi.exists(fileName)) {
    std::cerr << "Error: Invalid texture file '" << fileName.toStdString() << "'\n";
    return false;
  }

  QImageReader imageReader(fileName);

  QImage image;

  if (! imageReader.read(&image))
    return false;

  if (image.isNull()) {
    std::cerr << "Error: Failed to read image from '" << fileName.toStdString() << "'\n";
    return false;
  }

  return load(image, flip);
}

bool
CQGLTexture::
load(const QImage &image, bool flip)
{
  return init(image, flip);
}

void
CQGLTexture::
setImage(const QImage &image)
{
  init(image, /*flip*/false);
}

void
CQGLTexture::
setImage(const CImagePtr &image)
{
  QImage &qimage = dynamic_cast<CQImage *>(image.get())->getQImage();

  init(qimage, /*flip*/false);
}

bool
CQGLTexture::
setTarget(int w, int h)
{
  if (! valid_ || w != targetWidth_ ||  h != targetHeight_) {
    targetWidth_  = w;
    targetHeight_ = h;

    // The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
    if (frameBufferId_ == 0)
      functions_->glGenFramebuffers(1, &frameBufferId_);

    functions_->glBindFramebuffer(GL_FRAMEBUFFER, frameBufferId_);
    if (! checkError("glBindFramebuffer")) return false;

    // The texture we're going to render to
    if (id_ == 0) {
      glGenTextures(1, &id_);
      if (! checkError("glGenTextures")) return false;
    }

    // generate texture
    glBindTexture(GL_TEXTURE_2D, id_);
    if (! checkError("glBindTexture")) return false;

    // Give an empty image to OpenGL ( the last "0" )
    // no difference for GL_RGBA and GL_RGB
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, targetWidth_, targetHeight_,
                 /*border*/0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    if (! checkError("glTexImage2D")) return false;

    // Poor filtering (need min filter to avoid mip map use - not set)
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

//  glBindTexture(GL_TEXTURE_2D, 0);
//  if (! checkError("glBindTexture")) return false;

    if (depthRenderBuffer_ == 0)
      functions_->glGenRenderbuffers(1, &depthRenderBuffer_);

    functions_->glBindRenderbuffer(GL_RENDERBUFFER, depthRenderBuffer_);
    functions_->glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24,
                                      targetWidth_, targetHeight_);
//  functions_->glBindRenderbuffer(GL_RENDERBUFFER, 0);

    functions_->glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                          GL_RENDERBUFFER, depthRenderBuffer_);

    // attach it to currently bound framebuffer object
    //functions_->glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, id_, 0);
    functions_->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, id_, 0);

    // generate render buffer
    // Set the list of draw buffers.
    GLenum drawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
    functions_->glDrawBuffers(1, drawBuffers); // "1" is the size of DrawBuffers

    if (functions_->glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      std::cerr << "Framebuffer error\n";
      return false;
    }

    valid_ = true;
  }

  return true;
}

bool
CQGLTexture::
init(const QImage &image, bool flip)
{
  if (image.isNull()) {
    std::cerr << "Invalid image data\n";
    return false;
  }

  if (useAlpha())
    image_ = image.convertToFormat(QImage::Format_RGBA8888);
  else
    image_ = image.convertToFormat(QImage::Format_RGB888);

//if (flip)
//  image_ = image_.flippedH();

  //------

  // convert to GL compatible data
  width_  = image_.width ();
  height_ = image_.height();

  imageData_ = new unsigned char [4*width_*height_];

  int i = 0;

  for (int y = 0; y < height_; ++y) {
    int y1 = (flip ? height_ - 1 - y : y);

    for (int x = 0; x < width_; ++x) {
      auto rgba = image_.pixel(x, y1);

      imageData_[i++] = qBlue (rgba);
      imageData_[i++] = qGreen(rgba);
      imageData_[i++] = qRed  (rgba);
      imageData_[i++] = qAlpha(rgba);
    }
  }

  //------

  //glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  // allocate texture id
  glGenTextures(1, &id_);
  if (! checkError("glGenTextures")) return false;

  valid_ = true;

  // set texture type
  glBindTexture(GL_TEXTURE_2D, id_);
  if (! checkError("glBindTexture")) return false;

  if (wrapType() == WrapType::CLAMP) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  }
  else {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  }
  if (! checkError("glTexParameteri")) return false;

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  if (! checkError("glTexParameteri")) return false;

  // select modulate to mix texture with color for shading
  //glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
  //if (! checkError("glTexEnvf")) return false;

  // build our texture mipmaps
  GLint internalFormat = (useAlpha() ? GL_RGBA : GL_RGB);

  if (CMathGen::isPowerOf(2, width_) && CMathGen::isPowerOf(2, height_)) {
#if 1
    // Hardware mipmap generation
    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
    if (! checkError("glTexParameteri")) return false;

    glHint(GL_GENERATE_MIPMAP_HINT_SGIS, GL_NICEST);
    if (! checkError("glHint")) return false;
#endif

    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, width_, height_, 0,
                 GL_BGRA, GL_UNSIGNED_BYTE, &imageData_[0]);
    if (! checkError("glTexImage2D")) return false;

#if 0
    glGenerateMipmap(GL_TEXTURE_2D);
    if (! checkError("glGenerateMipmap")) return false;
#endif
  }
  else {
    // No hardware mipmap generation support, fall back to the
    // good old gluBuild2DMipmaps function
    gluBuild2DMipmaps(GL_TEXTURE_2D, internalFormat, width_, height_,
                      GL_BGRA, GL_UNSIGNED_BYTE, &imageData_[0]);
    if (! checkError("gluBuild2DMipmaps")) return false;
  }

  return true;
}

void
CQGLTexture::
bind() const
{
  glEnable(GL_TEXTURE_2D);

  if (frameBufferId_ > 0) {
    glBindTexture(GL_TEXTURE_2D, 0);

    functions_->glBindFramebuffer(GL_FRAMEBUFFER, frameBufferId_);
    functions_->glViewport(0, 0, targetWidth_, targetHeight_);

    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  }
  else {
    glBindTexture(GL_TEXTURE_2D, id_);
  }
}

void
CQGLTexture::
unbind() const
{
  if (frameBufferId_ > 0) {
    functions_->glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }
  else
    glBindTexture(GL_TEXTURE_2D, 0);
}

void
CQGLTexture::
bindBuffer() const
{
  glEnable(GL_TEXTURE_2D);

  if (frameBufferId_)
    glBindTexture(GL_TEXTURE_2D, id_);
}

void
CQGLTexture::
unbindBuffer() const
{
  if (frameBufferId_)
    glBindTexture(GL_TEXTURE_2D, 0);
}

void
CQGLTexture::
enable(bool b)
{
  if (b)
    glEnable(GL_TEXTURE_2D);
  else
    glDisable(GL_TEXTURE_2D);
}

void
CQGLTexture::
draw()
{
  draw(0.0, 0.0, 1.0, 1.0);
}

void
CQGLTexture::
draw(double x1, double y1, double x2, double y2)
{
  draw(x1, y1, 0, x2, y2, 0, 0, 0, 1, 1);
}

void
CQGLTexture::
draw(double x1, double y1, double z1, double x2, double y2, double z2,
     double tx1, double ty1, double tx2, double ty2)
{
  //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  enable(true);

  bind();

  // select modulate to mix texture with color for shading
  //glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

  glBegin(GL_QUADS);

  if      (fabs(z2 - z1) < 1E-3) {
    glTexCoord2d(tx1, ty1); glVertex3d(x1, y1, z1);
    glTexCoord2d(tx1, ty2); glVertex3d(x1, y2, z1);
    glTexCoord2d(tx2, ty2); glVertex3d(x2, y2, z1);
    glTexCoord2d(tx2, ty1); glVertex3d(x2, y1, z1);
  }
  else if (fabs(y2 - y1) < 1E-3) {
    glTexCoord2d(tx1, ty1); glVertex3d(x1, y1, z1);
    glTexCoord2d(tx1, ty2); glVertex3d(x1, y1, z2);
    glTexCoord2d(tx2, ty2); glVertex3d(x2, y1, z2);
    glTexCoord2d(tx2, ty1); glVertex3d(x2, y1, z1);
  }
  else {
    glTexCoord2d(tx1, ty1); glVertex3d(x1, y1, z1);
    glTexCoord2d(tx1, ty2); glVertex3d(x1, y2, z1);
    glTexCoord2d(tx2, ty2); glVertex3d(x1, y2, z2);
    glTexCoord2d(tx2, ty1); glVertex3d(x1, y1, z2);
  }

  glEnd();

  enable(false);
}

#if 0
void
CQGLTexture::
displayFramebufferTexture(ShaderProgram *program, int vertexId)
{
  if (! notInitialized) {
    // initialize shader and vao w/ NDC vertex coordinates at top-right of the screen
    [...]
  }

  glActiveTexture(GL_TEXTURE0);

  program->bind();

  bind();

  glBindVertexArray(vertexId);

  glDrawArrays(GL_TRIANGLES, 0, 6);

  glBindVertexArray(0);

  unbind();

  program->unbind();
}
#endif
