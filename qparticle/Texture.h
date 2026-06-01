#ifndef Texture_H
#define Texture_H

#include <CGeomTexture.h>
#include <CQGLTexture.h>

class CQGLTexture;

namespace CQTclParticle3D {

class Texture : public CGeomTexture {
 public:
  Texture() { }

  bool isFlipped() const { return flipped_; }
  void setFlipped(bool b) { flipped_ = b; }

  CQGLTexture *glTexture(Canvas *canvas) {
    auto &canvasTexture = canvasTextureData_[canvas->ind()];

    if (flipped_)
      return canvasTexture.glTextureFlipped;
    else
      return canvasTexture.glTexture;
  }

  bool isWrapped() const { return wrapped_; }

  void setWrapped(bool b) {
    wrapped_ = b;

    for (auto pw : canvasTextureData_) {
      auto &textureData = pw.second;

      if (textureData.glTexture)
        textureData.glTexture->setWrapType(wrapped_ ?
         CQGLTexture::WrapType::REPEAT : CQGLTexture::WrapType::CLAMP);

      if (textureData.glTextureFlipped)
        textureData.glTextureFlipped->setWrapType(wrapped_ ?
         CQGLTexture::WrapType::REPEAT : CQGLTexture::WrapType::CLAMP);
    }
  }

  void setGlTextures(Canvas *canvas, CQGLTexture *t1, CQGLTexture *t2) {
    auto &textureData = canvasTextureData_[canvas->ind()];

    textureData.glTexture        = t1;
    textureData.glTextureFlipped = t2;

    setWrapped(wrapped_);
  }

 private:
  struct TextureData {
    CQGLTexture* glTexture        { nullptr };
    CQGLTexture* glTextureFlipped { nullptr };
  };

  using CanvasTextureData = std::map<uint, TextureData>;

  bool flipped_ { false }; // TODO: per canvas

  CanvasTextureData canvasTextureData_;
  bool              wrapped_ { true };
};

}

#endif
