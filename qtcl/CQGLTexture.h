#ifndef CQGL_TEXTURE_H
#define CQGL_TEXTURE_H

#include <CImagePtr.h>
#include <QImage>
#include <QOpenGLExtraFunctions>

class CQGLTexture {
 public:
  enum class WrapType {
    CLAMP,
    REPEAT
  };

 public:
  CQGLTexture();
  CQGLTexture(const QImage &image);
  CQGLTexture(const CImagePtr &image);

 ~CQGLTexture();

  const WrapType &wrapType() const { return wrapType_; }
  void setWrapType(const WrapType &v) { wrapType_ = v; }

  bool useAlpha() const { return useAlpha_; }
  void setUseAlpha(bool b) { useAlpha_ = b; }

  bool isFlipped() const { return flipped_; }
  void setFlipped(bool b) { flipped_ = b; }

  bool load(const QString &fileName, bool flip=false);
  bool load(const QImage &image, bool flip=false);

  const QImage &getImage() const { return image_; }
  void setImage(const QImage &image);
  void setImage(const CImagePtr &image);

  int getWidth () const { return width_ ; }
  int getHeight() const { return height_; }

  uint getId() const { return id_; }

  const std::string &getName() const { return name_; }
  void setName(const std::string &s) { name_ = s; }

  bool setTarget(int w, int h);

  const QOpenGLExtraFunctions *functions() const { return functions_; }
  void setFunctions(QOpenGLExtraFunctions *p) { functions_ = p; }

  void bind() const;
  void unbind() const;

  void bindBuffer() const;
  void unbindBuffer() const;

  void enable(bool b);

  void draw();
  void draw(double x1, double y1, double x2, double y2);
  void draw(double x1, double y1, double z1, double x2, double y2, double z2,
            double tx1=0.0, double ty1=0.0, double tx2=1.0, double ty2=1.0);

 private:
  CQGLTexture(const CQGLTexture &);

  CQGLTexture &operator=(const CQGLTexture &);

  bool init(const QImage &image, bool flip);

 private:
  QImage         image_;
  unsigned char *imageData_ { nullptr };

  int width_  { 0 };
  int height_ { 0 };

  uint        id_       { 0 };
  std::string name_;
  bool        valid_    { false };
  WrapType    wrapType_ { WrapType::REPEAT };
  bool        useAlpha_ { true };
  bool        flipped_  { false };

  GLuint frameBufferId_ { 0 };
  GLuint depthRenderBuffer_ { 0 };

  int targetWidth_  { -1 };
  int targetHeight_ { -1 };

  QOpenGLExtraFunctions *functions_ { nullptr };
};

//---

class CQGLTextureMgr {
 public:
  static CQGLTextureMgr *getInstance() {
    static CQGLTextureMgr *instance;

    if (! instance)
      instance = new CQGLTextureMgr;

    return instance;
  }

  CQGLTexture *load(const QString &fileName, bool flip=false) {
    // Look for the texture in the registry
    auto p = texture_map_.find(fileName);

    if (p != texture_map_.end())
      return (*p).second;

    // If not found, load the texture
    auto *texture = new CQGLTexture;

    if (! texture->load(fileName, flip)) {
      delete texture;
      return nullptr;
    }

    // The texture has been successfully loaded, register it.
    texture_map_[fileName] = texture;

    return texture;
  }

 private:
  using TextureMap = std::map<QString, CQGLTexture *>;

  TextureMap texture_map_;
};

#endif
