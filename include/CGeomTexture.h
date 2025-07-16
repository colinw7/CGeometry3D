#ifndef CGEOM_TEXTURE_H
#define CGEOM_TEXTURE_H

#include <CFile.h>
#include <CRGBA.h>
#include <CImagePtr.h>
#include <CImage.h>
#include <CImageMgr.h>

class CGeomTextureImage {
 public:
  CGeomTextureImage(CImagePtr image) :
   image_(image) {
  }

 ~CGeomTextureImage() { }

  const std::string &name() const { return name_; }
  void setName(const std::string &s) { name_ = s; }

  const CImagePtr &image() const { return image_; }

  void getSize(int *width, int *height) {
    *width  = int(image_->getWidth ());
    *height = int(image_->getHeight());
  }

  CRGBA getRGBA(int x, int y) const {
    x = std::min(std::max(x, 0), int(image_->getWidth () - 1));
    y = std::min(std::max(y, 0), int(image_->getHeight() - 1));

    CRGBA rgba;

    image_->getRGBAPixel(x, y, rgba);

    return rgba;
  }

 private:
  std::string name_;
  CImagePtr   image_;
};

//------

class CGeomTextureMapping {
 public:
  using PointList = std::vector<CPoint2D>;

 public:
  CGeomTextureMapping() :
   points_() {
  }

  CGeomTextureMapping(const PointList &points1) :
   points_() {
    copy(points1.begin(), points1.end(), back_inserter(points_));
  }

  CGeomTextureMapping(CGeomTextureImage *texture_image, int num_vertices=4) :
   points_() {
    int width, height;

    texture_image->getSize(&width, &height);

    if      (num_vertices == 3) {
      points_.push_back(CPoint2D(         0   , height - 1));
      points_.push_back(CPoint2D( width - 1   , height - 1));
      points_.push_back(CPoint2D((width - 1)/2, 0         ));
    }
    else if (num_vertices == 4) {
      points_.push_back(CPoint2D(0        , height - 1));
      points_.push_back(CPoint2D(width - 1, height - 1));
      points_.push_back(CPoint2D(width - 1, 0         ));
      points_.push_back(CPoint2D(0        , 0         ));
    }
    else
      std::cerr << "CGeomTextureMapping: " << num_vertices <<
                   "vertices not supported" << std::endl;
  }

  CGeomTextureMapping(const CGeomTextureMapping &mapping) :
   points_(mapping.points_) {
  }

  uint numPoints() const { return uint(points_.size()); }

  const CPoint2D &getPoint(uint i) const { return points_[i]; }

 private:
  PointList points_;
};

//------

class CGeomTexture {
 public:
  CGeomTexture(CGeomTextureImage *image, CGeomTextureMapping *mapping=nullptr) :
   image_(image), mapping_(mapping), num_points_(4) {
    if (! mapping_)
      setMapping(image_, num_points_);
  }

  CGeomTexture(CImagePtr image, uint num_points=4) :
   num_points_(num_points) {
    image_ = new CGeomTextureImage(image);

    setMapping(image_, num_points);
  }

  CGeomTexture(const std::string &filename, uint num_points=4) :
   num_points_(num_points) {
    CFile file(filename);

    CImageFileSrc src(file);

    CImagePtr image = CImageMgrInst->createImage(src);

    if (image)
      image_ = new CGeomTextureImage(image);

    setMapping(image_, num_points);
  }

  CGeomTexture(const CGeomTexture &texture) :
   image_     (texture.image_),
   mapping_   (new CGeomTextureMapping(*texture.mapping_)),
   num_points_(texture.num_points_) {
  }

 ~CGeomTexture() {
    delete image_;
    delete mapping_;
  }

  CGeomTexture *dup() const {
    return new CGeomTexture(*this);
  }

  int id() const { return id_; }
  void setId(int i) { id_ = i; }

  const std::string &name() const { return image_->name(); }
  void setName(const std::string &s) { image_->setName(s); }

  CGeomTextureImage *image() const { return image_; }

  void setImage(CGeomTextureImage *image) {
    delete image_;

    image_ = image;

    setMapping(image_, num_points_);
  }

  void setMapping(CGeomTextureMapping *mapping) {
    delete mapping_;

    mapping_ = mapping;
  }

  void setMapping(CGeomTextureImage *image, uint num_points=4) {
    delete mapping_;

    mapping_    = new CGeomTextureMapping(image, int(num_points));
    num_points_ = num_points;
  }

  void setMapping(const std::vector<CPoint2D> &points) {
    delete mapping_;

    mapping_    = new CGeomTextureMapping(points);
    num_points_ = uint(points.size());
  }

  bool hasMapping() const { return mapping_; }

  void getImageSize(int *width, int *height) {
    return image()->getSize(width, height);
  }

  uint numMappingPoints() const { return mapping_->numPoints(); }

  const CPoint2D &getMappingPoint(uint i) const {
    return mapping_->getPoint(i);
  }

  CRGBA getImageRGBA(const CIPoint2D &point) {
    return image()->getRGBA(point.x, point.y);
  }

  CRGBA getImageRGBA(uint x, uint y) const {
    return image()->getRGBA(int(x), int(y));
  }

  // disable assign/copy
 private:
  CGeomTexture &operator=(const CGeomTexture &texture);

 private:
  int                  id_         { -1 };
  CGeomTextureImage   *image_      { nullptr };
  CGeomTextureMapping *mapping_    { nullptr };
  uint                 num_points_ { 0 };
};

#endif
