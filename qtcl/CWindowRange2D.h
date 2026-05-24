#ifndef CWindowRange2D_H
#define CWindowRange2D_H

#include <CMathRound.h>
#include <CMatrix2D.h>
#include <CBBox2D.h>

// Class to represent a 2D mapping from window to pixel coordinates
class CDisplayRange2D {
 public:
  enum class HAlign {
    NONE,
    LEFT,
    CENTER,
    RIGHT
  };

  enum class VAlign {
    NONE,
    TOP,
    CENTER,
    BOTTOM
  };

  using Point  = CPoint2D;
  using BBox   = CBBox2D;
  using Matrix = CMatrix2D;

 public:
  //! value range (TODO: always double ?)
  template<typename T>
  struct RangeT {
    T xmin, ymin, xmax, ymax;

    RangeT(T xmin1=0, T ymin1=0, T xmax1=0, T ymax1=0) :
     xmin(xmin1), ymin(ymin1), xmax(xmax1), ymax(ymax1) {
    }

    void set(T xmin1, T ymin1, T xmax1, T ymax1) {
      xmin = xmin1; ymin = ymin1; xmax = xmax1; ymax = ymax1;
    }

    void get(T *xmin1, T *ymin1, T *xmax1, T *ymax1) const {
      *xmin1 = xmin; *ymin1 = ymin; *xmax1 = xmax; *ymax1 = ymax;
    }

    T dx() const { return xmax - xmin; }
    T dy() const { return ymax - ymin; }

    T xmid() const { return (xmin + xmax)/2.0; }
    T ymid() const { return (ymin + ymax)/2.0; }

    void incX(T dx) { xmin += dx; xmax += dx; }
    void incY(T dy) { ymin += dy; ymax += dy; }
  };

//using IRange = RangeT<int>;
  using RRange = RangeT<double>;

 public:
  CDisplayRange2D(double pxmin = 0, double pymin = 0, double pxmax = 100, double pymax  = 100,
                  double wxmin = 0.0, double wymin = 0.0, double wxmax = 1.0, double wymax = 1.0) :
   pixel_(pxmin, pymin, pxmax, pymax), window_(wxmin, wymin, wxmax, wymax) {
    reset();
  }

  void setPixelRange(double pxmin, double pymin, double pxmax, double pymax) {
    pixel_.set(pxmin, pymin, pxmax, pymax);

    reset();
  }

  void setWindowRange(double wxmin, double wymin, double wxmax, double wymax) {
    window_.set(wxmin, wymin, wxmax, wymax);

    reset();
  }

  void getPixelRange(double *pxmin, double *pymin, double *pxmax, double *pymax) const {
    pixel_.get(pxmin, pymin, pxmax, pymax);
  }

  void getPixelRange(int *pxmin, int *pymin, int *pxmax, int *pymax) const {
    double pixel_xmin1, pixel_ymin1, pixel_xmax1, pixel_ymax1;
    pixel_.get(&pixel_xmin1, &pixel_ymin1, &pixel_xmax1, &pixel_ymax1);

    *pxmin = int(pixel_xmin1);
    *pymin = int(pixel_ymin1);
    *pxmax = int(pixel_xmax1);
    *pymax = int(pixel_ymax1);
  }

  double getPixelWidth () const { return (pixel_.dx() >= 0 ? pixel_.dx() + 1 : pixel_.dx() - 1); }
  double getPixelHeight() const { return (pixel_.dy() >= 0 ? pixel_.dy() + 1 : pixel_.dy() - 1); }

  void getWindowRange(double *wxmin, double *wymin, double *wxmax, double *wymax) const {
    window_.get(wxmin, wymin, wxmax, wymax);
  }

  void getWindowRange(BBox &bbox) const {
    double wxmin, wymin, wxmax, wymax;
    getWindowRange(&wxmin, &wymin, &wxmax, &wymax);

    bbox = BBox(Point(wxmin, wymin), Point(wxmax, wymax));
  }

  double getWindowWidth () const { return window_.dx(); }
  double getWindowHeight() const { return window_.dy(); }

  Point getWindowCenter() const { return Point(window_.xmid(), window_.ymid()); }

  void getTransformedWindowRange(double *wxmin, double *wymin, double *wxmax, double *wymax) const {
    window1_.get(wxmin, wymin, wxmax, wymax);
  }

  // get/set equal scale flag
  bool getEqualScale() const { return equalScale_; }
  void setEqualScale(bool flag) { equalScale_ = flag; recalc(); }

  bool getScaleMin() const { return scaleMin_; }
  void setScaleMin(bool flag) { scaleMin_ = flag; recalc(); }

  HAlign getHAlign() const { return halign_; }
  void setHAlign(HAlign halign) { halign_ = halign; recalc(); }

  VAlign getVAlign() const { return valign_; }
  void setVAlign(VAlign valign) { valign_ = valign; recalc(); }

  void setAlign(HAlign halign, VAlign valign) {
    halign_ = halign; valign_ = valign; recalc();
  }

  bool getFlipX() const { return ((pixel_.xmax - pixel_.xmin)*(window_.xmax - window_.xmin) < 0); }
  bool getFlipY() const { return ((pixel_.ymax - pixel_.ymin)*(window_.ymax - window_.ymin) < 0); }

  void zoomIn(double factor=2.0) {
    assert(factor > 0.0);

    zoomOut(1.0/factor);
  }

  void zoomOut(double factor=2.0) {
    zoomFactor_ *= factor;

    double window_hwidth  = 0.5*window_size_.x*factor;
    double window_hheight = 0.5*window_size_.y*factor;

    zoomTo(window_center_.x - window_hwidth, window_center_.y - window_hheight,
           window_center_.x + window_hwidth, window_center_.y + window_hheight);
  }

  void zoomTo(double wxmin, double wymin, double wxmax, double wymax) {
    window1_.set(wxmin, wymin, wxmax, wymax);

    recalc();
  }

  double zoomFactor() const { return zoomFactor_; }

  void scroll(double offset_x, double offset_y) {
    scrollX(offset_x);
    scrollY(offset_y);
  }

  void scrollX(double offset_x) { window1_.incX(offset_x); recalc(); }
  void scrollY(double offset_y) { window1_.incY(offset_y); recalc(); }

  void reset() { window1_ = window_; zoomFactor_ = 1.0; recalc(); }

  void recalc() {
    flipPX_ = (pixel_ .xmin > pixel_ .xmax);
    flipPY_ = (pixel_ .ymin < pixel_ .ymax); // pixel y default inverted
    flipWX_ = (window_.xmin > window_.xmax);
    flipWY_ = (window_.ymin > window_.ymax);

    //---

    window_center_.x = window1_.xmid();
    window_center_.y = window1_.ymid();

    window_size_.x = window1_.dx();
    window_size_.y = window1_.dy();

    pixel1_ = pixel_;

    //---

    calcFactors();

    factor_ = factor1_;

    //---

    if (getEqualScale()) {
      auto rsign = [](double x) { return (x >= 0.0 ? 1.0 : -1.0); };

      // update to equal scale factors
      if (getScaleMin()) {
        if (std::fabs(factor1_.x) > std::fabs(factor1_.y))
          factor1_.x = rsign(factor1_.x)*std::fabs(factor1_.y);
        else
          factor1_.y = rsign(factor1_.y)*std::fabs(factor1_.x);
      }
      else {
        if (std::fabs(factor1_.x) < std::fabs(factor1_.y))
          factor1_.x = rsign(factor1_.x)*std::fabs(factor1_.y);
        else
          factor1_.y = rsign(factor1_.y)*std::fabs(factor1_.x);
      }

      initMatrix();

      //---

      // get new pixel range
      double px1, py1, px2, py2;

      windowToPixel(window1_.xmin, window1_.ymin, &px1, &py1);
      windowToPixel(window1_.xmax, window1_.ymax, &px2, &py2);

      //---

      // use align to assign extra pixels
      extra_width_  = std::fabs(pixel_.xmax - pixel_.xmin) - std::fabs(px2 - px1);
      extra_height_ = std::fabs(pixel_.ymax - pixel_.ymin) - std::fabs(py2 - py1);

      if (pixel_.xmax > pixel_.xmin) {
        if      (halign_ == HAlign::LEFT)
          pixel1_.xmax -= extra_width_;
        else if (halign_ == HAlign::RIGHT)
          pixel1_.xmin += extra_width_;
        else if (halign_ == HAlign::CENTER) {
          pixel1_.xmin += extra_width_/2.0;
          pixel1_.xmax -= extra_width_/2.0;
        }
      }
      else {
        if      (halign_ == HAlign::LEFT)
          pixel1_.xmin -= extra_width_;
        else if (halign_ == HAlign::RIGHT)
          pixel1_.xmax += extra_width_;
        else if (halign_ == HAlign::CENTER) {
          pixel1_.xmin -= extra_width_/2.0;
          pixel1_.xmax += extra_width_/2.0;
        }
      }

      if (pixel_.ymax > pixel_.ymin) {
        if      (valign_ == VAlign::TOP)
          pixel1_.ymax -= extra_height_;
        else if (valign_ == VAlign::BOTTOM)
          pixel1_.ymin += extra_height_;
        else if (valign_ == VAlign::CENTER) {
          pixel1_.ymin += extra_height_/2.0;
          pixel1_.ymax -= extra_height_/2.0;
        }
      }
      else {
        if      (valign_ == VAlign::TOP)
          pixel1_.ymin -= extra_height_;
        else if (valign_ == VAlign::BOTTOM)
          pixel1_.ymax += extra_height_;
        else if (valign_ == VAlign::CENTER) {
          pixel1_.ymin -= extra_height_/2.0;
          pixel1_.ymax += extra_height_/2.0;
        }
      }

      calcFactors();

      initMatrix();
    }
    else {
      initMatrix();
    }
  }

  void calcFactors() {
    if (window_size_.x != 0.0)
      factor1_.x = (pxmax() - pxmin())/(wxmax() - wxmin());
    else
      factor1_.x = 1.0;

    if (window_size_.y != 0.0)
      factor1_.y = (pymax() - pymin())/(wymax() - wymin());
    else
      factor1_.y = 1.0;
  }

  void initMatrix() {
    Matrix matrix1, matrix2, matrix3;

    matrix1.setTranslation(pxmin(), pymin());
    matrix2.setScale      (factor1_.x, factor1_.y);
    matrix3.setTranslation(-wxmin(), -wymin());

    matrix_ = matrix1*matrix2*matrix3; // window to pixel

    matrix_.invert(imatrix_); // pixel to window
  }

  double pxmin() const { return std::min(pixel1_.xmin, pixel1_.xmax); }
  double pxmax() const { return std::max(pixel1_.xmin, pixel1_.xmax); }

  double pymin() const {
    if (! flipPY_)
      return std::min(pixel1_.ymin, pixel1_.ymax);
    else
      return std::max(pixel1_.ymin, pixel1_.ymax);
  }

  double pymax() const {
    if (! flipPY_)
      return std::max(pixel1_.ymin, pixel1_.ymax);
    else
      return std::min(pixel1_.ymin, pixel1_.ymax);
  }

  double wxmin() const { return std::min(window1_.xmin, window1_.xmax); }
  double wxmax() const { return std::max(window1_.xmin, window1_.xmax); }

  double wymin() const {
    if (! flipWY_)
      return std::min(window1_.ymin, window1_.ymax);
    else
      return std::max(window1_.ymin, window1_.ymax);
  }

  double wymax() const {
    if (! flipWY_)
      return std::max(window1_.ymin, window1_.ymax);
    else
      return std::min(window1_.ymin, window1_.ymax);
  }

  //---

  void windowToPixel(const Point &window, Point &pixel) const {
    pixel = matrix_*window;
  }

  void windowToPixel(double wx, double wy, double *px, double *py) const {
    Point window(wx, wy), pixel;
    windowToPixel(window, pixel);

    *px = pixel.x;
    *py = pixel.y;
  }

  void windowToPixel(double wx, double wy, int *px, int *py) const {
    Point window(wx, wy), pixel;
    windowToPixel(window, pixel);

    *px = CMathRound::Round(pixel.x);
    *py = CMathRound::Round(pixel.y);
  }

  //---

  void pixelToWindow(const Point &pixel, Point &window) const {
    window = imatrix_*Point(pixel);
  }

  void pixelToWindow(double px, double py, double *wx, double *wy) const {
    Point pixel(px, py), window;
    pixelToWindow(pixel, window);

    *wx = window.x;
    *wy = window.y;
  }

  void pixelToWindow(int px, int py, double *wx, double *wy) const {
    Point pixel(px, py), window;
    pixelToWindow(pixel, window);

    *wx = window.x;
    *wy = window.y;
  }

  void pixelLengthToWindowLength(double pixel, double *window) const {
    pixelWidthToWindowWidth(pixel, window);
  }

  //---

  void pixelWidthToWindowWidth(double pixel, double *window) const {
    if (pixel < 0) {
      pixelWidthToWindowWidth(-pixel, window);
      *window = -(*window);
      return;
    }

    double window_x1, window_y1;
    double window_x2, window_y2;

    pixelToWindow(0    , 0    , &window_x1, &window_y1);
    pixelToWindow(pixel, pixel, &window_x2, &window_y2);

    *window = fabs(window_x2 - window_x1);
  }

  void pixelHeightToWindowHeight(double pixel, double *window) const {
    if (pixel < 0) {
      pixelHeightToWindowHeight(-pixel, window);
      *window = -(*window);
      return;
    }

    double window_x1, window_y1;
    double window_x2, window_y2;

    pixelToWindow(0    , 0    , &window_x1, &window_y1);
    pixelToWindow(pixel, pixel, &window_x2, &window_y2);

    *window = fabs(window_y2 - window_y1);
  }

  void windowWidthToPixelWidth(double window, double *pixel) const {
    if (window < 0) {
      windowWidthToPixelWidth(-window, pixel);
      *pixel = -(*pixel);
      return;
    }

    double pixel_x1, pixel_y1;
    double pixel_x2, pixel_y2;

    windowToPixel(0     , 0     , &pixel_x1, &pixel_y1);
    windowToPixel(window, window, &pixel_x2, &pixel_y2);

    *pixel = fabs(pixel_x2 - pixel_x1);
  }

  void windowHeightToPixelHeight(double window, double *pixel) const {
    if (window < 0) {
      windowHeightToPixelHeight(-window, pixel);
      *pixel = -(*pixel);
      return;
    }

    double pixel_x1, pixel_y1;
    double pixel_x2, pixel_y2;

    windowToPixel(0     , 0     , &pixel_x1, &pixel_y1);
    windowToPixel(window, window, &pixel_x2, &pixel_y2);

    *pixel = fabs(pixel_y2 - pixel_y1);
  }

  //---

  bool checkPixel(double x, double y) {
    if (pixel1_.xmin < pixel1_.xmax) {
      if (x < pixel1_.xmin || x > pixel1_.xmax) return false;
    }
    else {
      if (x > pixel1_.xmin || x < pixel1_.xmax) return false;
    }

    if (pixel1_.ymin < pixel1_.ymax) {
      if (y < pixel1_.ymin || y > pixel1_.ymax) return false;
    }
    else {
      if (y > pixel1_.ymin || y < pixel1_.ymax) return false;
    }

    return true;
  }

  const Matrix &getMatrix () const { return matrix_ ; }
  const Matrix &getIMatrix() const { return imatrix_; }

 private:
  RRange pixel_;
  RRange window_;

  RRange pixel1_;
  RRange window1_;

  Point window_center_ { 0.0, 0.0 };
  Point window_size_   { 1.0, 1.0 };

  Point factor_  { 1.0, 1.0 };
  Point factor1_ { 1.0, 1.0 };

  double extra_width_  { 0.0 };
  double extra_height_ { 0.0 };

  bool equalScale_ { false };
  bool scaleMin_   { true };

  HAlign halign_ { HAlign::CENTER };
  VAlign valign_ { VAlign::CENTER };

  Matrix matrix_;
  Matrix imatrix_;

  bool flipPX_ { false };
  bool flipPY_ { false };
  bool flipWX_ { false };
  bool flipWY_ { false };

  double zoomFactor_ { 1.0 };
};

#endif
