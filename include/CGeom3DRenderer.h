#ifndef CGeom3DRenderer_H
#define CGeom3DRenderer_H

#include <CFont.h>

class CGeom3DRenderer {
 public:
  CGeom3DRenderer() { }

  virtual ~CGeom3DRenderer() { }

  virtual uint getWidth () const = 0;
  virtual uint getHeight() const = 0;

  virtual void setForeground(const CRGBA &rgba) = 0;

  virtual void drawPoint(const CIPoint2D &point) = 0;

  virtual void drawLine(const CIPoint2D &point1, const CIPoint2D &point2) = 0;

  virtual void setFont(CFontPtr font) = 0;

  virtual void drawString(const CIPoint2D &p, const std::string &str) = 0;

  virtual void fillCircle(const CIPoint2D &c, double r) = 0;
};

#endif
