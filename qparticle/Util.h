#ifndef Util_H
#define Util_H

#include <CRGBA.h>
#include <CGLVector3D.h>

namespace {

inline CGLVector3D ColorToVector(const CRGBA &c) {
  return CGLVector3D(c.getRed(), c.getGreen(), c.getBlue());
}

inline QColor RGBAToQColor(const CRGBA &c) {
  return QColor(c.getRed()*255, c.getGreen()*255, c.getBlue()*255, c.getAlpha()*255);
}

inline QVector3D vectorToQVector(const CVector3D &v) {
  return QVector3D(v.getX(), v.getY(), v.getZ());
}

}

#endif
