#ifndef Camera_H
#define Camera_H

#include <CGLCameraIFace.h>
#include <CQuaternion.h>
#include <CVector3D.h>

#include <QObject>

namespace CQTclParticle3D {

class App;

class Camera : public QObject, public CGLCameraIFace {
  Q_OBJECT

 public:
  Camera(App *app);

  //---

  bool isDisableRoll() const { return disableRoll_; }
  void setDisableRoll(bool b) { disableRoll_ = b; stateChanged(); }

  //---

  bool isClampPitch() const { return clampPitch_; }
  void setClampPitch(bool b) { clampPitch_ = b; stateChanged(); }

  double minPitch() const { return minPitch_; }
  void setMinPitch(double r) { minPitch_ = r; stateChanged(); }

  double maxPitch() const { return maxPitch_; }
  void setMaxPitch(double r) { maxPitch_ = r; stateChanged(); }

  //---

  bool isClampYaw() const { return clampYaw_; }
  void setClampYaw(bool b) { clampYaw_ = b; stateChanged(); }

  double minYaw() const { return minYaw_; }
  void setMinYaw(double r) { minYaw_ = r; stateChanged(); }

  double maxYaw() const { return maxYaw_; }
  void setMaxYaw(double r) { maxYaw_ = r; stateChanged(); }

  //---

  bool isClampRoll() const { return clampRoll_; }
  void setClampRoll(bool b) { clampRoll_ = b; stateChanged(); }

  double minRoll() const { return minRoll_; }
  void setMinRoll(double r) { minRoll_ = r; stateChanged(); }

  double maxRoll() const { return maxRoll_; }
  void setMaxRoll(double r) { maxRoll_ = r; stateChanged(); }

  //---

  // origin (for rotation center)
  const CVector3D &origin() const override;
  void setOrigin(const CVector3D &p) override;

  // position (for camera position)
  const CVector3D &position() const override;
  void setPosition(const CVector3D &p) override;

  //---

  // rotation x angle
  double pitch() const override;
  void setPitch(double r) override;

  // rotation x angle
  double yaw() const override;
  void setYaw(double r) override;

  // rotation z angle
  double roll() const override;
  void setRoll(double r) override;

  //---

  // coordinate system vectors
  CVector3D front() const override;
  CVector3D up   () const override;
  CVector3D right() const override;

  //---

  void moveFront(double d) override;
  void moveUp   (double d) override;
  void moveRight(double d) override;

  void rotateX(double da) override; // pitch
  void rotateY(double da) override; // yaw
  void rotateZ(double da) override; // roll

  CMatrix3DH perspectiveMatrix() const override;
  CMatrix3DH orthoMatrix() const override;
  CMatrix3DH viewMatrix() const override;

  double distance() const { return distance_; }
  void setDistance(double r) { if (distance_ != r) { distance_ = r; stateChanged(); } }

  void stateChanged() override { orientationValid_ = false; Q_EMIT stateChangedSignal(); }

  //---

  void printMatrices() const;

 Q_SIGNALS:
  void stateChangedSignal();

 private:
  void updateOrientation() const;
  void updateOrientationI();

  void calcViewMatrix();

  void calcPerspectiveMatrix();
  void calcOrthoMatrix();

  CVector3D toEuler(const CQuaternion &q) const;

  void lookAt(const CVector3D &forward, const CVector3D &up);

  CQuaternion calcLookAt(const CVector3D &forward, const CVector3D &up) const;

 private:
  App* app_ { nullptr };

  bool disableRoll_ { false };

  bool   clampPitch_ { false };
  double minPitch_   { -M_PI/2.0 };
  double maxPitch_   {  M_PI/2.0 };

  bool   clampYaw_ { false };
  double minYaw_   { -M_PI/2.0 };
  double maxYaw_   {  M_PI/2.0 };

  bool   clampRoll_ { false };
  double minRoll_   { -M_PI/2.0 };
  double maxRoll_   {  M_PI/2.0 };

  CQuaternion orientation_;
  bool        orientationValid_ { false };

  CVector3D front_ { 0, 0, 1 };
  CVector3D up_    { 0, 1, 0 };
  CVector3D right_ { 1, 0, 0 };

  CMatrix3DH perspectiveMatrix_ { CMatrix3DH::identity() };
  CMatrix3DH orthoMatrix_       { CMatrix3DH::identity() };
  CMatrix3DH viewMatrix_        { CMatrix3DH::identity() };

  CVector3D originDelta_ { 0, 0, 0 };
  CVector3D angleDelta_  { 0, 0, 0 };

  double distance_ { 1.0 };
};

}

#endif
