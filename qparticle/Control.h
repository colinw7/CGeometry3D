#ifndef Control_H
#define Control_H

#include <QFrame>

class CQRealSpin;

class QTabWidget;
class QCheckBox;

namespace CQTclParticle3D {

class App;
class Camera;

class Control : public QFrame {
  Q_OBJECT

 public:
  Control(App *app);

 private:
  struct UpdateScope {
    UpdateScope(Control *control) : control_(control) { control_->connectSlots(false); }

   ~UpdateScope() { control_->connectSlots(true); control_->updateWidgets(); }

    Control* control_ { nullptr };
  };

  friend class UpdateScope;

  void connectSlots(bool);

  Camera *camera() const;

 private Q_SLOTS:
  void updateWidgets();

  // Model
  void showWireframeSlot(int);
  void showSolidSlot(int);
  void showTexturedSlot(int);

  // Camera
  void disableRollSlot(int);

  void clampPitchSlot(int);
  void minPitchSlot(double);
  void maxPitchSlot(double);

  void clampYawSlot(int);
  void minYawSlot(double);
  void maxYawSlot(double);

  void clampRollSlot(int);
  void minRollSlot(double);
  void maxRollSlot(double);

  void xOriginSlot(double);
  void yOriginSlot(double);
  void zOriginSlot(double);

  void xPosSlot(double);
  void yPosSlot(double);
  void zPosSlot(double);

  void distanceSlot(double);

  void pitchSlot(double);
  void yawSlot(double);
  void rollSlot(double);

  void nearSlot(double);
  void farSlot(double);
  void fovSlot(double);

  void fixedDiffuseSlot(int);
  void flatShadedSlot(int);

  void showOverviewCameraSlot(int);
  void showOverviewSolidSlot(int);

 private:
  App* app_ { nullptr };

  QTabWidget *mainTab_ { nullptr };

  // Model
  struct ModelData {
    QCheckBox* showWireframeCheck { nullptr };
    QCheckBox* showSolidCheck     { nullptr };
    QCheckBox* showTexturedCheck  { nullptr };
  };

  ModelData modelData_;

  // Camera
  struct CameraData {
    QCheckBox* disableRollCheck { nullptr };

    QCheckBox*  clampPitchCheck { nullptr };
    CQRealSpin* minPitchEdit    { nullptr };
    CQRealSpin* maxPitchEdit    { nullptr };

    QCheckBox*  clampYawCheck { nullptr };
    CQRealSpin* minYawEdit    { nullptr };
    CQRealSpin* maxYawEdit    { nullptr };

    QCheckBox*  clampRollCheck { nullptr };
    CQRealSpin* minRollEdit    { nullptr };
    CQRealSpin* maxRollEdit    { nullptr };

    CQRealSpin* xOriginEdit { nullptr };
    CQRealSpin* yOriginEdit { nullptr };
    CQRealSpin* zOriginEdit { nullptr };

    CQRealSpin* xPosEdit { nullptr };
    CQRealSpin* yPosEdit { nullptr };
    CQRealSpin* zPosEdit { nullptr };

    CQRealSpin* distanceEdit { nullptr };

    CQRealSpin* pitchEdit { nullptr };
    CQRealSpin* yawEdit   { nullptr };
    CQRealSpin* rollEdit  { nullptr };

    CQRealSpin* nearEdit { nullptr };
    CQRealSpin* farEdit  { nullptr };

    CQRealSpin* fovEdit { nullptr };
  };

  CameraData cameraData_;

  // Light
  struct LightData {
    QCheckBox* fixedDiffuseCheck { nullptr };
    QCheckBox* flatShadedCheck   { nullptr };
  };

  LightData lightData_;

  // Overview
  struct OverviewData {
    QCheckBox* showCameraCheck { nullptr };
    QCheckBox* showSolidCheck  { nullptr };
  };

  OverviewData overviewData_;
};

}

#endif
