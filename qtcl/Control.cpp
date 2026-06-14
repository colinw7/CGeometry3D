#include <Control.h>
#include <Canvas.h>
#include <Camera.h>
#include <Overview.h>
#include <App.h>
#include <UI.h>

#include <CQRealSpin.h>

#include <QCheckBox>

namespace CQTclModel3DView {

Control::
Control(App *app) :
 app_(app)
{
  setObjectName("control");

  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);

  //---

  auto *layout = new QVBoxLayout(this);
  layout->setMargin(2); layout->setSpacing(2);

  UI ui(this, layout);

  //---

  mainTab_ = ui.startTab("main");

  //------

  // Model
  ui.startTabPage("Model");

  ui.startFrame(/*horizontal*/true);

  modelData_.showWireframeCheck = ui.addCheck("Wireframe");
  modelData_.showSolidCheck     = ui.addCheck("Solid");
  modelData_.showTexturedCheck  = ui.addCheck("Textured");

  ui.addStretch();

  ui.endFrame();

  modelData_.showOrientCheck = ui.addCheck("Orientation");

  ui.addStretch();

  ui.endTabPage();

  //------

  // Camera
  ui.startTabPage("Camera");

  //---

  cameraData_.disableRollCheck = ui.addLabelEdit("Disable Roll", new QCheckBox);

  ui.startGroup("Clamp Pitch");

  cameraData_.clampPitchCheck = ui.addLabelEdit("Enabled", new QCheckBox);
  cameraData_.minPitchEdit    = ui.addLabelEdit("Min"    , new CQRealSpin);
  cameraData_.maxPitchEdit    = ui.addLabelEdit("Max"    , new CQRealSpin);

  ui.endGroup();

  ui.startGroup("Clamp Yaw");

  cameraData_.clampYawCheck = ui.addLabelEdit("Yaw", new QCheckBox);
  cameraData_.minYawEdit    = ui.addLabelEdit("Min", new CQRealSpin);
  cameraData_.maxYawEdit    = ui.addLabelEdit("Max", new CQRealSpin);

  ui.endGroup();

  ui.startGroup("Clamp Roll");

  cameraData_.clampRollCheck = ui.addLabelEdit("Roll", new QCheckBox);
  cameraData_.minRollEdit    = ui.addLabelEdit("Min" , new CQRealSpin);
  cameraData_.maxRollEdit    = ui.addLabelEdit("Max" , new CQRealSpin);

  ui.endGroup();

  ui.startGroup("Origin");

  cameraData_.xOriginEdit = ui.addLabelEdit("X", new CQRealSpin);
  cameraData_.yOriginEdit = ui.addLabelEdit("Y", new CQRealSpin);
  cameraData_.zOriginEdit = ui.addLabelEdit("Z", new CQRealSpin);

  ui.endGroup();

  ui.startGroup("Position");

  cameraData_.xPosEdit = ui.addLabelEdit("X", new CQRealSpin);
  cameraData_.yPosEdit = ui.addLabelEdit("Y", new CQRealSpin);
  cameraData_.zPosEdit = ui.addLabelEdit("Z", new CQRealSpin);

  ui.endGroup();

  cameraData_.distanceEdit = ui.addLabelEdit("Distance", new CQRealSpin);

  ui.startGroup("Angles");

  cameraData_.pitchEdit = ui.addLabelEdit("Pitch", new CQRealSpin);
  cameraData_.yawEdit   = ui.addLabelEdit("Yaw"  , new CQRealSpin);
  cameraData_.rollEdit  = ui.addLabelEdit("Roll" , new CQRealSpin);

  ui.endGroup();

  ui.startGroup("Z");

  cameraData_.nearEdit = ui.addLabelEdit("Near", new CQRealSpin);
  cameraData_.farEdit  = ui.addLabelEdit("Far" , new CQRealSpin);
  cameraData_.fovEdit  = ui.addLabelEdit("FOV" , new CQRealSpin);

  ui.endGroup();

  ui.addStretch();

  ui.endTabPage();

  //------

  // Camera
  ui.startTabPage("Light");

  lightData_.fixedDiffuseCheck = ui.addCheck("Fixed Diffuse");
  lightData_.flatShadedCheck   = ui.addCheck("Flat Shaded");

  ui.addStretch();

  ui.endTabPage();

  //------

  // Viewport
  ui.startTabPage("Viewport");

  overviewData_.showCameraCheck = ui.addCheck("Camera");
  overviewData_.showSolidCheck  = ui.addCheck("Solid");

  ui.addStretch();

  ui.endTabPage();

  //------

  ui.endTab();

  connectSlots(true);

  updateWidgets();

  connect(camera(), SIGNAL(stateChangedSignal()), this, SLOT(updateWidgets()));
}

void
Control::
connectSlots(bool b)
{
  auto connectCheckBox = [&](QCheckBox *w, const char *slotName) {
    CQUtil::connectDisconnect(b, w, SIGNAL(stateChanged(int)), this, slotName);
  };

  auto connectRealSpin = [&](CQRealSpin *w, const char *slotName) {
    CQUtil::connectDisconnect(b, w, SIGNAL(realValueChanged(double)), this, slotName);
  };

  //---

  // Model
  connectCheckBox(modelData_.showWireframeCheck, SLOT(showWireframeSlot(int)));
  connectCheckBox(modelData_.showSolidCheck    , SLOT(showSolidSlot(int)));
  connectCheckBox(modelData_.showTexturedCheck , SLOT(showTexturedSlot(int)));
  connectCheckBox(modelData_.showOrientCheck   , SLOT(showOrientSlot(int)));

  // Camera
  connectCheckBox(cameraData_.disableRollCheck, SLOT(disableRollSlot(int)));

  connectCheckBox(cameraData_.clampPitchCheck, SLOT(clampPitchSlot(int)));
  connectRealSpin(cameraData_.minPitchEdit   , SLOT(minPitchSlot(double)));
  connectRealSpin(cameraData_.maxPitchEdit   , SLOT(maxPitchSlot(double)));

  connectCheckBox(cameraData_.clampYawCheck, SLOT(clampYawSlot(int)));
  connectRealSpin(cameraData_.minYawEdit   , SLOT(minYawSlot(double)));
  connectRealSpin(cameraData_.maxYawEdit   , SLOT(maxYawSlot(double)));

  connectCheckBox(cameraData_.clampRollCheck, SLOT(clampRollSlot(int)));
  connectRealSpin(cameraData_.minRollEdit   , SLOT(minRollSlot(double)));
  connectRealSpin(cameraData_.maxRollEdit   , SLOT(maxRollSlot(double)));

  connectRealSpin(cameraData_.xOriginEdit, SLOT(xOriginSlot(double)));
  connectRealSpin(cameraData_.yOriginEdit, SLOT(yOriginSlot(double)));
  connectRealSpin(cameraData_.zOriginEdit, SLOT(zOriginSlot(double)));

  connectRealSpin(cameraData_.xPosEdit, SLOT(xPosSlot(double)));
  connectRealSpin(cameraData_.yPosEdit, SLOT(yPosSlot(double)));
  connectRealSpin(cameraData_.zPosEdit, SLOT(zPosSlot(double)));

  connectRealSpin(cameraData_.distanceEdit, SLOT(distanceSlot(double)));

  connectRealSpin(cameraData_.pitchEdit, SLOT(pitchSlot(double)));
  connectRealSpin(cameraData_.yawEdit  , SLOT(yawSlot(double)));
  connectRealSpin(cameraData_.rollEdit , SLOT(rollSlot(double)));

  connectRealSpin(cameraData_.nearEdit, SLOT(nearSlot(double)));
  connectRealSpin(cameraData_.farEdit , SLOT(farSlot(double)));

  connectRealSpin(cameraData_.fovEdit, SLOT(fovSlot(double)));

  // Light
  connectCheckBox(lightData_.fixedDiffuseCheck, SLOT(fixedDiffuseSlot(int)));
  connectCheckBox(lightData_.flatShadedCheck  , SLOT(flatShadedSlot(int)));

  // Overview
  connectCheckBox(overviewData_.showCameraCheck, SLOT(showOverviewCameraSlot(int)));
  connectCheckBox(overviewData_.showSolidCheck , SLOT(showOverviewSolidSlot(int)));
}

void
Control::
updateWidgets()
{
  auto *canvas   = app_->canvas();
  auto *camera   = this->camera();
  auto *overview = app_->overview();

  connectSlots(false);

  // Model
  modelData_.showWireframeCheck->setChecked(canvas->isWireframe());
  modelData_.showSolidCheck    ->setChecked(canvas->isSolid());
  modelData_.showTexturedCheck ->setChecked(canvas->isTextured());
  modelData_.showOrientCheck   ->setChecked(canvas->isShowOrient());

  // Camera
  cameraData_.disableRollCheck->setChecked(camera->isDisableRoll());

  cameraData_.clampPitchCheck->setChecked(camera->isClampPitch());
  cameraData_.minPitchEdit   ->setValue(camera->minPitch());
  cameraData_.maxPitchEdit   ->setValue(camera->maxPitch());

  cameraData_.clampYawCheck->setChecked(camera->isClampYaw());
  cameraData_.minYawEdit   ->setValue  (camera->minYaw());
  cameraData_.maxYawEdit   ->setValue  (camera->maxYaw());

  cameraData_.clampRollCheck->setChecked(camera->isClampRoll());
  cameraData_.minRollEdit   ->setValue  (camera->minRoll());
  cameraData_.maxRollEdit   ->setValue  (camera->maxRoll());

  cameraData_.xOriginEdit->setValue(camera->origin().x());
  cameraData_.yOriginEdit->setValue(camera->origin().y());
  cameraData_.zOriginEdit->setValue(camera->origin().z());

  cameraData_.xPosEdit->setValue(camera->position().x());
  cameraData_.yPosEdit->setValue(camera->position().y());
  cameraData_.zPosEdit->setValue(camera->position().z());

  cameraData_.distanceEdit->setValue(camera->distance());

  cameraData_.pitchEdit->setValue(camera->pitch());
  cameraData_.yawEdit  ->setValue(camera->yaw());
  cameraData_.rollEdit ->setValue(camera->roll());

  cameraData_.nearEdit->setValue(camera->near());
  cameraData_.farEdit ->setValue(camera->far());

  cameraData_.fovEdit->setValue(camera->fov());

  // Light
  lightData_.fixedDiffuseCheck->setChecked(canvas->isFixedDiffuse());
  lightData_.flatShadedCheck  ->setChecked(canvas->isFlatShaded());

  // Model
  overviewData_.showCameraCheck->setChecked(overview->isShowCamera());
  overviewData_.showSolidCheck ->setChecked(overview->isSolid());

  connectSlots(true);
}

void
Control::
showWireframeSlot(int i)
{
  auto *canvas = app_->canvas();

  canvas->setWireframe(i);

  canvas->update();
}

void
Control::
showSolidSlot(int i)
{
  auto *canvas = app_->canvas();

  canvas->setSolid(i);

  canvas->update();
}

void
Control::
showTexturedSlot(int i)
{
  auto *canvas = app_->canvas();

  canvas->setTextured(i);

  canvas->update();
}

void
Control::
showOrientSlot(int i)
{
  auto *canvas = app_->canvas();

  canvas->setShowOrient(i);

  canvas->update();
}

void
Control::
disableRollSlot(int state)
{
  UpdateScope updateScope(this);

  camera()->setDisableRoll(state);
}

void
Control::
clampPitchSlot(int state)
{
  UpdateScope updateScope(this);

  camera()->setClampPitch(state);
}

void
Control::
minPitchSlot(double r)
{
  UpdateScope updateScope(this);

  camera()->setMinPitch(r);
}

void
Control::
maxPitchSlot(double r)
{
  UpdateScope updateScope(this);

  camera()->setMaxPitch(r);
}

void
Control::
clampYawSlot(int state)
{
  UpdateScope updateScope(this);

  camera()->setClampYaw(state);
}

void
Control::
minYawSlot(double r)
{
  UpdateScope updateScope(this);

  camera()->setMinYaw(r);
}

void
Control::
maxYawSlot(double r)
{
  UpdateScope updateScope(this);

  camera()->setMaxYaw(r);
}

void
Control::
clampRollSlot(int state)
{
  UpdateScope updateScope(this);

  camera()->setClampRoll(state);
}

void
Control::
minRollSlot(double r)
{
  UpdateScope updateScope(this);

  camera()->setMinRoll(r);
}

void
Control::
maxRollSlot(double r)
{
  UpdateScope updateScope(this);

  camera()->setMaxRoll(r);
}

void
Control::
xOriginSlot(double r)
{
  UpdateScope updateScope(this);

  auto o = camera()->origin(); o.setX(r);

  camera()->setOrigin(o);
}

void
Control::
yOriginSlot(double r)
{
  UpdateScope updateScope(this);

  auto o = camera()->origin(); o.setY(r);

  camera()->setOrigin(o);
}

void
Control::
zOriginSlot(double r)
{
  UpdateScope updateScope(this);

  auto o = camera()->origin(); o.setZ(r);

  camera()->setOrigin(o);
}

void
Control::
xPosSlot(double r)
{
  UpdateScope updateScope(this);

  auto p = camera()->position(); p.setX(r);

  camera()->setPosition(p);
}

void
Control::
yPosSlot(double r)
{
  UpdateScope updateScope(this);

  auto p = camera()->position(); p.setY(r);

  camera()->setPosition(p);
}

void
Control::
zPosSlot(double r)
{
  UpdateScope updateScope(this);

  auto p = camera()->position(); p.setZ(r);

  camera()->setPosition(p);
}

void
Control::
distanceSlot(double r)
{
  UpdateScope updateScope(this);

  camera()->setDistance(r);
}

void
Control::
pitchSlot(double r)
{
  UpdateScope updateScope(this);

  camera()->setPitch(r);
}

void
Control::
yawSlot(double r)
{
  UpdateScope updateScope(this);

  camera()->setYaw(r);
}

void
Control::
rollSlot(double r)
{
  UpdateScope updateScope(this);

  camera()->setRoll(r);
}

void
Control::
nearSlot(double r)
{
  UpdateScope updateScope(this);

  camera()->setNear(r);
}

void
Control::
farSlot(double r)
{
  UpdateScope updateScope(this);

  camera()->setFar(r);
}

void
Control::
fovSlot(double r)
{
  UpdateScope updateScope(this);

  camera()->setFov(r);
}

void
Control::
fixedDiffuseSlot(int s)
{
  auto *canvas = app_->canvas();

  canvas->setFixedDiffuse(s);

  canvas->update();
}

void
Control::
flatShadedSlot(int s)
{
  auto *canvas = app_->canvas();

  canvas->setFlatShaded(s);

  canvas->update();
}

void
Control::
showOverviewCameraSlot(int s)
{
  auto *overview = app_->overview();

  overview->setShowCamera(s);

  overview->update();
}

void
Control::
showOverviewSolidSlot(int s)
{
  auto *overview = app_->overview();

  overview->setSolid(s);

  overview->update();
}

Camera *
Control::
camera() const
{
  return app_->canvas()->camera();
}

}
