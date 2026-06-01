#include <Sidebar.h>
#include <App.h>
#include <Overview.h>
#include <UI.h>

#include <CQIconButton.h>
#include <CQUtil.h>

#include <QComboBox>
#include <QHBoxLayout>

namespace CQTclParticle3D {

Sidebar::
Sidebar(App *app) :
 app_(app)
{
  setObjectName("toolbar");

  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);

  auto *layout = new QVBoxLayout(this);
  layout->setMargin(0); layout->setSpacing(4);

  UI ui(this, layout);

  auto addIconCheckButton = [&](const QString &name, const QString &iconName, const QString &tip) {
    auto *button = ui.addIconCheckButton(name, iconName, tip);

    checkButtons_.push_back(button);

    return button;
  };

  // edit mode buttons
  selectButton_ = addIconCheckButton("select", "SELECT", "Select");
//cursorButton_ = addIconCheckButton("cursor", "CURSOR", "Cursor");
  cameraButton_ = addIconCheckButton("camera", "CAMERA", "Camera");
//lightButton_  = addIconCheckButton("light" , "LIGHT" , "Light" );

#if 0
  moveButton_   = addIconCheckButton("move"  , "MOVE"  , "Move"  );
  rotateButton_ = addIconCheckButton("rotate", "ROTATE", "Rotate");
  scaleButton_  = addIconCheckButton("scale" , "SCALE" , "Scale" );
#endif

  // function buttons
#if 0
  extrudeButton_ = ui.addIconButton("extrude", "EXTRUDE" , "Extrude" );
  loopCutButton_ = ui.addIconButton("loopcut", "LOOP_CUT", "Loop Cut");
#endif

  selectButton_->setChecked(true);

  ui.addStretch();

  connectSlots(true);
}

void
Sidebar::
connectSlots(bool b)
{
#if 0
  CQUtil::connectDisconnect(b,
    app_, SIGNAL(viewTypeChanged()), this, SLOT(viewTypeSlot()));
#endif

  CQUtil::connectDisconnect(b,
    app_->canvas(), SIGNAL(editTypeChanged()), this, SLOT(updateButtonState()));

  CQUtil::connectDisconnect(b,
    selectButton_, SIGNAL(toggled(bool)), this, SLOT(selectSlot(bool)));
#if 0
  CQUtil::connectDisconnect(b,
    cursorButton_, SIGNAL(toggled(bool)), this, SLOT(cursorSlot(bool)));
#endif
  CQUtil::connectDisconnect(b,
    cameraButton_, SIGNAL(toggled(bool)), this, SLOT(cameraSlot(bool)));
#if 0
  CQUtil::connectDisconnect(b,
    lightButton_, SIGNAL(toggled(bool)), this, SLOT(lightSlot(bool)));
#endif

#if 0
  CQUtil::connectDisconnect(b,
    moveButton_, SIGNAL(toggled(bool)), this, SLOT(moveSlot(bool)));
  CQUtil::connectDisconnect(b,
    rotateButton_, SIGNAL(toggled(bool)), this, SLOT(rotateSlot(bool)));
  CQUtil::connectDisconnect(b,
    scaleButton_, SIGNAL(toggled(bool)), this, SLOT(scaleSlot(bool)));
#endif

#if 0
  CQUtil::connectDisconnect(b,
    extrudeButton_, SIGNAL(clicked()), this, SLOT(extrudeSlot()));
  CQUtil::connectDisconnect(b,
    loopCutButton_, SIGNAL(clicked()), this, SLOT(loopCutSlot()));
#endif
}

#if 0
void
Sidebar::
viewTypeSlot()
{
  viewType_ = app_->viewType();

  cursorButton_->setVisible(viewType_ == ViewType::MODEL ||
                            viewType_ == ViewType::OVERVIEW);

  cameraButton_->setVisible(viewType_ == ViewType::MODEL ||
                            viewType_ == ViewType::OVERVIEW);
  lightButton_ ->setVisible(viewType_ == ViewType::MODEL ||
                            viewType_ == ViewType::OVERVIEW);

  moveButton_  ->setVisible(viewType_ == ViewType::MODEL ||
                            viewType_ == ViewType::OVERVIEW);
  rotateButton_->setVisible(viewType_ == ViewType::MODEL ||
                            viewType_ == ViewType::OVERVIEW);
  scaleButton_ ->setVisible(viewType_ == ViewType::MODEL ||
                            viewType_ == ViewType::OVERVIEW);

  extrudeButton_->setVisible(viewType_ == ViewType::MODEL ||
                            viewType_ == ViewType::OVERVIEW);
  loopCutButton_->setVisible(viewType_ == ViewType::MODEL ||
                            viewType_ == ViewType::OVERVIEW);

  updateEditType();
}
#endif

void
Sidebar::
selectSlot(bool)
{
  editType_ = EditType::SELECT;

  updateEditType();

  updateButtonState();
}

#if 0
void
Sidebar::
cursorSlot(bool state)
{
  editType_ = (state ? EditType::CURSOR : EditType::SELECT);

  updateEditType();

  updateButtonState();
}
#endif

void
Sidebar::
cameraSlot(bool state)
{
  editType_ = (state ? EditType::CAMERA : EditType::SELECT);

  updateEditType();

  updateButtonState();
}

#if 0
void
Sidebar::
lightSlot(bool state)
{
  editType_ = (state ? EditType::LIGHT : EditType::SELECT);

  updateEditType();

  updateButtonState();
}
#endif

#if 0
void
Sidebar::
moveSlot(bool state)
{
  editType_ = (state ? EditType::MOVE : EditType::SELECT);

  updateEditType();

  updateButtonState();
}

void
Sidebar::
rotateSlot(bool state)
{
  editType_ = (state ? EditType::ROTATE : EditType::SELECT);

  updateEditType();

  updateButtonState();
}

void
Sidebar::
scaleSlot(bool state)
{
  editType_ = (state ? EditType::SCALE : EditType::SELECT);

  updateEditType();

  updateButtonState();
}
#endif

void
Sidebar::
updateButtonState()
{
  connectSlots(false);

#if 0
  if      (viewType_ == ViewType::MODEL)
    editType_ = app_->canvas()->editType();
  else if (viewType_ == ViewType::OVERVIEW)
    editType_ = app_->overview()->editType();
#else
  editType_ = app_->canvas()->editType();
#endif

  selectButton_->setChecked(editType_ == EditType::SELECT);
//cursorButton_->setChecked(editType_ == EditType::CURSOR);
  cameraButton_->setChecked(editType_ == EditType::CAMERA);
//lightButton_ ->setChecked(editType_ == EditType::LIGHT);

#if 0
  moveButton_  ->setChecked(editType_ == EditType::MOVE);
  rotateButton_->setChecked(editType_ == EditType::ROTATE);
  scaleButton_ ->setChecked(editType_ == EditType::SCALE);
#endif

  connectSlots(true);
}

void
Sidebar::
updateEditType()
{
  connectSlots(false);

#if 0
  if      (viewType_ == ViewType::MODEL)
    app_->canvas()->setEditType(editType_);
  else if (viewType_ == ViewType::OVERVIEW)
    app_->overview()->setEditType(editType_);
#else
  app_->canvas()->setEditType(editType_);
#endif

  connectSlots(true);
}

#if 0
void
Sidebar::
extrudeSlot()
{
  // extrude
  app_->canvas()->extrude();

  // move mode
  app_->canvas()->extrudeMode();
}

void
Sidebar::
loopCutSlot()
{
  // loop cut
  app_->canvas()->loopCut();

  // TODO: mode
}
#endif

}
