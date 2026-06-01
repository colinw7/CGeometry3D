#include <Toolbar.h>
#include <App.h>
#include <Canvas.h>
#include <Overview.h>
#include <UI.h>

#include <CQUtil.h>
#include <CQPixmapCache.h>

#include <QComboBox>

namespace CQTclParticle3D {

Toolbar::
Toolbar(App *app) :
 app_(app)
{
  setObjectName("toolbar");

  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

  auto *layout = new QHBoxLayout(this);
  layout->setMargin(0); layout->setSpacing(4);

  //---

  UI ui(this, layout);

  //---

  canvasAnimData_.frame = ui.startFrame(/*horizontal*/true);

  canvasAnimData_.pauseButton =
    ui.addIconCheckButton("pause", "PAUSE", "Pause");

  //---

  canvasSelectData_.frame = ui.startFrame(/*horizontal*/true);

#if 0
  canvasSelectData_.editModeCombo = new QComboBox;
  canvasSelectData_.editModeCombo->addItems(QStringList() << "Select" << "Edit");

  ui.addWidget(canvasSelectData_.editModeCombo);
#endif

  canvasSelectData_.faceSelectButton =
    ui.addIconCheckButton("faceSelect" , "FACE_SELECT" , "Face Select");
  canvasSelectData_.edgeSelectButton =
    ui.addIconCheckButton("edgeSelect" , "EDGE_SELECT" , "Edge Select");
  canvasSelectData_.pointSelectButton =
    ui.addIconCheckButton("pointSelect", "POINT_SELECT", "Point Select");

  ui.endFrame();

  //---

#if 0
  overviewSelectData_.frame = ui.startFrame(/*horizontal*/true);

  overviewSelectData_.objectSelectButton =
    ui.addIconCheckButton("objectSelect", "OBJECT_SELECT", "Object Select");
  overviewSelectData_.faceSelectButton =
    ui.addIconCheckButton("faceSelect" , "FACE_SELECT" , "Face Select");
  overviewSelectData_.edgeSelectButton =
    ui.addIconCheckButton("edgeSelect" , "EDGE_SELECT" , "Edge Select");
  overviewSelectData_.pointSelectButton =
    ui.addIconCheckButton("pointSelect", "POINT_SELECT", "Point Select");

  ui.endFrame();
#endif

  //---

  menuFrame_ = ui.startFrame(/*horizontal*/true);

  menuBar_ = ui.startMenuBar();

#if 0
  ui.startMenu("View");

  ui.addAction("Perspective", SLOT(perspectiveSlot()));
  ui.addAction("Top"        , SLOT(topSlot()));
  ui.addAction("Bottom"     , SLOT(bottomSlot()));
  ui.addAction("Left"       , SLOT(leftSlot()));
  ui.addAction("Right"      , SLOT(rightSlot()));
  ui.addAction("Front"      , SLOT(frontSlot()));
  ui.addAction("Back"       , SLOT(backSlot()));

  ui.addMenuSeparator();

  ui.addCheckAction("Local", canvas()->isLocalMode(), SLOT(localSlot(bool)));

  ui.endMenu();
#endif

  ui.startMenu("Select");

//ui.addAction("All" , SLOT(selectAllSlot()));
  ui.addAction("None", SLOT(selectNoneSlot()));

  ui.endMenu();

#if 0
  ui.startMenu("Add");

  ui.addAction("Plane"   , SLOT(addPlaneSlot()));
  ui.addAction("Cube"    , SLOT(addCubeSlot()));
  ui.addAction("Circle"  , SLOT(addCircleSlot()));
  ui.addAction("Sphere"  , SLOT(addSphereSlot()));
  ui.addAction("Cylinder", SLOT(addCylinderSlot()));
  ui.addAction("Cone"    , SLOT(addConeSlot()));
  ui.addAction("Torus"   , SLOT(addTorusSlot()));

  ui.endMenu();
#endif

  ui.startMenu("Object");

  ui.addAction("Transform");

  ui.endMenu();

  ui.endMenuBar();

  ui.endFrame();

  //---

  ui.addStretch();

  depthTestButton_ = ui.addIconCheckButton("depthTest", "DEPTH3D", "Depth Test");
  cullFaceButton_  = ui.addIconCheckButton("cullFace" , "CULL3D" , "Cull Face" );
  frontFaceButton_ = ui.addIconCheckButton("frontFace", "FRONT3D", "Front Face");

  wireframeButton_   = ui.addIconButton("wireframe"  , "WIREFRAME"   , "Wireframe"   );
  solidFillButton_   = ui.addIconButton("solidFill"  , "SOLID_FILL"  , "Solid Fill"  );
  textureFillButton_ = ui.addIconButton("textureFill", "TEXTURE_FILL", "Texture Fill");

  //---

  auto *debugButton = addDebugButton();

  ui.addWidget(debugButton);

  //---

  connectSlots(true);

#if 0
  viewTypeSlot();
#endif

  updateWidgets();
}

QToolButton *
Toolbar::
addDebugButton()
{
  int is = QFontMetrics(font()).height() + 6;

  auto *button = new QToolButton;

  button->setIcon(CQPixmapCacheInst->getIcon("MENU"));
  button->setPopupMode(QToolButton::InstantPopup);

  button->setAutoRaise(true);
  button->setIconSize(QSize(is, is));

  auto *menu = new QMenu;

#if 0
  auto *action1 = menu->addAction("Meta Edit");
  auto *action2 = menu->addAction("Performance");
  auto *action3 = menu->addAction("Options");

  connect(action1, SIGNAL(triggered()), this, SLOT(metaEditSlot()));
  connect(action2, SIGNAL(triggered()), this, SLOT(performanceSlot()));
  connect(action3, SIGNAL(triggered()), this, SLOT(optionsSlot()));
#endif

  button->setMenu(menu);

  return button;
}

void
Toolbar::
connectSlots(bool b)
{
  auto *canvas = this->canvas();

#if 0
  CQUtil::connectDisconnect(b,
    app_, SIGNAL(viewTypeChanged()), this, SLOT(viewTypeSlot()));
#endif

  CQUtil::connectDisconnect(b,
    canvas, SIGNAL(selectTypeChanged()), this, SLOT(updateWidgets()));
#if 0
  CQUtil::connectDisconnect(b,
    canvas, SIGNAL(editModeChanged()), this, SLOT(updateWidgets()));
#endif
  CQUtil::connectDisconnect(b,
    canvas, SIGNAL(glStateChanged()), this, SLOT(updateWidgets()));

#if 0
  CQUtil::connectDisconnect(b,
    overview(), SIGNAL(selectTypeChanged()), this, SLOT(updateWidgets()));
#endif

#if 0
  CQUtil::connectDisconnect(b, canvasSelectData_.editModeCombo,
    SIGNAL(currentIndexChanged(int)), this, SLOT(editModeSlot(int)));
#endif

  CQUtil::connectDisconnect(b,
    canvasAnimData_.pauseButton, SIGNAL(toggled(bool)), this, SLOT(pauseSlot(bool)));

  CQUtil::connectDisconnect(b,
    canvasSelectData_.faceSelectButton, SIGNAL(toggled(bool)), this, SLOT(faceSelectSlot(bool)));
  CQUtil::connectDisconnect(b,
    canvasSelectData_.edgeSelectButton, SIGNAL(toggled(bool)), this, SLOT(edgeSelectSlot(bool)));
  CQUtil::connectDisconnect(b,
    canvasSelectData_.pointSelectButton, SIGNAL(toggled(bool)), this, SLOT(pointSelectSlot(bool)));

#if 0
  CQUtil::connectDisconnect(b,
    overviewSelectData_.objectSelectButton, SIGNAL(toggled(bool)),
    this, SLOT(objectSelectSlot(bool)));
  CQUtil::connectDisconnect(b,
    overviewSelectData_.faceSelectButton, SIGNAL(toggled(bool)),
    this, SLOT(faceSelectSlot(bool)));
  CQUtil::connectDisconnect(b,
    overviewSelectData_.edgeSelectButton, SIGNAL(toggled(bool)),
    this, SLOT(edgeSelectSlot(bool)));
  CQUtil::connectDisconnect(b,
    overviewSelectData_.pointSelectButton, SIGNAL(toggled(bool)),
    this, SLOT(pointSelectSlot(bool)));
#endif

  CQUtil::connectDisconnect(b,
    depthTestButton_, SIGNAL(clicked(bool)), this, SLOT(depthTestSlot(bool)));
  CQUtil::connectDisconnect(b,
    cullFaceButton_, SIGNAL(clicked(bool)), this, SLOT(cullFaceSlot(bool)));
  CQUtil::connectDisconnect(b,
    frontFaceButton_, SIGNAL(clicked(bool)), this, SLOT(frontFaceSlot(bool)));

  CQUtil::connectDisconnect(b,
    wireframeButton_, SIGNAL(clicked()), this, SLOT(wireframeSlot()));
  CQUtil::connectDisconnect(b,
    solidFillButton_, SIGNAL(clicked()), this, SLOT(solidFillSlot()));
  CQUtil::connectDisconnect(b,
    textureFillButton_, SIGNAL(clicked()), this, SLOT(textureFillSlot()));
}

#if 0
void
Toolbar::
viewTypeSlot()
{
  viewType_ = app_->viewType();

  canvasSelectData_  .frame->setVisible(viewType_ == ViewType::MODEL);
  overviewSelectData_.frame->setVisible(viewType_ == ViewType::OVERVIEW);

  menuFrame_->setVisible(viewType_ == ViewType::MODEL ||
                         viewType_ == ViewType::OVERVIEW);

  updateWidgets();
}
#endif

#if 0
void
Toolbar::
editModeSlot(int s)
{
  connectSlots(false);

  auto editMode = (s ? Canvas::EditMode::EDIT : Canvas::EditMode::OBJECT);

  canvas()->setEditMode(editMode);

  connectSlots(true);

  updateWidgets();
}
#endif

void
Toolbar::
pauseSlot(bool b)
{
  app_->setRunning(! b);
}

void
Toolbar::
objectSelectSlot(bool)
{
#if 0
  if (viewType_ == ViewType::OVERVIEW)
    overviewSelectData_.selectType = App::SelectType::OBJECT;
#endif

  updateSelectType();

  updateWidgets();
}

void
Toolbar::
faceSelectSlot(bool)
{
#if 0
  if      (viewType_ == ViewType::MODEL)
    canvasSelectData_.selectType = App::SelectType::FACE;
  else if (viewType_ == ViewType::OVERVIEW)
    overviewSelectData_.selectType = App::SelectType::FACE;
#else
  canvasSelectData_.selectType = App::SelectType::FACE;
#endif

  updateSelectType();

  updateWidgets();
}

void
Toolbar::
edgeSelectSlot(bool)
{
#if 0
  if      (viewType_ == ViewType::MODEL)
    canvasSelectData_.selectType = App::SelectType::EDGE;
  else if (viewType_ == ViewType::OVERVIEW)
    overviewSelectData_.selectType = App::SelectType::EDGE;
#else
  canvasSelectData_.selectType = App::SelectType::EDGE;
#endif

  updateSelectType();

  updateWidgets();
}

void
Toolbar::
pointSelectSlot(bool)
{
#if 0
  if      (viewType_ == ViewType::MODEL)
    canvasSelectData_.selectType = App::SelectType::POINT;
  else if (viewType_ == ViewType::OVERVIEW)
    overviewSelectData_.selectType = App::SelectType::POINT;
#else
  canvasSelectData_.selectType = App::SelectType::POINT;
#endif

  updateSelectType();

  updateWidgets();
}

void
Toolbar::
updateSelectType()
{
  connectSlots(false);

#if 0
  if      (viewType_ == ViewType::MODEL)
    canvas()->setSelectType(canvasSelectData_.selectType);
  else if (viewType_ == ViewType::OVERVIEW)
    overview()->setSelectType(overviewSelectData_.selectType);
#else
  canvas()->setSelectType(canvasSelectData_.selectType);
#endif

  connectSlots(true);
}

void
Toolbar::
updateWidgets()
{
  connectSlots(false);

  auto *canvas = this->canvas();

  depthTestButton_->setChecked(canvas->isDepthTest());
  cullFaceButton_ ->setChecked(canvas->isCullFace());
  frontFaceButton_->setChecked(canvas->isFrontFace());

#if 0
  if      (viewType_ == ViewType::MODEL) {
    auto editMode = canvas->editMode();

    if (editMode == Canvas::EditMode::EDIT)
      canvasSelectData_.selectType = canvas->selectType();

    canvasSelectData_.editModeCombo->
      setCurrentIndex(editMode == Canvas::EditMode::EDIT ? 1 : 0);

    canvasSelectData_.faceSelectButton ->setVisible(editMode == Canvas::EditMode::EDIT);
    canvasSelectData_.edgeSelectButton ->setVisible(editMode == Canvas::EditMode::EDIT);
    canvasSelectData_.pointSelectButton->setVisible(editMode == Canvas::EditMode::EDIT);

    canvasSelectData_.faceSelectButton ->
      setChecked(canvasSelectData_.selectType == App::SelectType::FACE);
    canvasSelectData_.edgeSelectButton ->
      setChecked(canvasSelectData_.selectType == App::SelectType::EDGE);
    canvasSelectData_.pointSelectButton->
      setChecked(canvasSelectData_.selectType == App::SelectType::POINT);
  }
  else if (viewType_ == ViewType::OVERVIEW) {
    overviewSelectData_.objectSelectButton->
      setChecked(overviewSelectData_.selectType == App::SelectType::OBJECT);
    overviewSelectData_.faceSelectButton ->
      setChecked(overviewSelectData_.selectType == App::SelectType::FACE);
    overviewSelectData_.edgeSelectButton ->
      setChecked(overviewSelectData_.selectType == App::SelectType::EDGE);
    overviewSelectData_.pointSelectButton->
      setChecked(overviewSelectData_.selectType == App::SelectType::POINT);
  }
#else
#if 0
  auto editMode = canvas->editMode();

  if (editMode == Canvas::EditMode::EDIT)
    canvasSelectData_.selectType = canvas->selectType();

  canvasSelectData_.editModeCombo->
    setCurrentIndex(editMode == Canvas::EditMode::EDIT ? 1 : 0);

  canvasSelectData_.faceSelectButton ->setVisible(editMode == Canvas::EditMode::EDIT);
  canvasSelectData_.edgeSelectButton ->setVisible(editMode == Canvas::EditMode::EDIT);
  canvasSelectData_.pointSelectButton->setVisible(editMode == Canvas::EditMode::EDIT);
#endif

  canvasSelectData_.faceSelectButton ->
    setChecked(canvasSelectData_.selectType == App::SelectType::FACE);
  canvasSelectData_.edgeSelectButton ->
    setChecked(canvasSelectData_.selectType == App::SelectType::EDGE);
  canvasSelectData_.pointSelectButton->
    setChecked(canvasSelectData_.selectType == App::SelectType::POINT);
#endif

  connectSlots(true);
}

#if 0
void
Toolbar::
perspectiveSlot()
{
  canvas()->setViewType(Canvas::ViewType::PERSPECTIVE);
}

void
Toolbar::
topSlot()
{
  canvas()->setViewType(Canvas::ViewType::TOP);
}

void
Toolbar::
bottomSlot()
{
  canvas()->setViewType(Canvas::ViewType::BOTTOM);
}

void
Toolbar::
leftSlot()
{
  canvas()->setViewType(Canvas::ViewType::LEFT);
}

void
Toolbar::
rightSlot()
{
  canvas()->setViewType(Canvas::ViewType::RIGHT);
}

void
Toolbar::
frontSlot()
{
  canvas()->setViewType(Canvas::ViewType::FRONT);
}

void
Toolbar::
backSlot()
{
  canvas()->setViewType(Canvas::ViewType::BACK);
}

void
Toolbar::
localSlot(bool)
{
  canvas()->setLocalMode(! canvas()->isLocalMode());
}
#endif

#if 0
void
Toolbar::
selectAllSlot()
{
  canvas()->selectAllObjects();
}
#endif

void
Toolbar::
selectNoneSlot()
{
  canvas()->deselectAll();
}

#if 0
void
Toolbar::
addPlaneSlot()
{
  canvas()->addPlane();
}

void
Toolbar::
addCubeSlot()
{
  if      (viewType_ == ViewType::MODEL)
    canvas()->addCube();
  else if (viewType_ == ViewType::OVERVIEW)
    overview()->addCube();
}

void
Toolbar::
addCircleSlot()
{
  if      (viewType_ == ViewType::MODEL)
    canvas()->addCircle();
  else if (viewType_ == ViewType::OVERVIEW)
    overview()->addCircle();
}

void
Toolbar::
addSphereSlot()
{
  if      (viewType_ == ViewType::MODEL)
    canvas()->addSphere();
  else if (viewType_ == ViewType::OVERVIEW)
    overview()->addSphere();
}

void
Toolbar::
addCylinderSlot()
{
  if      (viewType_ == ViewType::MODEL)
    canvas()->addCylinder();
  else if (viewType_ == ViewType::OVERVIEW)
    overview()->addCylinder();
}

void
Toolbar::
addConeSlot()
{
  if      (viewType_ == ViewType::MODEL)
    canvas()->addCone();
  else if (viewType_ == ViewType::OVERVIEW)
    overview()->addCone();
}

void
Toolbar::
addTorusSlot()
{
  if      (viewType_ == ViewType::MODEL)
    canvas()->addTorus();
  else if (viewType_ == ViewType::OVERVIEW)
    overview()->addTorus();
}
#endif

void
Toolbar::
depthTestSlot(bool b)
{
  auto *canvas = this->canvas();

  canvas->setDepthTest(b);

  canvas->update();
}

void
Toolbar::
cullFaceSlot(bool b)
{
  auto *canvas = this->canvas();

  canvas->setCullFace(b);

  canvas->update();
}

void
Toolbar::
frontFaceSlot(bool b)
{
  auto *canvas = this->canvas();

  canvas->setFrontFace(b);

  canvas->update();
}

void
Toolbar::
wireframeSlot()
{
  auto *canvas = this->canvas();

  canvas->setWireframe(true);
  canvas->setSolid    (false);
  canvas->setTextured (false);

  canvas->update();
}

void
Toolbar::
solidFillSlot()
{
  auto *canvas = this->canvas();

  canvas->setWireframe(false);
  canvas->setSolid    (true);
  canvas->setTextured (false);

  canvas->update();
}

void
Toolbar::
textureFillSlot()
{
  auto *canvas = this->canvas();

  canvas->setWireframe(false);
  canvas->setSolid    (true);
  canvas->setTextured (true);

  canvas->update();
}

#if 0
void
Toolbar::
metaEditSlot()
{
  app_->showMetaEdit();
}

void
Toolbar::
performanceSlot()
{
  app_->showPerfDialog();
}

void
Toolbar::
optionsSlot()
{
  app_->showAppOptions();
}
#endif

}
