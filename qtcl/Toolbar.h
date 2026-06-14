#ifndef Toolbar_H
#define Toolbar_H

#include <App.h>

#include <QFrame>

class CQIconButton;

class QMenuBar;
class QComboBox;
class QToolButton;

namespace CQTclModel3DView {

class App;
class Canvas;
class Overview;

class Toolbar : public QFrame {
  Q_OBJECT

 public:
  Toolbar(App *app);

  Canvas   *canvas  () const { return app_->canvas(); }
  Overview *overview() const { return app_->overview(); }

 private Q_SLOTS:
#if 0
  void viewTypeSlot();
#endif

#if 0
  void editModeSlot(int);
#endif

  void pauseSlot(bool);

  void objectSelectSlot(bool);
  void faceSelectSlot(bool);
  void edgeSelectSlot(bool);
  void pointSelectSlot(bool);

#if 0
  void perspectiveSlot();
  void topSlot();
  void bottomSlot();
  void leftSlot();
  void rightSlot();
  void frontSlot();
  void backSlot();

  void localSlot(bool);
#endif

#if 0
  void selectAllSlot();
#endif
  void selectNoneSlot();

#if 0
  void addPlaneSlot();
  void addCubeSlot();
  void addCircleSlot();
  void addSphereSlot();
  void addCylinderSlot();
  void addConeSlot();
  void addTorusSlot();
#endif

  void depthTestSlot(bool b);
  void cullFaceSlot(bool b);
  void frontFaceSlot(bool b);

  void wireframeSlot();
  void solidFillSlot();
  void textureFillSlot();

  void updateWidgets();

#if 0
  void metaEditSlot();
  void performanceSlot();
  void optionsSlot();
#endif

 private:
  QToolButton *addDebugButton();

  void connectSlots(bool b);
  void updateSelectType();

 private:
  App* app_ { nullptr };

#if 0
  ViewType viewType_ { ViewType::NONE };
#endif

  struct CanvasAnimData {
    QFrame *frame { nullptr };

    CQIconButton* pauseButton { nullptr };
  };

  CanvasAnimData canvasAnimData_;

  struct CanvasSelectData {
    App::SelectType selectType { App::SelectType::FACE };

    QFrame *frame { nullptr };

#if 0
    QComboBox* editModeCombo { nullptr };
#endif

    CQIconButton* pointSelectButton { nullptr };
    CQIconButton* edgeSelectButton  { nullptr };
    CQIconButton* faceSelectButton  { nullptr };
  };

  CanvasSelectData canvasSelectData_;

#if 0
  struct OverviewSelectData {
    App::SelectType selectType { App::SelectType::OBJECT };

    QFrame *frame { nullptr };

    CQIconButton* pointSelectButton  { nullptr };
    CQIconButton* edgeSelectButton   { nullptr };
    CQIconButton* faceSelectButton   { nullptr };
    CQIconButton* objectSelectButton { nullptr };
  };

  OverviewSelectData overviewSelectData_;
#endif

  QFrame* menuFrame_ { nullptr };

  QMenuBar* menuBar_ { nullptr };

  CQIconButton* depthTestButton_ { nullptr };
  CQIconButton* cullFaceButton_  { nullptr };
  CQIconButton* frontFaceButton_ { nullptr };

  CQIconButton* wireframeButton_   { nullptr };
  CQIconButton* solidFillButton_   { nullptr };
  CQIconButton* textureFillButton_ { nullptr };
};

}

#endif
