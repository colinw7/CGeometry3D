#ifndef Sidebar_H
#define Sidebar_H

#include <Canvas.h>

#include <QFrame>

class CQIconButton;

namespace CQTclParticle3D {

class App;

class Sidebar : public QFrame {
  Q_OBJECT

 public:
//using ViewType = App::ViewType;
  using EditType = App::EditType;

 public:
  Sidebar(App *app);

 private Q_SLOTS:
#if 0
  void viewTypeSlot();
#endif

  void selectSlot(bool state);
//void cursorSlot(bool state);
  void cameraSlot(bool state);
//void lightSlot (bool state);
  void tclSlot   (bool state);

#if 0
  void moveSlot  (bool state);
  void rotateSlot(bool state);
  void scaleSlot (bool state);
#endif

#if 0
  void extrudeSlot();
  void loopCutSlot();
#endif

  void updateButtonState();

 private:
  void connectSlots(bool b);
  void updateEditType();

 private:
  App* app_ { nullptr };

#if 0
  ViewType viewType_ { ViewType::NONE };
#endif

  EditType editType_ { EditType::SELECT };

  CQIconButton* selectButton_ { nullptr };
//CQIconButton* cursorButton_ { nullptr };
  CQIconButton* cameraButton_ { nullptr };
//CQIconButton* lightButton_  { nullptr };
  CQIconButton* tclButton_    { nullptr };

#if 0
  CQIconButton* moveButton_   { nullptr };
  CQIconButton* rotateButton_ { nullptr };
  CQIconButton* scaleButton_  { nullptr };
#endif

#if 0
  CQIconButton* extrudeButton_ { nullptr };
  CQIconButton* loopCutButton_ { nullptr };
#endif

  std::vector<CQIconButton *> checkButtons_;
};

}

#endif
