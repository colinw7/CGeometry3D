#ifndef UI_H
#define UI_H

#include <CQIconButton.h>
#include <CQUtil.h>

#include <QGroupBox>
#include <QCheckBox>
#include <QPushButton>
#include <QLabel>
#include <QMenuBar>
#include <QHBoxLayout>

#include <iostream>

namespace CQTclParticle3D {

class UI {
 public:
  UI(QWidget *parent, QBoxLayout *layout) :
   parent_(parent), currentLayout_(layout) {
  }

  //---

  void startLayout(QBoxLayout *layout) {
    layoutStack_.push_back(currentLayout_);

    currentLayout_ = layout;
  }

  void endLayout() {
    currentLayout_ = layoutStack_.back();

    layoutStack_.pop_back();
  }

  //---

  template<typename WIDGET>
  WIDGET *addLabelEdit(const QString &labelStr, WIDGET *w) {
    auto labelName = nameString(labelStr);

    auto *frame = new QFrame;
    frame->setObjectName(QString("frame_%1").arg(labelName));

    auto *layout1 = new QHBoxLayout(frame);
    layout1->setMargin(2); layout1->setSpacing(2);

    auto *label = new QLabel(labelStr);
    label->setObjectName("label");

    layout1->addWidget(label);
    layout1->addWidget(w);

    currentLayout_->addWidget(frame);

    return w;
  }

  template<typename WIDGET1, typename WIDGET2>
  std::pair<WIDGET1 *, WIDGET2 *>
  addEdit(WIDGET1 *w1, WIDGET2 *w2) {
    auto *frame = new QFrame;
    frame->setObjectName("frame");

    auto *layout1 = new QHBoxLayout(frame);
    layout1->setMargin(2); layout1->setSpacing(2);

    layout1->addWidget(w1);
    layout1->addWidget(w2);

    currentLayout_->addWidget(frame);

    return std::pair<WIDGET1 *, WIDGET2 *>(w1, w2);
  }

  //---

  QCheckBox *addCheck(const QString &label) {
    auto *check = new QCheckBox(label);

    currentLayout_->addWidget(check);

    return check;
  }

  QPushButton *addButton(const QString &name, const char *slotName) {
    auto *button = new QPushButton(name);

    QObject::connect(button, SIGNAL(clicked()), parent_, slotName);

    currentLayout_->addWidget(button);

    return button;
  }

  void addStretch() {
    currentLayout_->addStretch(1);
  }

  //---

  void startGroup(const QString &name, bool horizontal=false) {
    groupStack_.push_back(currentGroup_);

    currentGroup_ = new QGroupBox(name);

    currentGroup_->setObjectName(name);

    QBoxLayout *layout = nullptr;

    if (horizontal)
      layout = new QHBoxLayout(currentGroup_);
    else
      layout = new QVBoxLayout(currentGroup_);

    layout->setMargin(2); layout->setSpacing(2);

    currentLayout_->addWidget(currentGroup_);

    startLayout(layout);
  }

  void endGroup() {
    endLayout();

    currentGroup_ = groupStack_.back();

    groupStack_.pop_back();
  }

  //---

  QFrame *startFrame(bool horizontal=false) {
    frameStack_.push_back(currentFrame_);

    currentFrame_ = new QFrame;

    currentFrame_->setObjectName("frame");

    QBoxLayout *layout = nullptr;

    if (horizontal)
      layout = new QHBoxLayout(currentFrame_);
    else
      layout = new QVBoxLayout(currentFrame_);

    layout->setMargin(2); layout->setSpacing(2);

    currentLayout_->addWidget(currentFrame_);

    startLayout(layout);

    return currentFrame_;
  }

  void endFrame() {
    endLayout();

    currentFrame_ = frameStack_.back();

    frameStack_.pop_back();
  }

  //---

  QTabWidget *startTab(const QString &name) {
    tabStack_.push_back(currentTab_);

    currentTab_ = CQUtil::makeWidget<QTabWidget>(name + "_tab");

    currentTab_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    currentLayout_->addWidget(currentTab_);

    return currentTab_;
  }

  void endTab() {
    currentTab_ = tabStack_.back();

    tabStack_.pop_back();
  }

  //---

  void startTabPage(const QString &name) {
    pageStack_.push_back(currentPage_);

    currentPage_ = CQUtil::makeWidget<QFrame>(name + "_page");

    auto *layout = new QVBoxLayout(currentPage_);

    layout->setMargin(2); layout->setSpacing(2);

    currentTab_->addTab(currentPage_, name);

    startLayout(layout);
  }

  void endTabPage() {
    endLayout();

    currentPage_ = pageStack_.back();

    pageStack_.pop_back();
  }

  //---

  CQIconButton *addIconButton(const QString &name, const QString &iconName, const QString &tip) {
    auto *button = new CQIconButton;

    button->setObjectName(name);
    button->setIcon(iconName);
    button->setIconSize(QSize(32, 32));
    button->setAutoRaise(true);
    button->setToolTip(tip);

    currentLayout_->addWidget(button);

    return button;
  }

  CQIconButton *addIconCheckButton(const QString &name, const QString &iconName,
                                   const QString &tip) {
    auto *button = addIconButton(name, iconName, tip);

    button->setCheckable(true);

    return button;
  }

  //---

  void addWidget(QWidget *w) { currentLayout_->addWidget(w); }

  //---

  QMenuBar *startMenuBar() {
    menuBar_ = new QMenuBar;

    menuBar_->setObjectName("menuBar");

    currentLayout_->addWidget(menuBar_);

    return menuBar_;
  }

  void endMenuBar() {
    menuBar_ = nullptr;
  }

  QMenu *startMenu(const QString &name) {
    menuStack_.push_back(currentMenu_);

    if (! currentMenu_) {
      if (menuBar_)
        currentMenu_ = menuBar_->addMenu(name);
    }
    else
      currentMenu_ = currentMenu_->addMenu(name);

    return currentMenu_;
  }

  void endMenu() {
    currentMenu_ = menuStack_.back();

    menuStack_.pop_back();
  }

  QAction *addAction(const QString &name, const char *slotName=nullptr) {
    auto *action = currentMenu_->addAction(name);

    if (slotName)
      QObject::connect(action, SIGNAL(triggered()), parent_, slotName);

    return action;
  }

  QAction *addCheckAction(const QString &name, bool checked=false, const char *slotName=nullptr) {
    auto *action = currentMenu_->addAction(name);

    action->setCheckable(true);
    action->setChecked(checked);

    if (slotName)
      QObject::connect(action, SIGNAL(triggered(bool)), parent_, slotName);

    return action;
  }

  void addMenuSeparator() {
    currentMenu_->addSeparator();
  }

  //---

  void validate() {
    if (! layoutStack_.empty()) std::cerr << "Bad layout stack\n";
    if (! groupStack_ .empty()) std::cerr << "Bad group stack\n";
    if (! frameStack_ .empty()) std::cerr << "Bad frame stack\n";
    if (! tabStack_   .empty()) std::cerr << "Bad tab stack\n";
    if (! pageStack_  .empty()) std::cerr << "Bad page stack\n";
    if (! menuStack_  .empty()) std::cerr << "Bad menu stack\n";
    if (menuBar_              ) std::cerr << "Bad menu bar\n";
  }

 private:
  QString nameString(const QString &str) const {
    auto str1 = str;
    str1.replace(" ", "_");
    return str1;
  }

 private:
  QWidget *parent_ { nullptr };

  QBoxLayout*               currentLayout_ { nullptr };
  std::vector<QBoxLayout *> layoutStack_;

  QGroupBox*               currentGroup_ { nullptr };
  std::vector<QGroupBox *> groupStack_;

  QFrame*               currentFrame_ { nullptr };
  std::vector<QFrame *> frameStack_;


  QTabWidget*               currentTab_ { nullptr };
  std::vector<QTabWidget *> tabStack_;

  QFrame*               currentPage_ { nullptr };
  std::vector<QFrame *> pageStack_;

  QMenuBar*            menuBar_     { nullptr };
  QMenu*               currentMenu_ { nullptr };
  std::vector<QMenu *> menuStack_;
};

}

#endif
