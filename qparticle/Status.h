#ifndef Status_H
#define Status_H

#include <QFrame>

class QLabel;

namespace CQTclParticle3D {

class App;

class Status : public QFrame {
  Q_OBJECT

 public:
  Status(App *app);

  void setText(const QString &text);

 private:
  App*    app_   { nullptr };
  QLabel* label_ { nullptr };
};

}

#endif
