#include <Status.h>

#include <QLabel>
#include <QHBoxLayout>

namespace CQTclModel3DView {

Status::
Status(App *app) :
 app_(app)
{
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

  auto *layout = new QHBoxLayout(this);
  layout->setMargin(0); layout->setSpacing(2);

  label_ = new QLabel(" ");

  layout->addWidget(label_);
}

void
Status::
setText(const QString &str)
{
  label_->setText(str);
}

}
