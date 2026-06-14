#include <Overview.h>
#include <Canvas.h>
#include <Camera.h>
#include <GeomObject.h>
#include <App.h>
#include <Util.h>

#include <CQGLBuffer.h>

#include <CGeomScene3D.h>
#include <CGeomObject3D.h>

#include <QPainter>
#include <QMouseEvent>

namespace CQTclModel3DView {

Overview::
Overview(App *app) :
 app_(app)
{
  setObjectName("overview");

  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  setFocusPolicy(Qt::StrongFocus);

  setMouseTracking(true);

  //---

  xview_.ind = 0; xview_.range = new CDisplayRange2D; xview_.type = ViewType::XY;
  yview_.ind = 0; yview_.range = new CDisplayRange2D; yview_.type = ViewType::ZY;
  zview_.ind = 0; zview_.range = new CDisplayRange2D; zview_.type = ViewType::XZ;
  pview_.ind = 0; pview_.range = new CDisplayRange2D; pview_.type = ViewType::THREED;

  views_.push_back(&xview_);
  views_.push_back(&yview_);
  views_.push_back(&zview_);
  views_.push_back(&pview_);

  views2d_.push_back(&xview_);
  views2d_.push_back(&yview_);
  views2d_.push_back(&zview_);

  //---

  auto *camera = app_->canvas()->camera();

  connect(dynamic_cast<Camera *>(camera), SIGNAL(stateChangedSignal()), this, SLOT(invalidate()));
}

Overview::
~Overview()
{
}

void
Overview::
invalidate()
{
  facesValid_ = false;

  update();
}

void
Overview::
resizeEvent(QResizeEvent *)
{
  updateRange();
}

void
Overview::
updateRange()
{
  auto zoomFactor = xview_.range->zoomFactor();

  w_ = width ();
  h_ = height();

  auto w2 = w_/2.0;
  auto h2 = h_/2.0;

  xview_.range->setPixelRange( 0,  0,     w2, h2    ); // XY
  yview_.range->setPixelRange(w2,  0, w_ - 1, h2    ); // ZY
  zview_.range->setPixelRange( 0, h2,     w2, h_ - 1); // XZ
  pview_.range->setPixelRange(w2, h2, w_ - 1, h_ - 1); // 3D

  for (auto *view : views2d_)
    view->range->setEqualScale(true);

  for (auto *v : views_)
    v->range->zoomOut(zoomFactor);
}

void
Overview::
paintEvent(QPaintEvent *)
{
  QPainter painter(this);

  drawData_.painter = &painter;

  //---

  auto *canvas = app_->canvas();
  auto *camera = dynamic_cast<Camera *>(canvas->camera());

  if (canvas->isPerspective())
    drawData_.worldMatrix = camera->perspectiveMatrix();
  else
    drawData_.worldMatrix = camera->orthoMatrix();

  drawData_.viewMatrix = camera->viewMatrix();

  //---

  if (! facesValid_) {
    updateModel();

    facesValid_ = true;
  }

  updateBBox();

  //---

  // draw background
  painter.fillRect(rect(), QColor(220, 220, 220));

  //---

  // draw view borders (sets view rect)

  auto drawPixelBorder = [&](ViewData &viewData) {
    painter.setPen(Qt::black);

    QBrush brush(QColor(255, 255, 255));
    painter.setBrush(brush);

    double pxmin, pymin, pxmax, pymax;
    viewData.range->getPixelRange(&pxmin, &pymin, &pxmax, &pymax);

    viewData.rect = QRectF(pxmin, pymin, pxmax - pxmin - 1, pymax - pymin - 1);
    painter.drawRect(viewData.rect);

    painter.setBrush(Qt::NoBrush);
  };

  for (auto *view : views_)
    drawPixelBorder(*view);

  //---

  painter.save();

  drawModel();

  painter.restore();

  //---

  if (isShowCamera()) {
    painter.save();

    drawCamera(camera);

    painter.restore();
  }
}

void
Overview::
updateModel()
{
  drawData_.bbox = CBBox3D();

  drawData_.faces.clear();

  auto *scene = app_->scene();

  for (auto *object : scene->getObjects()) {
    auto *object1 = dynamic_cast<GeomObject *>(object);
    assert(object1);

    //---

    const auto &faceDatas = object1->faceDatas();
    auto       *buffer    = object1->buffer();

    for (const auto &faceData : faceDatas) {
      Face face;

      face.color = faceData.diffuse;

      for (int i = 0; i < faceData.len; ++i) {
        CQGLBuffer::PointData data;
        buffer->getPointData(faceData.pos + i, data);

        face.points.push_back(data.point->point());

        drawData_.bbox += data.point->point();
      }

      drawData_.faces.push_back(face);
    }
  }
}

void
Overview::
updateBBox()
{
  if (bboxSet_)
    return;

  bboxSet_ = true;

  bbox_ = drawData_.bbox;

  bbox_ += CPoint3D(-3, -3, -3);
  bbox_ += CPoint3D( 3,  3,  3);

  xview_.range->setWindowRange(bbox_.getXMin(), bbox_.getYMin(), bbox_.getXMax(), bbox_.getYMax());
  yview_.range->setWindowRange(bbox_.getZMax(), bbox_.getYMin(), bbox_.getZMin(), bbox_.getYMax());
  zview_.range->setWindowRange(bbox_.getXMin(), bbox_.getZMin(), bbox_.getXMax(), bbox_.getZMax());
  pview_.range->setWindowRange(-1, -1, 1, 1); // 3D

  std::cerr << "Overview BBox: " << bbox_ << "\n";
}

void
Overview::
drawModel()
{
  auto drawPolygon2D = [&](const ViewData &view, const std::vector<CPoint2D> &points) {
    drawData_.painter->setClipRect(view.rect);

    std::vector<QPointF> ppoints;

    for (const auto &p : points) {
      double px, py;
      view.range->windowToPixel(p.x, p.y, &px, &py);

      ppoints.push_back(QPointF(px, py));
    }

    drawData_.painter->drawPolygon(&ppoints[0], ppoints.size());
  };

  auto drawPolygon = [&](const std::vector<CPoint3D> &points) {
    std::vector<CPoint2D> xpoints, ypoints, zpoints, ppoints;

    for (const auto &p : points) {
      xpoints.push_back(CPoint2D(p.getX(), p.getY())); // XY
      ypoints.push_back(CPoint2D(p.getZ(), p.getY())); // ZY
      zpoints.push_back(CPoint2D(p.getX(), p.getZ())); // XZ

      auto p1 = drawData_.worldMatrix*drawData_.viewMatrix*p;

      ppoints.push_back(CPoint2D(p1.getX(), p1.getY()));
    }

    drawPolygon2D(xview_, xpoints);
    drawPolygon2D(yview_, ypoints);
    drawPolygon2D(zview_, zpoints);
    drawPolygon2D(pview_, ppoints);
  };

  for (auto &face : drawData_.faces) {
    if (isSolid()) {
      drawData_.painter->setPen(Qt::NoPen);

      drawData_.painter->setBrush(RGBAToQColor(face.color));

      drawPolygon(face.points);
    }

    if (isWireframe()) {
      drawData_.painter->setPen(Qt::black);

      drawData_.painter->setBrush(Qt::NoBrush);

      drawPolygon(face.points);
    }
  }
}

void
Overview::
drawCamera(Camera *camera)
{
  CameraShape shape;
  getCameraShape(camera, shape);

  auto pos    = camera->position();
  auto origin = camera->origin  ();

  //---

  drawData_.painter->setPen(Qt::black);
  drawData_.painter->setBrush(Qt::NoBrush);

  //---

  // draw camera position, origin and direction vectors
  auto drawCameraVector = [&](const CVector3D &v, const QColor &c, const QString &text) {
    drawData_.painter->setPen(c);
    drawVector(pos, v, text);
  };

  auto front = camera->front();
  auto up    = camera->up   ();
  auto right = camera->right();

  drawPoint(pos   , "P");
  drawPoint(origin, "O");

  drawCameraVector(front, QColor(255, 0, 0, 255), "F");
  drawCameraVector(up,    QColor(0, 255, 0, 255), "U");
  drawCameraVector(right, QColor(0, 0, 255, 255), "R");

  //---

  // draw view frustrum
  drawData_.painter->setPen(Qt::black);

  drawLine(shape.p11, shape.p12, "");
  drawLine(shape.p12, shape.p22, "");
  drawLine(shape.p22, shape.p21, "");
  drawLine(shape.p21, shape.p11, "");

  drawLine(pos.point(), shape.p11, "");
  drawLine(pos.point(), shape.p12, "");
  drawLine(pos.point(), shape.p22, "");
  drawLine(pos.point(), shape.p21, "");

  //---

#if 0
  // draw camera orbit
  drawData_.painter->setPen(QColor(100, 100, 100));

  drawSphere(origin.point(), pos.point());
#endif
}

void
Overview::
getCameraShape(Camera *camera, CameraShape &shape) const
{
  auto pos    = camera->position();
  auto origin = camera->origin();

  auto dist = CVector3D(pos, origin).length();

  auto front = camera->front();
  auto up    = camera->up   ();
  auto right = camera->right();

  auto fov = CMathGen::DegToRad(camera->fov());

  auto m1 = CMatrix3D::rotation(-fov/2.0, up   );
  auto m2 = CMatrix3D::rotation( fov/2.0, up   );
  auto m3 = CMatrix3D::rotation(-fov/2.0, right);
  auto m4 = CMatrix3D::rotation( fov/2.0, right);

  auto front1 = m1*front;
  auto front2 = m2*front;

  auto front11 = m3*front1;
  auto front12 = m4*front1;
  auto front21 = m3*front2;
  auto front22 = m4*front2;

  shape.p11 = (pos + dist*front11).point();
  shape.p12 = (pos + dist*front12).point();
  shape.p21 = (pos + dist*front21).point();
  shape.p22 = (pos + dist*front22).point();
}

void
Overview::
drawLine(const CPoint3D &p1, const CPoint3D &p2, const QString &) const
{
  auto px1 = windowToPixelX(CPoint2D(p1.x, p1.y));
  auto px2 = windowToPixelX(CPoint2D(p2.x, p2.y));

  drawData_.painter->setClipRect(xview_.rect);
  drawData_.painter->drawLine(px1.x, px1.y, px2.x, px2.y);

  auto py1 = windowToPixelY(CPoint2D(p1.z, p1.y));
  auto py2 = windowToPixelY(CPoint2D(p2.z, p2.y));

  drawData_.painter->setClipRect(yview_.rect);
  drawData_.painter->drawLine(py1.x, py1.y, py2.x, py2.y);

  auto pz1 = windowToPixelZ(CPoint2D(p1.x, p1.z));
  auto pz2 = windowToPixelZ(CPoint2D(p2.x, p2.z));

  drawData_.painter->setClipRect(zview_.rect);
  drawData_.painter->drawLine(pz1.x, pz1.y, pz2.x, pz2.y);

  auto pv1 = drawData_.worldMatrix*drawData_.viewMatrix*p1;
  auto pv2 = drawData_.worldMatrix*drawData_.viewMatrix*p2;

  auto pp1 = windowToPixelP(CPoint2D(pv1.x, pv1.y));
  auto pp2 = windowToPixelP(CPoint2D(pv2.x, pv2.y));

  drawData_.painter->setClipRect(pview_.rect);
  drawData_.painter->drawLine(pp1.x, pp1.y, pp2.x, pp2.y);
}

void
Overview::
drawVector(const CVector3D &p, const CVector3D &d, const QString &label) const
{
  auto drawVector2D = [&](const ViewData &view, const CPoint2D &p,
                          const CPoint2D &d, double sx, double sy, const QString &label) {
    drawData_.painter->setClipRect(view.rect);

    auto s = std::sqrt(sx*sx + sy*sy)/3.0;

    double px1, py1;
    view.range->windowToPixel(p.x, p.y, &px1, &py1);
    double px2, py2;
    view.range->windowToPixel(p.x + s*d.x, p.y + s*d.y, &px2, &py2);

    drawData_.painter->drawLine(px1, py1, px2, py2);

    if (label != "")
      drawData_.painter->drawText(px2, py2, label);
  };

  auto x1 = p.getX(), y1 = p.getY(), z1 = p.getZ();
  auto x2 = d.getX(), y2 = d.getY(), z2 = d.getZ();

  auto xs = drawData_.bbox.getXSize();
  auto ys = drawData_.bbox.getYSize();
  auto zs = drawData_.bbox.getZSize();

  drawVector2D(xview_, CPoint2D(x1, y1), CPoint2D(x2, y2), xs, ys, label); // XY
  drawVector2D(yview_, CPoint2D(z1, y1), CPoint2D(z2, y2), zs, ys, label); // ZY
  drawVector2D(zview_, CPoint2D(x1, z1), CPoint2D(x2, z2), xs, zs, label); // XZ
}

void
Overview::
drawPoint(const CVector3D &v, const QString &text) const
{
  drawPoint(v.point(), text);
}

void
Overview::
drawPoint(const CPoint3D &p, const QString &text) const
{
  double s = 8.0;

  drawData_.painter->setBrush(Qt::red);

  auto px = windowToPixelX(CPoint2D(p.x, p.y));
  auto py = windowToPixelY(CPoint2D(p.z, p.y));
  auto pz = windowToPixelZ(CPoint2D(p.x, p.z));

  drawData_.painter->setClipRect(xview_.rect);
  drawData_.painter->drawEllipse(QRectF(px.x - s/2, px.y - s/2, s, s));
  drawData_.painter->drawText(px.x, px.y, text);

  drawData_.painter->setClipRect(yview_.rect);
  drawData_.painter->drawEllipse(QRectF(py.x - s/2, py.y - s/2, s, s));
  drawData_.painter->drawText(py.x, py.y, text);

  drawData_.painter->setClipRect(zview_.rect);
  drawData_.painter->drawEllipse(QRectF(pz.x - s/2, pz.y - s/2, s, s));
  drawData_.painter->drawText(pz.x, pz.y, text);

  auto pv = drawData_.worldMatrix*drawData_.viewMatrix*p;
  auto pp = windowToPixelP(CPoint2D(pv.x, pv.y));

  drawData_.painter->setClipRect(pview_.rect);
  drawData_.painter->drawEllipse(QRectF(pp.x - s/2, pp.y - s/2, s, s));
  drawData_.painter->drawText(pp.x, pp.y, text);
}

CPoint2D
Overview::
windowToPixelX(const CPoint2D &w) const
{
  CPoint2D p;
  xview_.range->windowToPixel(w, p);
  return p;
}

CPoint2D
Overview::
windowToPixelY(const CPoint2D &w) const
{
  CPoint2D p;
  yview_.range->windowToPixel(w, p);
  return p;
}

CPoint2D
Overview::
windowToPixelZ(const CPoint2D &w) const
{
  CPoint2D p;
  zview_.range->windowToPixel(w, p);
  return p;
}

CPoint2D
Overview::
windowToPixelP(const CPoint2D &w) const
{
  CPoint2D p;
  pview_.range->windowToPixel(w, p);
  return p;
}

void
Overview::
mousePressEvent(QMouseEvent *e)
{
  mouseData_.pressPos = CPoint2D(e->x(), e->y());
  mouseData_.button   = e->button();
  mouseData_.pressed  = true;

  auto *camera = app_->canvas()->camera();

  if      (mouseData_.button == Qt::LeftButton) {
    if      (editType_ == EditType::SELECT) {
    }
    else if (editType_ == EditType::CAMERA) {
      auto position = camera->position();

      CPoint2D p;
      if (xview_.pressRange(mouseData_.pressPos, p))
        camera->setPosition(CVector3D(p.x, p.y, position.z())); // XY
      if (yview_.pressRange(mouseData_.pressPos, p))
        camera->setPosition(CVector3D(position.x(), p.y, p.x)); // ZY
      if (zview_.pressRange(mouseData_.pressPos, p))
        camera->setPosition(CVector3D(p.x, position.y(), p.y)); // XZ
    }
  }
  else if (mouseData_.button == Qt::MiddleButton) {
    if      (editType_ == EditType::SELECT) {
    }
    else if (editType_ == EditType::CAMERA) {
      auto origin = camera->origin();

      CPoint2D p;
      if (xview_.pressRange(mouseData_.pressPos, p))
        camera->setOrigin(CVector3D(p.x, p.y, origin.z())); // XY
      if (yview_.pressRange(mouseData_.pressPos, p))
        camera->setOrigin(CVector3D(origin.x(), p.y, p.x)); // ZY
      if (zview_.pressRange(mouseData_.pressPos, p))
        camera->setOrigin(CVector3D(p.x, origin.y(), p.y)); // XZ
    }
  }
  else if (mouseData_.button == Qt::RightButton) {
  }

  update();
}

void
Overview::
mouseReleaseEvent(QMouseEvent *)
{
  mouseData_.pressed = false;
}

void
Overview::
mouseMoveEvent(QMouseEvent *e)
{
  mouseData_.movePos = CPoint2D(e->x(), e->y());

//auto *camera = app_->canvas()->camera();

#if 0
  CPoint2D p;
  if (xview_.pressRange(mouseData_.movePos, p))
    app_->status()->setText(QString("X=%1 Y=%2").arg(p.x).arg(p.y));
  if (yview_.pressRange(mouseData_.movePos, p))
    app_->status()->setText(QString("Z=%1 Y=%2").arg(p.x).arg(p.y));
  if (zview_.pressRange(mouseData_.movePos, p))
    app_->status()->setText(QString("X=%1 Z=%2").arg(p.x).arg(p.y));
#endif

  //---

  if (! mouseData_.pressed)
    return;

  if      (mouseData_.button == Qt::LeftButton) {
#if 0
    auto position = camera->position();

    CPoint2D p;
    if (xview_.pressRange(mouseData_.movePos, p))
      camera->setPosition(CVector3D(p.x, p.y, position.z())); // XY
    if (yview_.pressRange(mouseData_.movePos, p))
      camera->setPosition(CVector3D(position.x(), p.y, p.x)); // ZY
    if (zview_.pressRange(mouseData_.movePos, p))
      camera->setPosition(CVector3D(p.x, position.y(), p.y)); // XZ
#endif
  }
  else if (mouseData_.button == Qt::MiddleButton) {
#if 0
    auto origin = camera->origin();

    CPoint2D p;
    if (xview_.pressRange(mouseData_.movePos, p))
      camera->setOrigin(CVector3D(p.x, p.y, origin.z())); // XY
    if (yview_.pressRange(mouseData_.movePos, p))
      camera->setOrigin(CVector3D(origin.x(), p.y, p.x)); // ZY
    if (zview_.pressRange(mouseData_.movePos, p))
      camera->setOrigin(CVector3D(p.x, origin.y(), p.y)); // XZ
#endif
  }
  else if (mouseData_.button == Qt::RightButton) {
  }

  update();
}

void
Overview::
wheelEvent(QWheelEvent *e)
{
  auto dw = e->angleDelta().y()/250.0;

  if      (dw < 0) {
    for (auto *v : views2d_)
      v->range->zoomOut(1.05);
  }
  else if (dw > 0) {
    for (auto *v : views2d_)
      v->range->zoomIn(1.05);
  }

  update();
}

void
Overview::
keyPressEvent(QKeyEvent *e)
{
  auto key = e->key();

  if (key == Qt::Key_R) {
    bboxSet_ = false;
    updateBBox();
  }

  update();
}

}
