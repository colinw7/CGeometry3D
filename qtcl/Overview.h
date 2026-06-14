#ifndef Overview_H
#define Overview_H

#include <CWindowRange2D.h>
#include <CMatrix3DH.h>
#include <CBBox3D.h>
#include <CPoint3D.h>
#include <CRGBA.h>

#include <QFrame>

namespace CQTclModel3DView {

class App;
class Shape;
class Camera;
class Light;

class Overview : public QFrame {
  Q_OBJECT

 public:
  enum class ViewType {
    NONE,
    XY,
    ZY,
    XZ,
    THREED
  };

  enum class EditType {
    SELECT,
    CAMERA,
    LIGHT
  };

 private:
  struct CameraShape {
    CPoint3D p11;
    CPoint3D p12;
    CPoint3D p21;
    CPoint3D p22;
  };

 public:
  Overview(App *app);
 ~Overview();

  App *app() { return app_; }

  const EditType &editType() const { return editType_; }
  void setEditType(const EditType &v) { editType_ = v; }

  bool isShowCamera() const { return showCamera_; }
  void setShowCamera(bool b) { showCamera_ = b; }

  bool isWireframe() const { return wireframe_; }
  void setWireframe(bool b) { wireframe_ = b; }

  bool isSolid() const { return solid_; }
  void setSolid(bool b) { solid_ = b; }

  //---

  void resizeEvent(QResizeEvent *) override;
  void paintEvent (QPaintEvent  *) override;

  void mousePressEvent  (QMouseEvent *) override;
  void mouseMoveEvent   (QMouseEvent *) override;
  void mouseReleaseEvent(QMouseEvent *) override;

  void wheelEvent(QWheelEvent *e) override;

  void keyPressEvent(QKeyEvent *e) override;

  //---

  void updateModel();

  void drawModel();

  void drawCamera(Camera *camera);

  void drawLight(Light *light);

  void updateBBox();

 private:
  void updateRange();

  void drawLine(const CPoint3D &, const CPoint3D &, const QString &) const;
  void drawVector(const CVector3D &, const CVector3D &, const QString &) const;
  void drawPoint(const CVector3D &, const QString &) const;
  void drawPoint(const CPoint3D &, const QString &) const;

  CPoint2D windowToPixelX(const CPoint2D &p) const;
  CPoint2D windowToPixelY(const CPoint2D &p) const;
  CPoint2D windowToPixelZ(const CPoint2D &p) const;
  CPoint2D windowToPixelP(const CPoint2D &p) const;

  void getCameraShape(Camera *camera, CameraShape &shape) const;

 private Q_SLOTS:
  void invalidate();

 private:
  App *app_ { nullptr };

  struct ViewData {
    CDisplayRange2D* range { nullptr };
    QRectF           rect;
    int              ind  { -1 };
    ViewType         type { ViewType::NONE };
    QString          name;

    CPoint2D pixelToView(const QPointF &p) const {
      CPoint2D p1(p.x(), p.y());
      CPoint2D p2;

      if      (type == ViewType::XY) {
        (void) pressRange(p1, p2);
      }
      else if (type == ViewType::ZY) {
        (void) pressRange(p1, p2);
      }
      else if (type == ViewType::XZ) {
        (void) pressRange(p1, p2);
      }
      else
        assert(false);

      return p2;
    }

    bool pressRange(const CPoint2D &p, CPoint2D &p1) const {
      double xmin, ymin, xmax, ymax;
      range->getPixelRange(&xmin, &ymin, &xmax, &ymax);

      if (xmin > xmax) std::swap(xmin, xmax);
      if (ymin > ymax) std::swap(ymin, ymax);

      bool rc = (p.x < xmin || p.y < ymin || p.x > xmax || p.y > ymax);

      double x1, y1;
      range->pixelToWindow(p.x, p.y, &x1, &y1);

      p1 = CPoint2D(x1, y1);

      return rc;
    }

    CPoint2D viewPoint(const CPoint3D &p) const {
      CPoint2D p1;

      if      (type == ViewType::XY)
        p1 = CPoint2D(p.x, p.y);
      else if (type == ViewType::ZY)
        p1 = CPoint2D(p.z, p.y);
      else if (type == ViewType::XZ)
        p1 = CPoint2D(p.x, p.z);
      else
        assert(false);

      return p1;
    }
  };

  //---

  struct Face {
    CRGBA                 color;
    std::vector<CPoint3D> points;
  };

  using Faces = std::vector<Face>;

  //---

  struct DrawData {
    QPainter* painter { nullptr };

    CMatrix3DH worldMatrix;
    CMatrix3DH viewMatrix;
    Faces      faces;
    CBBox3D    bbox;
  };

  //---

  struct MouseData {
    int      button   { Qt::LeftButton };
    bool     pressed  { false };
    CPoint2D pressPos;
    CPoint2D movePos;

    bool      raySet { false };
    CPoint3D  rayOrigin;
    CVector3D rayDirection;
  };

  //---

  ViewData xview_;
  ViewData yview_;
  ViewData zview_;
  ViewData pview_;

  std::vector<ViewData *> views_;   // all views
  std::vector<ViewData *> views2d_; // 2d views

  int w_ { 100 };
  int h_ { 100 };

  bool bboxSet_    { false };
  bool facesValid_ { false };

  CBBox3D bbox_;

  MouseData mouseData_;
  DrawData  drawData_;

  EditType editType_ { EditType::LIGHT };

  bool showCamera_ { false };
  bool wireframe_  { true };
  bool solid_      { false };
};

}

#endif
