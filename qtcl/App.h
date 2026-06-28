#ifndef App_H
#define App_H

#include <CGeom3DType.h>
#include <CVector3D.h>
#include <CPoint3D.h>
#include <CBBox3D.h>
#include <CBBox2D.h>
#include <CRGBA.h>

#include <QFrame>

#include <tcl/tcl.h>

class CGeomScene3D;
class CGeomObject3D;
class CGeomFace3D;
class CGeomEdge3D;
class CGeomLine3D;
class CGeomVertex3D;
class CGeomTexture;
class CGeomMaterial;

class CPSysParticle;
class CPSysSpring;
class CPSysAttraction;

class CTcl;

class QTimer;

namespace CQTclModel3DView {

class Canvas;
class Overview;
class Control;
class Toolbar;
class Sidebar;
class Status;
class Text;

class ParticleSystem;

class App : public QFrame {
  Q_OBJECT

 public:
  enum class SelectType {
    OBJECT,
    FACE,
    EDGE,
    POINT
  };

  enum class EditType {
    SELECT,
    CAMERA,
    TCL
  };

  class AnimData {
   public:
    AnimData(App *app, const std::string &proc) :
     app_(app), proc_(proc) {
    }

    virtual ~AnimData() { }

    bool isActive() const { return active_; }
    void setActive(bool b) { active_ = b; }

    bool isRepeat() const { return repeat_; }
    void setRepeat(bool b) { repeat_ = b; }

    virtual void tick() = 0;

   protected:
    App*        app_    { nullptr };
    std::string proc_;
    bool        active_ { false };
    bool        repeat_ { false };
  };

  class AnimRealData : public AnimData {
   public:
    AnimRealData(App *app, double start, double end, double delta, const std::string &proc) :
     AnimData(app, proc), start_(start), end_(end), delta_(delta) {
      value_ = start_;
    }

    void tick() override {
      if (value_ >= end_) {
        if (! repeat_) {
          active_ = false;
          return;
        }

        value_ = start_;
      }
      else {
        value_ += delta_;
      }

      app_->setVar("_value", value_);

      //std::cerr << "Value: " << value_ << " Proc: " << proc_ << "\n";

      app_->runTclCmd(proc_);
    }

   private:
    double start_ { 0 };
    double end_   { 10 };
    double delta_ { 1 };
    double value_ { 0 };
  };

  using AnimDatas = std::vector<AnimData *>;

 public:
  App();

  //---

  Canvas *canvas() const { return canvas_; }

  Overview *overview() const { return overview_; }

  Control *control() const { return control_; }

  Toolbar *toolbar() const { return toolbar_; }

  Sidebar *sidebar() const { return sidebar_; }

  Status *status() const { return status_; }

  CTcl *tcl() const { return tcl_; }

  //---

  const QString &buildDir() const { return buildDir_; }
  void setBuildDir(const QString &s) { buildDir_ = s; }

  CGeomScene3D *scene() const { return scene_; }

  //---

  const CPoint3D &cursor() const { return cursor_; }
  void setCursor(const CPoint3D &v) { cursor_ = v; }

  //---

  ParticleSystem *particleSystem() const { return psys_; }

  //---

  uint ticks() const { return ticks_; }

  bool isRunning() const { return running_; }
  void setRunning(bool b) { running_ = b; }

  //---

  int execFile(const std::string &filename);

  //---

  struct LoadData {
    CGeomObject3D* topObj { nullptr };
  };

  bool loadModel(const std::string &fileName, CGeom3DType format, LoadData &loadData);

  //---

  void setVar(const std::string &name, double value);
  void setVar(const std::string &name, const std::string &value);

  bool runTclCmd(const std::string &cmd);

 Q_SIGNALS:
  void modelAdded();

 private:
  void initTcl();

  bool decodeParticle  (Tcl_Obj *obj, CPSysParticle* &particle) const;
  bool decodeParticle  (const std::string &arg, CPSysParticle* &particle) const;
  bool decodeSpring    (const std::string &arg, CPSysSpring* &particle) const;
  bool decodeAttraction(const std::string &arg, CPSysAttraction* &particle) const;

  bool decodeObject  (const std::string &arg, CGeomObject3D* &object) const;
  bool decodeFaces   (const std::string &arg, std::vector<CGeomFace3D *> &faces) const;
  bool decodeFaces   (const std::vector<std::string> &args,
                      std::vector<CGeomFace3D *> &faces) const;
  bool decodeFace    (const std::string &arg, CGeomFace3D* &face) const;
  bool decodeEdge    (const std::string &arg, CGeomEdge3D* &edge) const;
  bool decodeLine    (const std::string &arg, CGeomLine3D* &line) const;
  bool decodeVertices(const std::string &arg, std::vector<CGeomVertex3D *> &vertices) const;
  bool decodeVertices(const std::vector<std::string> &args,
                      std::vector<CGeomVertex3D *> &vertices) const;
  bool decodeVertex  (const std::string &arg, CGeomVertex3D* &vertex) const;
  bool decodeTexture (Tcl_Obj *obj, CGeomTexture* &texture) const;
  bool decodeTexture (const std::string &arg, CGeomTexture* &texture) const;
  bool decodeMaterial(const std::string &arg, CGeomMaterial* &material) const;
  bool decodeText    (const std::string &arg, Text* &text) const;

  CGeomObject3D *getNearestObject(const CPoint3D &) const;
  CGeomFace3D   *getNearestFace  (CGeomObject3D *, const CPoint3D &) const;
  CGeomFace3D   *getNamedFace    (CGeomObject3D *, const std::string &name) const;
  CGeomEdge3D   *getNearestEdge  (CGeomObject3D *, const CPoint3D &) const;
  CGeomEdge3D   *getNearestEdge  (CGeomFace3D *, const CPoint3D &) const;
  CGeomVertex3D *getNearestVertex(CGeomObject3D *, const CPoint3D &) const;
  CGeomVertex3D *getNearestVertex(CGeomFace3D *, const CPoint3D &) const;

  bool stringToBBox  (const std::string &str, CBBox3D &bbox) const;
  bool decodePoint   (Tcl_Obj *str, CPoint2D &p) const;
  bool stringToPoint (const std::string &str, CPoint2D &p) const;
  bool decodePoint   (Tcl_Obj *str, CPoint3D &p) const;
  bool stringToPoint (const std::string &str, CPoint3D &p) const;
  bool stringToVector(const std::string &str, CVector3D &v) const;
  bool stringToRect  (const std::string &str, CBBox2D &r) const;
  bool decodeReal    (Tcl_Obj *obj, double &r) const;
  bool decodeBool    (Tcl_Obj *obj, bool &b) const;

  bool stringToColor(const std::string &str, CRGBA &c) const;

 public:
  using StringList = std::vector<std::string>;

  int getAppValueProc(const StringList &);
  int setAppValueProc(const StringList &);

  int addViewportProc     (const StringList &);
  int setViewportValueProc(const StringList &);

  int addShaderProc     (const StringList &);
  int setShaderValueProc(const StringList &);

  int addParticleProc     (const StringList &);
  int getParticleValueProc(const StringList &);
  int setParticleValueProc(const std::vector<Tcl_Obj *> &);

  int addSpringProc     (const StringList &);
  int getSpringValueProc(const StringList &);
  int setSpringValueProc(const StringList &);

  int addAttractionProc     (const StringList &);
  int getAttractionValueProc(const StringList &);
  int setAttractionValueProc(const StringList &);

  int getObjectValueProc  (const StringList &);
  int setObjectValueProc  (const StringList &);
  int getFaceValueProc    (const StringList &);
  int setFaceValueProc    (const StringList &);
  int getLineValueProc    (const StringList &);
  int setLineValueProc    (const StringList &);
  int getEdgeValueProc    (const StringList &);
  int setEdgeValueProc    (const StringList &);
  int getVertexValueProc  (const StringList &);
  int setVertexValueProc  (const StringList &);
  int setMaterialValueProc(const StringList &);
  int getTextValueProc    (const StringList &);
  int setTextValueProc    (const StringList &);

  int getObjectPropertyProc(const StringList &);
  int setObjectPropertyProc(const StringList &);

  int intersectObjectsProc(const StringList &);
  int inverseObjectProc   (const StringList &);
  int unionObjectsProc    (const StringList &);
  int subtractObjectsProc (const StringList &);

  int extrudeFacesProc (const StringList &);
  int extrudeEdgesProc (const StringList &);
  int mergeEdgeProc    (const StringList &);
  int duplicateFaceProc(const StringList &);
  int separateFaceProc (const StringList &);
  int separateEdgeProc (const StringList &);
  int mirrorObjectProc (const StringList &);
  int fillVerticesProc (const StringList &);

  int addObjectProc  (const StringList &);
  int addVertexProc  (const StringList &);
  int addFaceProc    (const StringList &);
  int addLineProc    (const StringList &);
  int addMaterialProc(const StringList &);
  int addTextureProc (const StringList &);
  int addTextProc    (const StringList &);

  int addPlaneProc   (const StringList &);
  int addCubeProc    (const StringList &);
  int addConeProc    (const StringList &);
  int addCylinderProc(const StringList &);
  int addSphereProc  (const StringList &);
  int addTerrainProc (const StringList &);

  int scaleFacesProc      (const StringList &);
  int circularizeFacesProc(const StringList &);

  int deleteObjectsProc (const StringList &);
  int deleteFacesProc   (const StringList &);
  int deleteVerticesProc(const StringList &);

  int animateRealProc(const StringList &);

  int readModelProc(const StringList &);
  int writeObjProc (const StringList &);

  int vectorProc    (const StringList &);
  int getVectorProc (const std::vector<Tcl_Obj *> &objs);
  int setVectorProc (const std::vector<Tcl_Obj *> &objs);
  int calcVectorProc(const StringList &);

  //---

  void showMetaEdit();
  void showPerfDialog();
  void showAppOptions();

 private Q_SLOTS:
  void timerSlot();

  void tick(bool update=true);

 private:
  QString buildDir_;

  // object data
  CGeomScene3D* scene_ { nullptr };

  Canvas*   canvas_   { nullptr };
  Overview* overview_ { nullptr };
  Control*  control_  { nullptr };
  Toolbar*  toolbar_  { nullptr };
  Sidebar*  sidebar_  { nullptr };
  Status*   status_   { nullptr };

  CTcl* tcl_ { nullptr };

  CPoint3D cursor_ { 0, 0, 0 };

  ParticleSystem *psys_ { nullptr };

  QTimer* timer_   { nullptr };
  uint    ticks_   { 0 };
  bool    running_ { true };

  AnimDatas animDatas_;
};

}

#endif
