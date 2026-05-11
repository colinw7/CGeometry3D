#ifndef App_H
#define App_H

#include <CPoint3D.h>

#include <vector>
#include <string>

#include <tcl/tcl.h>

class CGeomScene3D;
class CGeomObject3D;
class CGeomFace3D;
class CGeomEdge3D;
class CGeomVertex3D;

class CTcl;

namespace CTclGeometry3D {

class App {
 public:
  App();

  const CPoint3D &cursor() const { return cursor_; }
  void setCursor(const CPoint3D &v) { cursor_ = v; }

  int execFile(const std::string &filename);

 private:
  static int addObjectProc  (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int addVertexProc  (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int addFaceProc    (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int addMaterialProc(ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int addTextureProc (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);

  static int addPlaneProc   (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int addConeProc    (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int addCubeProc    (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int addCylinderProc(ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int addSphereProc  (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int addTerrainProc (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);

  static int getAppValueProc     (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int setAppValueProc     (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int getObjectValueProc  (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int setObjectValueProc  (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int getFaceValueProc    (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int setFaceValueProc    (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int getEdgeValueProc    (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int setEdgeValueProc    (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int getVertexValueProc  (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int setMaterialValueProc(ClientData, Tcl_Interp*, int, Tcl_Obj * const *);

  static int intersectObjectsProc(ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int inverseObjectProc   (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int unionObjectsProc    (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int subtractObjectsProc (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);

  static int extrudeFaceProc (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int extrudeEdgeProc (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int mergeEdgeProc   (ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int separateFaceProc(ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int separateEdgeProc(ClientData, Tcl_Interp*, int, Tcl_Obj * const *);
  static int mirrorObjectProc(ClientData, Tcl_Interp*, int, Tcl_Obj * const *);

  static int deleteObjectsProc(ClientData, Tcl_Interp*, int, Tcl_Obj * const *);

  static int writeObjProc(ClientData, Tcl_Interp*, int, Tcl_Obj * const *);

  bool decodeObject      (const std::string &arg, CGeomObject3D* &object) const;
  bool decodeObjectFace  (const std::string &arg, CGeomFace3D* &face) const;
  bool decodeObjectEdge  (const std::string &arg, CGeomEdge3D* &edge) const;
  bool decodeObjectVertex(const std::string &arg, CGeomVertex3D* &vertex) const;

  CGeomObject3D *getNearestObject(const CPoint3D &) const;
  CGeomFace3D   *getNearestFace(CGeomObject3D *, const CPoint3D &) const;
  CGeomEdge3D   *getNearestEdge(CGeomObject3D *, const CPoint3D &) const;
  CGeomVertex3D *getNearestVertex(CGeomObject3D *, const CPoint3D &) const;

  bool stringToPoint(const std::string &str, CPoint3D &p) const;

 public:
  int readObjProc(const std::vector<std::string> &args);

 private:
  CTcl*         tcl_   { nullptr };
  CGeomScene3D* scene_ { nullptr };

  CPoint3D cursor_ { 0, 0, 0 };
};

}

#endif
