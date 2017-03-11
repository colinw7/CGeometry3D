#include <CGeomTorus3D.h>
#include <CGeometry3D.h>
#include <CMathGen.h>
#include <CConv.h>

CGeomTorus3D::
CGeomTorus3D(CGeomScene3D *pscene, const std::string &name,
             double xc, double yc, double zc, double r0, double r1,
             double p1, double p2, int nu, int nv) :
 CGeomObject3D(pscene, name)
{
  addGeometry(this, xc, yc, zc, r0, r1, p1, p2, nu, nv);
}

void
CGeomTorus3D::
addGeometry(CGeomObject3D *object, double xc, double yc, double zc,
            double r0, double r1, double p1, double p2, int nu, int nv)
{
  uint vertex1, vertex2, vertex3, vertex4;

  double x, y, z;
  double phi, cphi, sphi;
  double theta, ctheta, stheta;

  int du = 360/nu;
  int dv = 360/nv;

  for (int u = 0; u < 360; u += du) {
    for (int v = 0; v < 360; v += dv) {
      theta = CMathGen::DegToRad(u);
      phi   = CMathGen::DegToRad(v);

      ctheta = power(cos(theta), p1);
      stheta = power(sin(theta), p1);

      cphi = power(cos(phi), p2);
      sphi = power(sin(phi), p2);

      x = ctheta * (r0 + r1 * cphi);
      y = stheta * (r0 + r1 * cphi);
      z = r1 * sphi;

      CPoint3D point1(xc + x, yc + y, zc + z);

      if (! object->findVertex(point1, &vertex1))
        vertex1 = object->addVertex(point1);

      //----

      theta = CMathGen::DegToRad(u + du);
      phi   = CMathGen::DegToRad(v);

      ctheta = power(cos(theta), p1);
      stheta = power(sin(theta), p1);

      cphi = power(cos(phi), p2);
      sphi = power(sin(phi), p2);

      x = ctheta * (r0 + r1 * cphi);
      y = stheta * (r0 + r1 * cphi);
      z = r1 * sphi;

      CPoint3D point2(xc + x, yc + y, zc + z);

      if (! object->findVertex(point2, &vertex2))
        vertex2 = object->addVertex(point2);

      //----

      theta = CMathGen::DegToRad(u + du);
      phi   = CMathGen::DegToRad(v + dv);

      ctheta = power(cos(theta), p1);
      stheta = power(sin(theta), p1);

      cphi = power(cos(phi), p2);
      sphi = power(sin(phi), p2);

      x = ctheta * (r0 + r1 * cphi);
      y = stheta * (r0 + r1 * cphi);
      z = r1 * sphi;

      CPoint3D point3(xc + x, yc + y, zc + z);

      if (! object->findVertex(point3, &vertex3))
        vertex3 = object->addVertex(point3);

      //----

      theta = CMathGen::DegToRad(u);
      phi   = CMathGen::DegToRad(v + dv);

      ctheta = power(cos(theta), p1);
      stheta = power(sin(theta), p1);

      cphi = power(cos(phi), p2);
      sphi = power(sin(phi), p2);

      x = ctheta * (r0 + r1 * cphi);
      y = stheta * (r0 + r1 * cphi);
      z = r1 * sphi;

      CPoint3D point4(xc + x, yc + y, zc + z);

      if (! object->findVertex(point4, &vertex4))
        vertex4 = object->addVertex(point4);

      //----

      object->addFace(CConv::toVector(vertex1, vertex2, vertex3, vertex4));
    }
  }
}

double
CGeomTorus3D::
power(double f, double p)
{
  double absf = fabs(f);

  if (absf < 0.00001)
    return 0.0;

  double pp = pow(absf, p);

  if (f < 0)
    return -pp;
  else
    return pp;
}
