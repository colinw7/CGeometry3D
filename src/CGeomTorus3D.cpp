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

CGeomTorus3D::
CGeomTorus3D(CGeomScene3D *pscene, const std::string &name, const CPoint3D &c,
             double r0, double r1, double p1, double p2, int nu, int nv) :
 CGeomObject3D(pscene, name)
{
  addGeometry(this, c, r0, r1, p1, p2, nu, nv);
}

void
CGeomTorus3D::
addGeometry(CGeomObject3D *object, const CPoint3D &c, double r0, double r1,
            double p1, double p2, int nu, int nv)
{
  addGeometry(object, c.x, c.y, c.z, r0, r1, p1, p2, nu, nv);
}

void
CGeomTorus3D::
addGeometry(CGeomObject3D *object, double xc, double yc, double zc,
            double r0, double r1, double p1, double p2, int nu, int nv)
{
  // rotate circle of radius r1 at radius r0 around center (xc, yc, zc)
  int du = 360/nu;
  int dv = 360/nv;

  for (int u = 0; u < 360; u += du) {
    for (int v = 0; v < 360; v += dv) {
      auto addPoint = [&](int u1, int v1) {
        auto theta  = CMathGen::DegToRad(u1);
        auto ctheta = power(std::cos(theta), p1);
        auto stheta = power(std::sin(theta), p1);

        auto phi  = CMathGen::DegToRad(v1);
        auto cphi = power(std::cos(phi), p2);
        auto sphi = power(std::sin(phi), p2);

        auto x = ctheta*(r0 + r1*cphi);
        auto y = stheta*(r0 + r1*cphi);
        auto z = r1*sphi;

        auto point1 = CPoint3D(xc + x, yc + y, zc + z);

        uint vind;
        if (! object->findVertex(point1, &vind)) {
          vind = object->addVertex(point1);

          auto &vertex = object->getVertex(vind);

          auto x1 = ctheta*r0;
          auto y1 = stheta*r0;
          auto z1 = 0.0;

          auto n = CVector3D(CPoint3D(xc + x1, yc + y1, zc + z1), point1);

          vertex.setNormal(n);
        }

        return vind;
      };

      //----

      auto vertex1 = addPoint(u     , v     );
      auto vertex2 = addPoint(u + du, v     );
      auto vertex3 = addPoint(u + du, v + dv);
      auto vertex4 = addPoint(u     , v + dv);

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
