#include <CGeomUtil3D.h>

CPoint3D
CGeomUtil3D::
getMidPoint(const std::vector<CGeomVertex3D *> &vertices)
{
  CPoint3D mid_point(0,0,0);

  double n1 = 1.0/vertices.size();

  std::vector<CGeomVertex3D *>::const_iterator p1 = vertices.begin();
  std::vector<CGeomVertex3D *>::const_iterator p2 = vertices.end  ();

  for ( ; p1 != p2; ++p1)
    mid_point += n1*(*p1)->getViewed();

  return mid_point;
}

CVector3D
CGeomUtil3D::
getNormal(const std::vector<CGeomVertex3D *> &vertices)
{
  CVector3D diff1(vertices[0]->getViewed(), vertices[1]->getViewed());
  CVector3D diff2(vertices[1]->getViewed(), vertices[2]->getViewed());

  return diff1.crossProduct(diff2).normalized();
}

//------------------------

class CTriangulate3D {
 private:
  struct EarPoint {
    const CPoint3D &point;
    bool            is_ear;
    CPoint2D        point2;

    EarPoint(const CPoint3D &p) :
     point(p), is_ear(false), point2(p.x, p.y) {
    }

    bool operator==(const EarPoint &ep) {
      return (&point == &ep.point);
    }
  };

  const std::list<CPoint3D> &points_;

 public:
  CTriangulate3D(const std::list<CPoint3D> points);

  void exec(std::vector<CTriangle3D> &triangle_list);

 private:
  void addTriangle(std::vector<CTriangle3D> &triangle_list,
                   const std::list<EarPoint>::const_iterator &ep1,
                   const std::list<EarPoint>::const_iterator &ep2,
                   const std::list<EarPoint>::const_iterator &ep3);

  bool isDiagonal(const std::list<EarPoint> &ear_points,
                  const std::list<EarPoint>::const_iterator &epa,
                  const std::list<EarPoint>::const_iterator &epb);

  bool isDiagonalInOut(const std::list<EarPoint> &ear_points,
                       const std::list<EarPoint>::const_iterator &epa,
                       const std::list<EarPoint>::const_iterator &epb);

  bool inCone(const std::list<EarPoint> &ear_points,
              const std::list<EarPoint>::const_iterator &ep1,
              const std::list<EarPoint>::const_iterator &epb);

  bool Intersects(const std::list<EarPoint>::const_iterator &ep1,
                  const std::list<EarPoint>::const_iterator &ep2,
                  const std::list<EarPoint>::const_iterator &ep3,
                  const std::list<EarPoint>::const_iterator &ep4);

  bool PointLineLeftOn(const std::list<EarPoint>::const_iterator &ep1,
                       const std::list<EarPoint>::const_iterator &ep2,
                       const std::list<EarPoint>::const_iterator &ep3);

  bool PointLineLeft(const std::list<EarPoint>::const_iterator &ep1,
                     const std::list<EarPoint>::const_iterator &ep2,
                     const std::list<EarPoint>::const_iterator &ep3);
};

void
CGeomUtil3D::
triangulate(const std::list<CPoint3D> points, std::vector<CTriangle3D> &triangle_list)
{
  CTriangulate3D triangulate(points);

  triangulate.exec(triangle_list);
}

CTriangulate3D::
CTriangulate3D(const std::list<CPoint3D> points) :
 points_(points)
{
}

void
CTriangulate3D::
exec(std::vector<CTriangle3D> &triangle_list)
{
  std::list<EarPoint> ear_points;

  std::list<CPoint3D>::const_iterator ps = points_.begin();
  std::list<CPoint3D>::const_iterator pe = points_.end  ();

  for ( ; ps != pe; ++ps)
    ear_points.push_back(EarPoint(*ps));

  std::list<EarPoint>::iterator eps = ear_points.begin();
  std::list<EarPoint>::iterator epe = ear_points.end  ();

  std::list<EarPoint>::iterator ep3 = eps;
  std::list<EarPoint>::iterator ep1 = ep3++;
  std::list<EarPoint>::iterator ep2 = ep3++;

  std::list<EarPoint>::iterator ep4, ep5;

  do {
    (*ep2).is_ear = isDiagonal(ear_points, ep1, ep3);

    ep1 = ep2;
    ep2 = ep3++;

    if (ep3 == epe) ep3 = eps;
  } while (ep1 != eps);

  while (ear_points.size() > 3) {
    ep5 = eps;
    ep1 = ep5++;
    ep2 = ep5++;
    ep3 = ep5++;
    ep4 = ep5++;

    if (ep5 == epe) ep5 = eps;

    do {
      if ((*ep3).is_ear) {
        addTriangle(triangle_list, ep3, ep2, ep4);

        (*ep2).is_ear = isDiagonal(ear_points, ep1, ep4);
        (*ep4).is_ear = isDiagonal(ear_points, ep2, ep5);

        ear_points.erase(ep3);

        eps = ear_points.begin();
        epe = ear_points.end  ();

        break;
      }

      ep1 = ep2;
      ep2 = ep3;
      ep3 = ep4;
      ep4 = ep5++;

      if (ep5 == epe) ep5 = eps;
    } while (ep1 != eps);
  }

  ep3 = eps;
  ep1 = ep3++;
  ep2 = ep3++;

  addTriangle(triangle_list, ep1, ep2, ep3);
}

void
CTriangulate3D::
addTriangle(std::vector<CTriangle3D> &triangle_list,
            const std::list<EarPoint>::const_iterator &ep1,
            const std::list<EarPoint>::const_iterator &ep2,
            const std::list<EarPoint>::const_iterator &ep3)
{
  std::cout << "Diagonal " << (*ep2).point << "->" << (*ep3).point << std::endl;

  triangle_list.push_back(
    CTriangle3D((*ep1).point, (*ep2).point, (*ep3).point));
}

bool
CTriangulate3D::
isDiagonal(const std::list<EarPoint> &ear_points,
           const std::list<EarPoint>::const_iterator &epa,
           const std::list<EarPoint>::const_iterator &epb)
{
  return inCone(ear_points, epa, epb) &&
         inCone(ear_points, epb, epa) &&
         isDiagonalInOut(ear_points, epa, epb);
}

bool
CTriangulate3D::
isDiagonalInOut(const std::list<EarPoint> &ear_points,
                const std::list<EarPoint>::const_iterator &epa,
                const std::list<EarPoint>::const_iterator &epb)
{
  std::list<EarPoint>::const_iterator eps = ear_points.begin();
  std::list<EarPoint>::const_iterator epe = ear_points.end  ();

  std::list<EarPoint>::const_iterator ep2 = eps;
  std::list<EarPoint>::const_iterator ep1 = ep2++;

  do {
    if (ep1 != epa && ep2 != epa && ep1 != epb && ep2 != epb) {
      if (Intersects(epa, epb, ep1, ep2))
        return false;
    }

    ep1 = ep2++;

    if (ep2 == epe) ep2 = eps;
  } while (ep1 != eps);

  return true;
}

bool
CTriangulate3D::
inCone(const std::list<EarPoint> &ear_points,
       const std::list<EarPoint>::const_iterator &ep1,
       const std::list<EarPoint>::const_iterator &epb)
{
  std::list<EarPoint>::const_iterator eps = ear_points.begin();
  std::list<EarPoint>::const_iterator epe = ear_points.end();

  std::list<EarPoint>::const_iterator ep0 = ep1; --ep0;
  std::list<EarPoint>::const_iterator ep2 = ep1; ++ep2;

  if (ep0 == epe) ep0 = (++ear_points.rbegin()).base(); // last element
  if (ep2 == epe) ep2 = eps;

  if (PointLineLeftOn(ep1, ep2, ep0))
    return    PointLineLeft(ep1, epb, ep0) && PointLineLeft(epb, ep1, ep2);
  else
    return ! (PointLineLeft(ep1, epb, ep2) && PointLineLeft(epb, ep1, ep0));
}

bool
CTriangulate3D::
Intersects(const std::list<EarPoint>::const_iterator &ep1,
           const std::list<EarPoint>::const_iterator &ep2,
           const std::list<EarPoint>::const_iterator &ep3,
           const std::list<EarPoint>::const_iterator &ep4)
{
  CLine2D line1((*ep1).point2, (*ep2).point2);
  CLine2D line2((*ep3).point2, (*ep4).point2);

  return line1.intersects(line2);
}

bool
CTriangulate3D::
PointLineLeftOn(const std::list<EarPoint>::const_iterator &ep1,
                const std::list<EarPoint>::const_iterator &ep2,
                const std::list<EarPoint>::const_iterator &ep3)
{
  CLine2D line((*ep1).point2, (*ep2).point2);

  return line.leftOrOn((*ep3).point2);
}

bool
CTriangulate3D::
PointLineLeft(const std::list<EarPoint>::const_iterator &ep1,
              const std::list<EarPoint>::const_iterator &ep2,
              const std::list<EarPoint>::const_iterator &ep3)
{
  CLine2D line((*ep1).point2, (*ep2).point2);

  return line.left((*ep3).point2);
}
