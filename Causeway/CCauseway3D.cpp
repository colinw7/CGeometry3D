#include <CCauseway3D.h>
#include <CMathRand.h>

struct Point;

int                  pointInd = 1;
std::vector<Point *> points;

struct Point {
  double x   { 0.0 };
  double y   { 0.0 };
  double z   { 0.0 };
  int    ind { -1 };

  Point() { }

  Point(double x, double y) :
   x(x), y(y) {
    points.push_back(this);

    ind = int(points.size());
  }

  Point(double x, double y, double z) :
   x(x), y(y), z(z) {
    points.push_back(this);

    ind = int(points.size());
  }
};

int faceInd = 1;

struct Face {
  std::vector<Point *> points;
  int                  ind { -1 };

  Face() {
    ind = faceInd++;
  }
};

int shapeInd = 1;

struct Shape {
  std::vector<Face *> faces;
  int                 ind { -1 };

  Shape() {
    ind = shapeInd++;
  }
};

int
main(int argc, char **argv)
{
  for (int i = 1; i < argc; ++i) {
    if (argv[i][0] == '-') {
    }
  }

  //---

  std::vector<Shape *> shapes;

  auto genPillar = [&](double xc, double yc, double r, double h) {
    auto *shape = new Shape;

    int    ns = 6;
    double da = 2.0*M_PI/ns;

    auto *face1 = new Face;

    for (int is = 0; is < ns; ++is) {
      auto x = xc + r*std::cos(is*da);
      auto y = yc + r*std::sin(is*da);

      x += CMathRand::randInRange(-r/10, r/10);
      y += CMathRand::randInRange(-r/10, r/10);

      face1->points.push_back(new Point(x, 0.0, y));
    }

    shape->faces.push_back(face1);

    auto *face2 = new Face;

    for (const auto &p : face1->points) {
      face2->points.push_back(new Point(p->x, p->y + h, p->z));
    }

    shape->faces.push_back(face2);

    for (int i = 0; i < 6; ++i) {
      int i1 = (i + 1) % 6;

      auto *face3 = new Face;

      face3->points.push_back(face1->points[size_t(i )]);
      face3->points.push_back(face1->points[size_t(i1)]);
      face3->points.push_back(face2->points[size_t(i1)]);
      face3->points.push_back(face2->points[size_t(i )]);

      shape->faces.push_back(face3);
    }

    shapes.push_back(shape);
  };

  //---

  double xc { 0.0 }, yc { 0.0 };

  auto r1 = CMathRand::randInRange(0.9, 1.1);
  auto h1 = CMathRand::randInRange(0.9, 1.1);

  genPillar(xc, yc, r1, h1);

  for (int ir = 1; ir <= 3; ++ir) {
    int    ns = ir*6;
    double r  = (ir & 1 ? std::sqrt(3.0) : 2.0);
    double da = 2.0*M_PI/ns;

    for (int is = 0; is < ns; ++is) {
      auto x = r*std::cos(is*da + da/2.0);
      auto y = r*std::sin(is*da + da/2.0);

      auto r2 = CMathRand::randInRange(0.7, 0.9);
      auto h2 = CMathRand::randInRange(0.7, 0.9);

      x += CMathRand::randInRange(-r/10, r/10);
      y += CMathRand::randInRange(-r/10, r/10);

      genPillar(xc + x, yc + y, r2, h2);
    }
  }

  //---

  std::cout << "g\n";

  for (auto *p : points)
    std::cout << "v " << p->x << " " << p->y << " " << p->z << "\n";

  std::cout << "\n";

  for (auto *shape : shapes) {
    std::cout << "g " << shape->ind << "\n";

    for (auto *face : shape->faces) {
      std::cout << "f";

      for (auto *p : face->points) {
        std::cout << " " << p->ind;
      }

      std::cout << "\n";
    }
  }

  return 0;
}
