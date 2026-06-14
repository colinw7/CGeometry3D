#include <CPSysVector3D.h>

std::string
CPSysVector3D::
toString() const
{
  std::string str = "(";

  return str + x_ + ", " + y_ + ", " + z_ + ")";
}

