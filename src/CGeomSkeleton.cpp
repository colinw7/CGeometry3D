#include <CGeomSkeleton.h>

CGeomSkeleton::
CGeomSkeleton()
{
}

CGeomSkeleton::
~CGeomSkeleton()
{
}

void
CGeomSkeleton::
clear()
{
  nodes  .clear();
  nodeIds.clear();

  meshNode = -1;
  rootNode = -1;
}
