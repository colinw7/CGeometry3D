#ifndef CGeomSkeleton_H
#define CGeomSkeleton_H

#include <CGeomNodeData.h>

struct CGeomSkeleton {
  CGeomSkeleton();
 ~CGeomSkeleton();

  void clear();

  using NodeDatas = std::map<int, CGeomNodeData>;
  using NodeIds   = std::vector<int>;

  NodeDatas nodes;
  NodeIds   nodeIds;
  int       meshNode { -1 };
  int       rootNode { -1 };
};

#endif
