set cube [addCube 2]

set edges [getObjectValue $cube edges]

echo "edges: $edges"

proc getNearestEdge { obj p } {
  set edge [getObjectValue $obj nearest_edge $p]

  set p1 [getVertexValue [getEdgeValue $edge start] model]
  set p2 [getVertexValue [getEdgeValue $edge end  ] model]

  echo "$p : Edge: $edge ($p1 -> $p2)"
}

getNearestEdge $cube {-1  1  0}
getNearestEdge $cube { 1  1  0}
getNearestEdge $cube { 0  1 -1}
getNearestEdge $cube { 0  1  1}
getNearestEdge $cube {-1  0 -1}
getNearestEdge $cube { 1  0 -1}
getNearestEdge $cube {-1  0  1}
getNearestEdge $cube { 1  0  1}
getNearestEdge $cube {-1 -1  0}
getNearestEdge $cube { 1 -1  0}
getNearestEdge $cube { 0 -1 -1}
getNearestEdge $cube { 0 -1  1}
