set plane [addPlane 6 4]

set faces [getObjectValue $plane faces]

set face1 [lindex $faces 0]
setFaceValue $face1 normal {0 1 0}
setFaceValue $face1 color red
echo [getFaceValue $face1 bbox]

set faces2 [extrudeFaces $face1 4]
set face2 [lindex $faces2 0]
setFaceValue $face2 color green
echo [getFaceValue $faces2 bbox]

set faces3 [extrudeFaces $face2 4]
set face3 [lindex $faces3 0]
setFaceValue $face3 color blue
echo [getFaceValue $face3 bbox]

proc printFaceEdges { face } {
  set edges [getFaceValue $face edges]
  echo "Face $face : Edges $edges"

  foreach edge $edges {
    set start [getVertexValue [getEdgeValue $edge start] model]
    set end   [getVertexValue [getEdgeValue $edge end  ] model]

    echo "  $edge : $start $end"
  }
}

#printFaceEdges $face3

set edge1 [getFaceValue $face3 nearest_edge [list 3 8 0]]
setEdgeValue $edge1 selected 1

mergeEdge $edge1

#printFaceEdges $face3

set edge1 [getFaceValue $face3 nearest_edge [list -3 8 0]]
setEdgeValue $edge1 selected 1

mergeEdge $edge1

#printFaceEdges $face3

#writeObj "house.obj"
