set plane [addPlane 6 4]

set faces [getObjectValue $plane faces]

set face1 [lindex $faces 0]
setFaceValue $face1 normal {0 1 0}
setFaceValue $face1 color red
echo [getFaceValue $face1 bbox]

set face2 [extrudeFace $face1 4]
setFaceValue $face2 color green
echo [getFaceValue $face2 bbox]

set face3 [extrudeFace $face2 4]
setFaceValue $face3 color blue
echo [getFaceValue $face3 bbox]

set edges [getFaceValue $face3 edges]
echo $edges

foreach edge $edges {
  set start [getVertexValue [getEdgeValue $edge start] model]
  set end   [getVertexValue [getEdgeValue $edge end  ] model]

  echo "$edge : $start $end"
}

mergeEdge [lindex $edges 0]

set edges [getFaceValue $face3 edges]
echo $edges

foreach edge $edges {
  set start [getVertexValue [getEdgeValue $edge start] model]
  set end   [getVertexValue [getEdgeValue $edge end  ] model]

  echo "$edge : $start $end"
}

mergeEdge [lindex $edges 1]

set edges [getFaceValue $face3 edges]
echo $edges

foreach edge $edges {
  set start [getVertexValue [getEdgeValue $edge start] model]
  set end   [getVertexValue [getEdgeValue $edge end  ] model]

  echo "$edge : $start $end"
}

writeObj "house.obj"
