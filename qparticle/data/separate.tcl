set cube [addCube 1]

echo $cube

if {0} {
set faces [getObjectValue $cube faces]

echo "faces: $faces"

set face1 [lindex $faces 1]

set object1 [separateFace $face1]

setObjectValue $object1 translate {0 1 0}
}

set edges [getObjectValue $cube edges]

echo "edges: $edges"

set edge1 [lindex $edges 1]

set object1 [separateEdge $edge1]

setObjectValue $object1 translate {0 1 0}

set edges1 [getObjectValue $object1 edges]

set edge2 [lindex $edges1 1]

echo "Edge"
puts [getVertexValue [getEdgeValue $edge2 start] model]
puts [getVertexValue [getEdgeValue $edge2 end  ] model]
puts [getEdgeValue $edge2 normal]

set face2 [extrudeEdges $edge2 0.5]

echo "Face"
foreach vertex [getFaceValue $face2 vertices] {
  puts [getVertexValue $vertex model]
}

writeObj "separate.obj"
