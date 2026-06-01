set cube [addCube 1]

set edges [getObjectValue $cube edges]
echo "edges: $edges"

set edge1 [getObjectValue $cube nearest_edge {-1 -1 0}]]

setEdgeValue $edge1 bevel 0.1

set edges [getObjectValue $cube edges]
echo "edges: $edges"

#set edge2 [getObjectValue $cube nearest_edge {1 -1 0}]]
#setEdgeValue $edge2 bevel 0.1

#writeObj "bevel_edge.obj"
