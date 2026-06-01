set cube [addCube]

set edge1 [getObjectValue $cube nearest_edge {-1 1 0}]]

setEdgeValue $edge1 scale 0.5

writeObj "scale_edge.obj"
