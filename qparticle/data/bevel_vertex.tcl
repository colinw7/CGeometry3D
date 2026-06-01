set cube [addCube 1]

set vertices [getObjectValue $cube vertices]
echo "vertices: $vertices"

#set v1 [getObjectValue $cube nearest_vertex {0.5 0.5 0.5}]]
#setVertexValue $v1 bevel 0.1

foreach v $vertices {
  setVertexValue $v bevel 0.1
}

#source "data/showNormals.tcl"
#showNormals
