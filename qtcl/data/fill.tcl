proc selectionProc { } {
  set vertices [getObjectValue $::cube selected_vertices]

  foreach vertex $vertices {
    set p [getVertexValue $vertex model]

    echo $p
  }
}

set cube [addCube 1]

set faces [getObjectValue $cube faces]
foreach face $faces {
  set name [getFaceValue $face name]
  echo "Face $face : $name"
}

set edges [getObjectValue $cube edges]
echo $edges

set face1 [getObjectValue $cube named_face top]
set face2 [getObjectValue $cube named_face front]
set face3 [getObjectValue $cube named_face right]
echo "$face1 $face2 $face3"

#set edge [getFaceValue $face nearest_edge [list -0.5 0.5 0.0]]
#echo $edge
#setEdgeValue $edge selected 1

#setFaceValue $face1 selected 1
#setFaceValue $face2 selected 1
#setFaceValue $face3 selected 1

set vertex [getFaceValue $face1 nearest_vertex [list 0.5 0.5 0.5]]
deleteVertices $vertex

set vertex1 [getObjectValue $cube nearest_vertex [list -0.5  0.5  0.5]]
set vertex2 [getObjectValue $cube nearest_vertex [list  0.5  0.5 -0.5]]
set vertex3 [getObjectValue $cube nearest_vertex [list  0.5 -0.5  0.5]]
echo "$vertex1 $vertex2 $vertex3"

setVertexValue $vertex1 selected 1
setVertexValue $vertex2 selected 1
setVertexValue $vertex3 selected 1

set faces [getObjectValue $cube faces]
foreach face $faces {
  set name [getFaceValue $face name]
  echo "Face $face : $name"
}

set edges [getObjectValue $cube edges]
echo $edges

fillVertices [list $vertex1 $vertex2 $vertex3]
