proc printFaceVertices { face } {
  set res ""
  set vertices [getFaceValue $face vertices]
  foreach vertex $vertices {
    set p [getVertexValue $vertex model]
    append res " {$p}"
  }
  echo $res
}

set m1 [addMaterial red ]; setMaterialValue $m1 diffuse red
set m2 [addMaterial blue]; setMaterialValue $m2 diffuse blue

set cube [addCube 1]

set face1 [getObjectValue $cube nearest_face {0  1 0}]]
set face2 [getObjectValue $cube nearest_face {0 -1 0}]]

printFaceVertices $face1
printFaceVertices $face2

setFaceValue $face1 material $m1
setFaceValue $face2 material $m2

