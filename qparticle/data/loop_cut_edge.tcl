set m1 [addMaterial red   ]; setMaterialValue $m1 diffuse red
set m2 [addMaterial green ]; setMaterialValue $m2 diffuse green
set m3 [addMaterial blue  ]; setMaterialValue $m3 diffuse blue
set m4 [addMaterial orange]; setMaterialValue $m4 diffuse orange
set m5 [addMaterial yellow]; setMaterialValue $m5 diffuse yellow
set m6 [addMaterial cyan  ]; setMaterialValue $m6 diffuse cyan

set m7  [addMaterial purple]; setMaterialValue $m7  diffuse purple
set m8  [addMaterial white ]; setMaterialValue $m8  diffuse white
set m9  [addMaterial pink  ]; setMaterialValue $m9  diffuse pink
set m10 [addMaterial black ]; setMaterialValue $m10 diffuse black

set cube [addCube 1]

set face1 [getObjectValue $cube nearest_face { 1  0  0}]]
set face2 [getObjectValue $cube nearest_face {-1  0  0}]]
set face3 [getObjectValue $cube nearest_face { 0  1  0}]]
set face4 [getObjectValue $cube nearest_face { 0 -1  0}]]
set face5 [getObjectValue $cube nearest_face { 0  0  1}]]
set face6 [getObjectValue $cube nearest_face { 0  0 -1}]]

#setFaceValue $face1 material $m1
#setFaceValue $face2 material $m2
#setFaceValue $face3 material $m3
#setFaceValue $face4 material $m4
#setFaceValue $face5 material $m5
#setFaceValue $face6 material $m6

proc printFaceVertices { face } {
  set vertices [getFaceValue $face vertices]
  echo "face: $face, vertices: $vertices"
}

proc printEdgeVertices { edge } {
  set vertices [getEdgeValue $edge vertices]
  echo "edge: $edge, vertices: $vertices"
}

proc printObjectFaces { object } {
  set faces [getObjectValue $cube faces]
  echo "faces: $faces"
}

proc printObjectEdges { object } {
  set edges [getObjectValue $cube edges]
  echo "edges: $edges"
}

proc colorFace { face m } {
  setFaceValue $face material $m
}

proc colorFaces { faces m1 m2 } {
  set faces1 [lindex $faces 0]
  set faces2 [lindex $faces 1]

  foreach face $faces1 {
    setFaceValue $face material $m1
  }

  foreach face $faces2 {
    setFaceValue $face material $m2
  }
}

set edge1 [getObjectValue $cube nearest_edge {0 0.5 0.5}]

setEdgeValue $edge1 loop_cut 3

set edge2 [getObjectValue $cube nearest_edge {0.5 0.5 0}]]

setEdgeValue $edge2 loop_cut 3

set face1 [getObjectValue $cube nearest_face {-0.125 0.5 -0.125}]]
set face2 [getObjectValue $cube nearest_face { 0.125 0.5 -0.125}]]
set face3 [getObjectValue $cube nearest_face {-0.125 0.5  0.125}]]
set face4 [getObjectValue $cube nearest_face { 0.125 0.5  0.125}]]

colorFace $face1 $m1
colorFace $face2 $m2
colorFace $face3 $m3
colorFace $face4 $m4

set cfaces [list $face1 $face2 $face3 $face4]

circularizeFaces $cfaces

scaleFaces $cfaces [list 0 0.5 0] 1.25

foreach face $cfaces {
  extrudeFaces $face 0.1
}
