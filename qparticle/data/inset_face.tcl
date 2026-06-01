proc initMaterial { name color } {
  set m [addMaterial $name]
  setMaterialValue $m diffuse $color
  return $m
}

set m1 [initMaterial red   "red"  ]
set m2 [initMaterial green "green"]

set cube [addCube 1]

set faces [getObjectValue $cube faces]
echo "faces: $faces"

set face1 [getObjectValue $cube nearest_face {0 1 0}]]

setFaceValue $face1 inset    0.1
setFaceValue $face1 material $m1

extrudeFaces $face1 0.2

set face2 [getObjectValue $cube nearest_face {0 1.1 0}]]

setFaceValue $face2 inset    0.1
setFaceValue $face2 material $m2

extrudeFaces $face2 0.2

#writeObj "inset_face.obj"
