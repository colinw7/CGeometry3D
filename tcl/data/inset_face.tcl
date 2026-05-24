proc initMaterial { name color } {
  set m [addMaterial $name]
  setMaterialValue $m diffuse $color
  return $m
}

set m1 [initMaterial red "red"]

set cube [addCube 1]

set faces [getObjectValue $cube faces]
echo "faces: $faces"

set face1 [getObjectValue $cube nearest_face {0 0 -1}]]

setFaceValue $face1 inset    0.1
setFaceValue $face1 material $m1

extrudeFace $face1 0.2

writeObj "inset_face.obj"
