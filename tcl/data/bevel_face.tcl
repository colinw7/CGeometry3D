set cube [addCube 1]

set faces [getObjectValue $cube faces]
echo "faces: $faces"

set face1 [getObjectValue $cube nearest_face {0 0 -1}]]

setFaceValue $face1 bevel 0.1

writeObj "bevel_face.obj"
