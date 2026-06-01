set cube [addCube 1]

set faces [getObjectValue $cube faces]

echo "faces: $faces"

set face [getObjectValue $cube nearest_face { 2 0 0}]

setFaceValue $face scale {2 1 2}

writeObj "scale.obj"
