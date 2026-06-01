set cube [addCube 1]

set faces [getObjectValue $cube faces]

echo "faces: $faces"

set face1 [getObjectValue $cube nearest_face {-2 0 0}]
set face2 [getObjectValue $cube nearest_face { 2 0 0}]
set face3 [getObjectValue $cube nearest_face {0 -2 0}]
set face4 [getObjectValue $cube nearest_face {0  2 0}]
set face5 [getObjectValue $cube nearest_face {0 0 -2}]
set face6 [getObjectValue $cube nearest_face {0 0  2}]

echo "$face1 $face2 $face3 $face4 $face5 $face6"
