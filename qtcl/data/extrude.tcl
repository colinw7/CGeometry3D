set cube [addCube 1]

set faces [getObjectValue $cube faces]

echo "faces: $faces"

set face2 [lindex $faces 1]

extrudeFaces $face2 0.5

set vertices [getFaceValue $face2 vertices]

echo "vertices: $vertices"
