set cube [addCube 1]

set face [getObjectValue $cube nearest_face {0.0 0.5 0.5}]]

#setFaceValue $face subdivide 1
#setFaceValue $face subdivide 2
setFaceValue $face subdivide 3
#setFaceValue $face subdivide 4

#source "data/showNormals.tcl"
#showNormals
