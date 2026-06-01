set m1 [addMaterial blue]; setMaterialValue $m1 diffuse blue

set cube [addCube 1]

setObjectValue $cube material $m1

set faces [getObjectValue $cube faces]

set face [lindex $faces 1]

setFaceValue $face selected 1
