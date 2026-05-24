set cyl [addCylinder 0.5 2.0]

set inverse [inverseObject $cyl]

#deleteObjects $cyl

writeObj "inverse.obj"
