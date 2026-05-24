set cube [addCube     1.0]
set cyl  [addCylinder 0.5 2.0]

set subtract [subtractObjects [list $cube $cyl]]

deleteObjects [list $cube $cyl]

#writeObj "subtract.obj"
