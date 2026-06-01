set cube1 [addCube 1.0]
set cube2 [addCube 1.0]

setObjectValue $cube2 rotate [list 10 20 30]

set intersect [intersectObjects [list $cube1 $cube2]]

deleteObjects [list $cube1 $cube2]

#writeObj "intersect.obj"
