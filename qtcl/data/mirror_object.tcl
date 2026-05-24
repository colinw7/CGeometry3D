set cube [addCube]

setObjectValue $cube translate [list 1 1 1]

set objects [mirrorObject $cube xyz]

foreach object $objects {
  echo "$object"
}

#writeObj "mirror_object.obj"
