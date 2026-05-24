set plane [addPlane]

set faces [getObjectValue $plane faces]

echo "faces: $faces"

set face [lindex $faces 0]

set a  15
set dy 0.2

for {set i 0} {$i < 12} {incr i} {
  set face1 [extrudeFaces $face $dy]

  setFaceValue $face1 rotate [list 0 $a 0]

  set face $face1
}

writeObj "rotate.obj"
