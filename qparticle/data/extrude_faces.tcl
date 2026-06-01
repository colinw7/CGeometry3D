set m1 [addMaterial red   ]; setMaterialValue $m1 diffuse red
set m2 [addMaterial green ]; setMaterialValue $m2 diffuse green
set m3 [addMaterial blue  ]; setMaterialValue $m3 diffuse blue
set m4 [addMaterial yellow]; setMaterialValue $m4 diffuse yellow

proc colorFaces { faces m } {
  foreach face $faces {
    setFaceValue $face material $m
  }
}

set plane [addPlane 2 2]

set faces [getObjectValue $plane faces]
set face [lindex $faces 0]

set efaces1 [extrudeFaces $face 0.5]
echo $efaces1

set face1 [lindex $efaces1 0]

colorFaces $efaces1 $m1

set efaces2 [extrudeFaces $face1 0.5]
echo $efaces2
set face2 [lindex $efaces2 0]

colorFaces $efaces2 $m2

set efaces3 [extrudeFaces $face2 0.5]
echo $efaces3
set face3 [lindex $efaces3 0]

colorFaces $efaces3 $m3

set efaces {}

foreach face $efaces2 {
  if {$face != $face2} {
    lappend efaces $face
  }
}

extrudeFaces $efaces 0.5

proc showNormals { } {
  set selection_obj [addObject]

  foreach object [getAppValue objects] {
    set faces [getObjectValue $object faces]

    foreach face $faces {
      set center [getFaceValue $face center]
      set normal [getFaceValue $face normal]
echo "$face : $center $normal"

      set p [calcVector $center add $normal]

      set v1 [addVertex $selection_obj $center]
      set v2 [addVertex $selection_obj $p]

      addLine $selection_obj [list $v1 $v2]
    }
  }
}

#showNormals
