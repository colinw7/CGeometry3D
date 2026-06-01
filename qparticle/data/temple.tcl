set plane [addPlane 2 2]

set faces [getObjectValue $plane faces]
set face1 [lindex $faces 0]

extrudeFaces $face1 0.5

set face1 [getObjectValue $plane nearest_face [list 0 0.5 0]]
#echo $face1

setFaceValue $face1 center_scale [list 0 0.5 0] 0.8

set face2 [duplicateFace $face1]

setFaceValue $face2 center_scale [list 0 0.5 0] 0.8

extrudeFaces $face2 0.4

set face1 [getObjectValue $plane nearest_face [list 0 0.9 0]]
#echo $face1

setFaceValue $face1 center_scale [list 0 0.5 0] 0.8

set face2 [duplicateFace $face1]

setFaceValue $face2 center_scale [list 0 0.5 0] 0.8

extrudeFaces $face2 0.4

set face1 [getObjectValue $plane nearest_face [list 0 1.3 0]]
#echo $face1

set efaces [extrudeFaces $face1 0.1]
echo $efaces
set etface [lindex $efaces 0]

set face1 [getObjectValue $plane nearest_face [list 0 1.4 0]]
#echo $face1

extrudeFaces $face1 0.1

set efaces1 {}

foreach eface $efaces {
  if {$eface != $etface} {
    lappend efaces1 $eface
  }
}
extrudeFaces $efaces1 0.1

proc showNormals { } {
  set selection_obj [addObject]

  foreach object [getAppValue objects] {
    set faces [getObjectValue $object faces]

    foreach face $faces {
      set center [getFaceValue $face center]
      set normal [getFaceValue $face normal]

      set p [calcVector $center add $normal]

      set v1 [addVertex $selection_obj $center]
      set v2 [addVertex $selection_obj $p]

      addLine $selection_obj [list $v1 $v2]
    }
  }
}

#showNormals
