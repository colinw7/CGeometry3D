set cube [addCube]

set selection_obj { }

proc selectionProc { } {
  if {$::selection_obj != ""} {
    deleteObjects $::selection_obj
  }

  set faces [getObjectValue $::cube selected_faces]

  set ::selection_obj [addObject]

  foreach face $faces {
    set center [getFaceValue $face center]
    set normal [getFaceValue $face normal]

    set p [calcVector $center add $normal]

    set v1 [addVertex $::selection_obj $center]
    set v2 [addVertex $::selection_obj $p]

    set l [addLine $::selection_obj [list $v1 $v2]]

    setLineValue $l color white
  }
}
