set cube [addCube]

set selection_obj { }

proc selectionProc { } {
  if {$::selection_obj != ""} {
    deleteObjects $::selection_obj
  }

  set faces [getObjectValue $::cube selected_faces]

  set ::selection_obj [addObject]

  foreach face $faces {
    set edges [getFaceValue $face edges]

    set center [getFaceValue $face center]

    foreach edge $edges {
      set vector [getFaceValue $face edge_vector $edge]

      set p [calcVector $center add $vector]

      set v1 [addVertex $::selection_obj $center]
      set v2 [addVertex $::selection_obj $p]

      set l [addLine $::selection_obj [list $v1 $v2]]

      setLineValue $l color red
    }
  }
}
