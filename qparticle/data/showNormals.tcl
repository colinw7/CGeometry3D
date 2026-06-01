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
