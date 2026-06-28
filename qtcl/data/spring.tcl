proc mousePress { args } {
  addParticleProc
}

proc printParticle { p } {
  echo "$p"
  echo [getParticleValue $p position]
  echo [getParticleValue $p velocity]
  echo [getParticleValue $p mass]
  echo [getParticleValue $p force]
  echo [getParticleValue $p age]
}

setAppValue gravity 0.0

set p [addParticle [list 0 1 0]]

proc addParticleProc { } {
  set particles [getAppValue particles]

  set x [expr {rand()}]
  set y [expr {rand()}]

  set p1 [addParticle [list $x $y 0]]

  set np [llength $particles]

  set p2 [lindex $particles [expr {$np - 1}]]

  set s [addSpring $p1 $p2]
  setSpringValue $s rest_length 0.2

  foreach p3 $particles {
    set a [addAttraction $p1 $p3]

    setAttractionValue $a strength -0.1
    setAttractionValue $a min_distance 0.04
  }
}
