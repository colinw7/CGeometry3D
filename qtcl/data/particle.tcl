for {set i 0} {$i < 100} {incr i} {
  set p($i) [addParticle [list 0 0 0]]
}

proc printParticle { p } {
  echo "$p"
  echo [getParticleValue $p position]
  echo [getParticleValue $p velocity]
  echo [getParticleValue $p mass]
  echo [getParticleValue $p force]
  echo [getParticleValue $p age]
}

if {0} {
for {set i 0} {$i < 100} {incr i} {
  printParticle $p($i)
}
}

for {set i 0} {$i < 100} {incr i} {
  set dx [expr {2*(rand() - 0.5)}]
  set dy [expr {2*(rand() - 0.5)}]

  setParticleValue $p($i) velocity [list $dx $dy 0.0]

  set r [expr {rand()}]
  set g [expr {rand()}]
  set b [expr {rand()}]

  setParticleValue $p($i) color [list $r $g $b]
}
