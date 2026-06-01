set tex(A) [addTexture "data/letter_a.png"]
set tex(B) [addTexture "data/letter_b.png"]
set tex(C) [addTexture "data/letter_c.png"]
set tex(D) [addTexture "data/letter_d.png"]
set tex(E) [addTexture "data/letter_e.png"]
set tex(F) [addTexture "data/letter_f.png"]
set tex(G) [addTexture "data/letter_g.png"]
set tex(H) [addTexture "data/letter_h.png"]
set tex(I) [addTexture "data/letter_i.png"]
set tex(J) [addTexture "data/letter_a.png"]
set tex(K) [addTexture "data/letter_b.png"]
set tex(L) [addTexture "data/letter_c.png"]
set tex(M) [addTexture "data/letter_d.png"]
set tex(N) [addTexture "data/letter_e.png"]
set tex(O) [addTexture "data/letter_f.png"]
set tex(P) [addTexture "data/letter_g.png"]
set tex(Q) [addTexture "data/letter_h.png"]
set tex(R) [addTexture "data/letter_i.png"]
set tex(S) [addTexture "data/letter_a.png"]
set tex(T) [addTexture "data/letter_b.png"]
set tex(U) [addTexture "data/letter_c.png"]
set tex(V) [addTexture "data/letter_d.png"]
set tex(W) [addTexture "data/letter_e.png"]
set tex(X) [addTexture "data/letter_f.png"]
set tex(Y) [addTexture "data/letter_g.png"]
set tex(Z) [addTexture "data/letter_h.png"]

setAppValue gravity 0
setAppValue drag    0
setAppValue bgColor [list 0.5 0.5 0.5]

proc randIn { min max } {
  return [expr {rand()*($max - $min) + $min}]
}

proc randIntIn { min max } {
  set i [expr {int(rand()*($max - $min) + $min + 0.5)}]
  if {$i < $min} { set i $min }
  if {$i > $max} { set i $max }
  return $i
}

proc addLetter { c } {
  #echo "addLetter $c"

  set x [randIn -1 1]

  set p [addParticle [list $x 0 0]]

  set r [randIn 0.3 0.8]
  set g [randIn 0.3 0.8]
  set b [randIn 0.3 0.8]

  setParticleValue $p color   [list $r $g $b]
  setParticleValue $p alpha   1
  setParticleValue $p texture $::tex($c)
  setParticleValue $p size    0.2

  set xv [randIn -2.0  2.0]
  set yv [randIn -2.0 -0.5]

  setParticleValue $p velocity [list $xv $yv 0]

  set ::attraction($p) [addAttraction $p null]

  setAttractionValue $::attraction($p) strength     -10
  setAttractionValue $::attraction($p) min_distance 0.01
}

set numLetters 40

set letters "ABCDEFGHIJKLMNOPQRSTUVWXYZ"

for {set i 0} {$i < $numLetters} {incr i} {
  set ic [randIntIn 0 25]

  set c [string range $letters $ic $ic]

  addLetter $c
}

for {set i 0} {$i < 300} {incr i} {
  tickProc
}

proc pointDistance { p1 p2 } {
  set dx [expr {[lindex $p1 0] - [lindex $p2 0]}]
  set dy [expr {[lindex $p1 1] - [lindex $p2 1]}]
  set dz [expr {[lindex $p1 2] - [lindex $p2 2]}]

  return [expr {sqrt($dx*$dx + $dy*$dy + $dz*$dz)}]
}

proc tickProc { args } {
  set particles [getAppValue particles]

  foreach particle $particles {
    set pos [getParticleValue $particle position]
    set vel [getParticleValue $particle velocity]

    set x [lindex $pos 0]
    set y [lindex $pos 1]

    set xv [lindex $vel 0]
    set yv [lindex $vel 1]

    if {($x < -1 && $xv < 0) || ($x > 1 && $xv > 0)} {
      set xv [expr {-$xv}]
    }

    if {$yv < -2.0} { set yv -2.0} elseif {$yv > -0.5} { set yv -0.5 }
    if {$xv < -2.0} { set xv -2.0} elseif {$xv >  2.0} { set xv  2.0 }

    setParticleValue $particle velocity [list $xv $yv 0]

    if {$y < -2} {
      setParticleValue $particle position [list $x 2 0]
    }

    set minParticle {}
    set minDist     0

    foreach particle1 $particles {
      if {$particle1 == $particle} {
        continue
      }

      set pos1 [getParticleValue $particle1 position]

      set dist [pointDistance $pos $pos1]

      if {$minParticle == "" || $dist < $minDist} {
        set minParticle $particle1
        set minDist     $dist
      }
    }

    if {$minParticle != ""} {
      setAttractionValue $::attraction($particle)    target $minParticle
      setAttractionValue $::attraction($minParticle) target $particle
    }
  }
}
