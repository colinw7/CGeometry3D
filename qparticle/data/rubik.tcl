proc getSideMaterial { v } {
  set vx [lindex $v 0]
  set vy [lindex $v 1]
  set vz [lindex $v 2]

  set color [expr {$vx + 2*$vy + 3*$vz}]
  if {$color < 0} { 
    set color [expr {$color + 3}]
  } else {
    set color [expr {$color + 2}]
  }
  #echo $color

  if       {$color == 0} { return $::m1
  } elseif {$color == 1} { return $::m2
  } elseif {$color == 2} { return $::m3
  } elseif {$color == 3} { return $::m4
  } elseif {$color == 4} { return $::m5
  } elseif {$color == 5} { return $::m6
  }
  
  return $::m1
}

proc cmpReal { r1 r2 } {
  set d [expr {abs($r1 - $r2)}]
  if {$d < 1E-5} { return 0 }
  if {$r1 < $r2} { return -1 }
  return -1
}

proc cmpVectors { v1 v2 } {
  set v1x [lindex $v1 0]
  set v2x [lindex $v2 0]

  set cx [cmpReal $v1x $v2x]
  if {$cx != 0} { return $cx }

  set v1y [lindex $v1 1]
  set v2y [lindex $v2 1]

  set cy [cmpReal $v1y $v2y]
  if {$cy != 0} { return $cy }

  set v1z [lindex $v1 2]
  set v2z [lindex $v2 2]

  set cz [cmpReal $v1z $v2z]
  return $cz
}

proc setFaceColor { obj v } {
  set material [getSideMaterial $v]

  foreach face [getObjectValue $obj faces] {
    set n [getFaceValue $face normal]

    if {[cmpVectors $n $v] == 0} {
      setFaceValue $face material $material
    }
  }
}

proc createCubelet { cube pos v } {
  set vx [lindex $v 0]
  set vy [lindex $v 1]
  set vz [lindex $v 2]

  set s 0.95

  set cubelet [addCube 1]

  set d 0.5
  set t 0.1

  set xt [expr {[lindex $pos 0] + $d*$vx}]
  set yt [expr {[lindex $pos 1] + $d*$vy}]
  set zt [expr {[lindex $pos 2] + $d*$vz}]

  set xs [expr {(abs($vx)*$t + (1 - abs($vx)))*$s}]
  set ys [expr {(abs($vy)*$t + (1 - abs($vy)))*$s}]
  set zs [expr {(abs($vz)*$t + (1 - abs($vz)))*$s}]

  setObjectValue $cubelet scale     [list $xs $ys $zs]
  setObjectValue $cubelet translate [list $xt $yt $zt]

  set material [getSideMaterial $v]

  setObjectValue $cubelet material $material

  setObjectProperty $cubelet cube      $cube
  setObjectProperty $cubelet direction $v

  return $cubelet
}

proc createRubik { } {
  set cubes {}

  set s 0.99

  for {set iz -1} {$iz <= 1} {incr iz} {
    for {set iy -1} {$iy <= 1} {incr iy} {
      for {set ix -1} {$ix <= 1} {incr ix} {
        if {$ix == 0 && $iy == 0 && $iz == 0} {
          continue
        }

        set cube [addCube 1]

        set pos [list $ix $iy $iz]

        setObjectValue $cube scale     [list $s $s $s]
        setObjectValue $cube translate $pos

        setObjectValue $cube material $::m0

        set cubelets {}

        if {$ix != 0} {
          set xv [list $ix 0 0]

          #setObjectProperty $cube xside $xv
          #setFaceColor $cube $xv

          set cubelet [createCubelet $cube $pos $xv]
          lappend cubelets $cubelet
        } else {
          #setObjectProperty $cube xside {0 0 0}
        }

        if {$iy != 0} {
          set yv [list 0 $iy 0]

          #setObjectProperty $cube yside $yv
          #setFaceColor $cube $yv

          set cubelet [createCubelet $cube $pos $yv]
          lappend cubelets $cubelet
        } else {
          #setObjectProperty $cube yside {0 0 0}
        }

        if {$iz != 0} {
          set zv [list 0 0 $iz]

          #setObjectProperty $cube zside $zv
          #setFaceColor $cube $zv

          set cubelet [createCubelet $cube $pos $zv]
          lappend cubelets $cubelet
        } else {
          #setObjectProperty $cube zside {0 0 0}
        }

        lappend cubes [list $cube $cubelets]
      }
    }
  }

  return $cubes
}

proc turnFaceProc { cube cubelets v a } {
  setObjectValue $cube rotate $v $a

  foreach cubelet $cubelets {
    set d [getObjectProperty $cubelet direction]

    set d1 [calcVector $d rotate $v $a]

    setObjectValue $cubelet rotate $v $a

    setObjectProperty $cubelet direction $d1

    echo "Cubelet $cubelet direction : $d -> $d1"
  } 
}
 
proc turnFace { v clockwise } {
  echo "turnFace $v $clockwise"

  set a [expr {$clockwise*10}]

  set aproc ""

  foreach cube $::cubes {
    set cube1     [lindex $cube 0]
    set cubelets1 [lindex $cube 1]

    set found 0

    foreach cubelet1 $cubelets1 {
      set d [getObjectProperty $cubelet1 direction]

      if {[cmpVectors $d $v] == 0} {
        set found 1
        break
      }
    }

    if {$found} {
      append aproc "
        turnFaceProc $cube1 {$cubelets1} {$v} $a"
    }

    if {0} {
    set xv [getObjectProperty $cube1 xside]
    set yv [getObjectProperty $cube1 yside]
    set zv [getObjectProperty $cube1 zside]
    if {[cmpVectors $v $xv] == 0 ||
        [cmpVectors $v $yv] == 0 ||
        [cmpVectors $v $zv] == 0} {
      set found 1
    }

    if {$found} {
      append aproc "
        setObjectValue $cube1 rotate {$v} $a
      "
    }
    }
  }

  if {$aproc != ""} {
    animateReal [list 0 9 1] $aproc
  }
}

proc keyPress { args } {
  set key   [lindex $args 0]
  set ctrl  [lindex $args 1]
  set shift [lindex $args 2]

  echo "keyPress ($key) ($ctrl) ($shift)"

  set clockwise 1

  if {$shift} {
    set clockwise -1
  }

  if       {$key == "l" || $key == "L"} {
    turnFace [list -1 0 0] $clockwise
  } elseif {$key == "r" || $key == "R"} {
    turnFace [list  1 0 0] $clockwise
  } elseif {$key == "u" || $key == "U"} {
    turnFace [list 0  1 0] $clockwise
  } elseif {$key == "d" || $key == "D"} {
    turnFace [list 0 -1 0] $clockwise
  } elseif {$key == "f" || $key == "F"} {
    turnFace [list 0 0  1] $clockwise
  } elseif {$key == "b" || $key == "B"} {
    turnFace [list 0 0 -1] $clockwise
  }
}

set m0 [addMaterial black ]; setMaterialValue $m0 diffuse black ; #puts "Material: $m0"
set m1 [addMaterial red   ]; setMaterialValue $m1 diffuse red   ; #puts "Material: $m1"
set m2 [addMaterial green ]; setMaterialValue $m2 diffuse green ; #puts "Material: $m2"
set m3 [addMaterial blue  ]; setMaterialValue $m3 diffuse blue  ; #puts "Material: $m3"
set m4 [addMaterial yellow]; setMaterialValue $m4 diffuse yellow; #puts "Material: $m4"
set m5 [addMaterial orange]; setMaterialValue $m5 diffuse orange; #puts "Material: $m5"
set m6 [addMaterial white ]; setMaterialValue $m6 diffuse white ; #puts "Material: $m6"

set cubes [createRubik]
#echo $cubes

#writeObj "rubik.obj"
