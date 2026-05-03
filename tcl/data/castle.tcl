set wall   100
set height 40
set thick  4

proc addWall { } {
  set objects {}

  set cube [addCube 1]

  setObjectValue $cube scale     [list $::wall $::height $::thick]
  setObjectValue $cube translate [list [expr {$::wall/2}] [expr {$::height/2}] 0]

  lappend objects $cube

  set nt [expr {int($::wall/(2*$::thick) + 0.5) + 1}]

  set x 0

  while {$x <= $::wall} {
    set cren [addCube 1]

    set x [expr {$x + $::thick/2.0}]
    set y [expr {($::height + $::thick)/2.0}]
    set z 0

    setObjectValue $cren scale     [list $::thick [expr {$::height + $::thick}] $::thick]
    setObjectValue $cren translate [list $x $y $z]

    lappend objects $cren

    set x [expr {$x + $::thick*2}]
  }

  return $objects
}

proc addTower { } {
  set h1 [expr {$::height + $::thick}]
  set w1 [expr {2.0*$::thick}]

  set cylinder [addCylinder $w1 $h1]
  set cone     [addCone     $w1 $w1]

  setObjectValue $cylinder translate [list 0 [expr {$h1/2}] 0]
  setObjectValue $cone     translate [list 0 [expr {$h1 + $w1/2}] 0]

  lappend objects $cylinder
  lappend objects $cone

  return $objects
}

proc addTowerWall { name } {
  set objects [concat [addWall] [addTower]]

  set union [unionObjects $objects]

  deleteObjects $objects

  setObjectValue $union name     $name
  setObjectValue $union material $::mat

  return $union
}

set mat [addMaterial mat]

setMaterialValue $mat diffuse gray

set wall1 [addTowerWall "wall1"]
set wall2 [addTowerWall "wall2"]
set wall3 [addTowerWall "wall3"]
set wall4 [addTowerWall "wall4"]

setObjectValue $wall1 rotate {0  90 0}
setObjectValue $wall2 rotate {0 180 0}
setObjectValue $wall3 rotate {0 270 0}

set dw [expr {$wall + $thick}]

setObjectValue $wall1 translate [list $dw 0 0]
setObjectValue $wall2 translate [list $dw 0 $dw]
setObjectValue $wall3 translate [list 0 0 $dw]

writeObj "castle.obj"
