set invader_w 16
set invader_h 8

set dw [expr {$invader_w + 1}]
set dh [expr {$invader_h + 2}]

setAppValue camera_distance 128

set invader1 [readModel "data/invader1.obj"]
set invader2 [readModel "data/invader2.obj"]
set invader3 [readModel "data/invader3.obj"]
set invader4 [readModel "data/invader4.obj"]

setObjectValue $invader1 visible 0
setObjectValue $invader2 visible 0
setObjectValue $invader3 visible 0
setObjectValue $invader4 visible 0

set invader_nx 5
set invader_ny 11
set invader_dx 1

proc addInvader { ix iy } {
  set ind [expr {$iy*$::invader_ny + $ix}]

  if       {$iy == 0} {
    set ::invader($ind) [getObjectValue $::invader1 ref_object]
  } elseif {$iy == 1} {
    set ::invader($ind) [getObjectValue $::invader2 ref_object]
  } elseif {$iy == 2} {
    set ::invader($ind) [getObjectValue $::invader3 ref_object]
  } elseif {$iy == 3 || $iy == 4} {
    set ::invader($ind) [getObjectValue $::invader4 ref_object]
  }

  set ::invader_x($ind) [expr {($ix - $::invader_nx)*$::dw}]
  set ::invader_y($ind) [expr {$iy*$::dh + 32}]

  setObjectValue $::invader($ind) visible 1
  setObjectValue $::invader($ind) translate [list $::invader_x($ind) $::invader_y($ind) 0] set
}

proc moveInvader { ix iy } {
  set ind [expr {$iy*$::invader_ny + $ix}]

  set ::invader_x($ind) [expr {$::invader_x($ind) + $::invader_dx}]

  setObjectValue $::invader($ind) translate [list $::invader_x($ind) $::invader_y($ind) 0] set
}

proc updateInvaderDx { } {
  set flip 0

  for {set iy 0} {$iy < $::invader_nx} {incr iy} {
    for {set ix 0} {$ix < $::invader_ny} {incr ix} {
      set ind [expr {$iy*$::invader_ny + $ix}]

      if {$::invader_dx > 0} {
        if {$::invader_x($ind) > 128} {
          set flip 1
          break
        }
      } else {
        if {$::invader_x($ind) < -128} { 
          set flip 1
          break
        }
      }
    }
  }
  
  if {$flip} {
    set ::invader_dx [expr {-$::invader_dx}]

    for {set iy 0} {$iy < $::invader_nx} {incr iy} {
      for {set ix 0} {$ix < $::invader_ny} {incr ix} {
        set ind [expr {$iy*$::invader_ny + $ix}]

        set ::invader_y($ind) [expr {$::invader_y($ind) - 4}]

        setObjectValue $::invader($ind) translate [list $::invader_x($ind) $::invader_y($ind) 0] set
      }
    }
  }
}

for {set iy 0} {$iy < $::invader_nx} {incr iy} {
  for {set ix 0} {$ix < $::invader_ny} {incr ix} {
    addInvader $ix $iy
  }
} 

set base [readModel "data/invader_base.obj"]

setObjectValue $base visible 0

proc addBase { ix } {
  set base1 [getObjectValue $::base ref_object]

  set x [expr {($ix - 2)*48 + 24}]
  set y -64

  setObjectValue $base1 visible 1
  setObjectValue $base1 translate [list $x $y 0] set
}

for {set ix 0} {$ix < 4} {incr ix} {
  addBase $ix
}

set ship [readModel "data/invader_ship.obj"]

set ship_x 0
set ship_y -72

setObjectValue $ship translate [list $ship_x $ship_y 0] set

proc hitInvaders { pos } {
  set px [lindex $pos 0]
  set py [lindex $pos 1]

  for {set iy 0} {$iy < $::invader_nx} {incr iy} {
    for {set ix 0} {$ix < $::invader_ny} {incr ix} {
      set ind [expr {$iy*$::invader_ny + $ix}]

      set visible [getObjectValue $::invader($ind) visible]
      if {! $visible} { continue }

      set x1 [expr {$::invader_x($ind) - $::invader_w/2}]
      set y1 [expr {$::invader_y($ind) - $::invader_h/2}]
      set x2 [expr {$x1 + $::invader_w}]
      set y2 [expr {$y1 + $::invader_h}]

      if {$px >= $x1 && $px < $x2 && $py >= $y1 && $py < $y2} {
        return $::invader($ind)
      }
    }
  }

  return ""
}

proc tickProc { args } {
  foreach particle [getAppValue particles] {
    set dead [getParticleValue $particle dead]
    if {$dead} { continue }

    set meta [getParticleValue $particle meta]

    if {$meta == "ship"} {
      set pos [getParticleValue $particle position]
      
      set invader [hitInvaders $pos]

      if {$invader != ""} {
        echo "Hit: $invader"

        setParticleValue $particle dead 1
        setObjectValue   $invader  visible 0
      }
    }
  }

  for {set iy 0} {$iy < $::invader_nx} {incr iy} {
    for {set ix 0} {$ix < $::invader_ny} {incr ix} {
      moveInvader $ix $iy
    }
  }

  updateInvaderDx
}

proc shipShoot { } {
  set p [addParticle [list $::ship_x $::ship_y 0]]

  setParticleValue $p velocity [list 0 250 0]
  setParticleValue $p meta     ship
}

proc keyPress { args } {
  set key   [lindex $args 0]
  set ctrl  [lindex $args 1]
  set shift [lindex $args 2]
  
  echo "keyPress ($key) ($ctrl) ($shift)"

  if       {$key == "left"} {
    set ::ship_x [expr {$::ship_x - 1}]

    setObjectValue $::ship translate [list $::ship_x $::ship_y 0] set
  } elseif {$key == "right"} {
    set ::ship_x [expr {$::ship_x + 1}]

    setObjectValue $::ship translate [list $::ship_x $::ship_y 0] set
  } elseif {$key == "space"} {
    shipShoot
  }
}
