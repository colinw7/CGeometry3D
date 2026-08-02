# TODO:
#  . invader shoot
#  . alien animation
#  . base destroy

set invader_w 16
set invader_h 8

set dw [expr {$invader_w + 1}]
set dh [expr {$invader_h + 2}]

setAppValue bg_color        "#444444"
setAppValue camera_distance 128
setAppValue edit_type       "tcl"

proc randIn { min max } {
  return [expr {rand()*($max - $min) + $min}]
}

proc loadInvader { filename name } {
  set obj [readModel $filename]

  setObjectValue $obj name $name
  setObjectValue $obj visible 0
}

set invader1 [loadInvader "data/invaders/invader1.obj" "invader1"]
set invader2 [loadInvader "data/invaders/invader2.obj" "invader2"]
set invader3 [loadInvader "data/invaders/invader3.obj" "invader3"]
set invader4 [loadInvader "data/invaders/invader4.obj" "invader4"]

set invader_bullet_obj [loadInvader "data/invaders/invader_bullet.obj" "invader_bullet"]

set invader_nx 5
set invader_ny 11
set invader_dx 1

set invader_shoot_vel [vector 0 -250 0]

#set invader_edge 128
set invader_edge 100

proc addOverlayText { str pos align } {
  set text [addText $str $pos 0.1]

  setTextValue $text overlay   1
  setTextValue $text billboard 1
  setTextValue $text color     white
  setTextValue $text halign    $align

  return $text
}

set lives 3
set score 0
set level 1

set lives_text [addOverlayText "Lives: $lives" {-0.9 0.9 0} left  ]
set score_text [addOverlayText "Score: $score" { 0.0 0.9 0} center]
set level_text [addOverlayText "Level: $level" { 0.9 0.9 0} right ]

set game_over_text [addOverlayText "GAME OVER" { 0 0 0} center ]
setTextValue $game_over_text visible 0

proc addInvader { ix iy } {
  set ind [expr {$iy*$::invader_ny + $ix}]

  set meta ""

  if       {$iy == 0 || $iy == 1} {
    set ::invader($ind) [getObjectValue $::invader3 ref_object]
    set meta invader3
  } elseif {$iy == 2 || $iy == 3} {
    set ::invader($ind) [getObjectValue $::invader1 ref_object]
    set meta invader1
  } elseif {$iy == 4} {
    set ::invader($ind) [getObjectValue $::invader4 ref_object]
    set meta invader4
  }

  set ::invader_x($ind) [expr {($ix - $::invader_nx)*$::dw}]
  set ::invader_y($ind) [expr {$iy*$::dh + 32}]

  setObjectValue $::invader($ind) name "invader.${ind}"
  setObjectValue $::invader($ind) meta $meta
  setObjectValue $::invader($ind) visible 1
  setObjectValue $::invader($ind) translate [list $::invader_x($ind) $::invader_y($ind) 0] set
}

proc invaderScore { invader } {
  set meta [getObjectValue $invader meta]

  if {$meta == "invader3"} { return 100 }
  if {$meta == "invader1"} { return 200 }
  if {$meta == "invader4"} { return 300 }

  return 0
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
        if {$::invader_x($ind) > $::invader_edge} {
          set flip 1
          break
        }
      } else {
        if {$::invader_x($ind) < -$::invader_edge} {
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

proc loadBase { filename name } {
  set obj [readModel $filename]

  setObjectValue $obj name $name
  setObjectValue $obj visible 0
}

set base0_obj [loadBase "data/invaders/invader_base.obj"   "base0"]
set base1_obj [loadBase "data/invaders/invader_base_1.obj" "base1"]
set base2_obj [loadBase "data/invaders/invader_base_2.obj" "base2"]
set base3_obj [loadBase "data/invaders/invader_base_3.obj" "base3"]
set base4_obj [loadBase "data/invaders/invader_base_4.obj" "base4"]

set num_bases 4
set base_dx   48
set base_y    -64
set base_w    32
set base_h    32

set base_max_damage 4

proc addBase { ix } {
  set ::base_damage($ix) 0

  set ::base($ix,0) [getObjectValue $::base0_obj ref_object]
  set ::base($ix,1) [getObjectValue $::base1_obj ref_object]
  set ::base($ix,2) [getObjectValue $::base2_obj ref_object]
  set ::base($ix,3) [getObjectValue $::base3_obj ref_object]
  set ::base($ix,4) [getObjectValue $::base4_obj ref_object]

  set ::base_x($ix) [expr {($ix - $::num_bases/2)*$::base_dx + $::base_dx/2}]

  setObjectValue $::base($ix,$::base_damage($ix)) visible 1
  setObjectValue $::base($ix,$::base_damage($ix)) translate [list $::base_x($ix) $::base_y 0] set
}

for {set ix 0} {$ix < $num_bases} {incr ix} {
  addBase $ix
}

set ship_obj [readModel "data/invader_ship.obj"]

set ship_x 0
set ship_y -72
set ship_w 16
set ship_h 16

set ship_shoot_vel [vector 0 250 0]

setObjectValue $ship_obj translate [list $ship_x $ship_y 0] set

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

proc hitBase { pos } {
  set px [lindex $pos 0]
  set py [lindex $pos 1]

  for {set ix 0} {$ix < $::num_bases} {incr ix} {
    if {$::base_damage($ix) > $::base_max_damage} { continue }

    set x1 [expr {$::base_x($ix) - $::base_w/2}]
    set y1 [expr {$::base_y      - $::base_h/2}]
    set x2 [expr {$x1 + $::base_w}]
    set y2 [expr {$y1 + $::base_h}]

    if {$px >= $x1 && $px < $x2 && $py >= $y1 && $py < $y2} {
      return $ix
    }
  }

  return ""
}

proc damageBase { ix } {
  setObjectValue $::base($ix,$::base_damage($ix)) visible 0
    
  incr ::base_damage($ix)

  if {$::base_damage($ix) <= $::base_max_damage} {
    setObjectValue $::base($ix,$::base_damage($ix)) visible 1
    setObjectValue $::base($ix,$::base_damage($ix)) \
      translate [list $::base_x($ix) $::base_y 0] set
  }
}

proc hitShip { pos } {
  set px [lindex $pos 0]
  set py [lindex $pos 1]

  set x1 [expr {$::ship_x - $::ship_w/2}]
  set y1 [expr {$::ship_y - $::ship_h/2}]
  set x2 [expr {$x1 + $::ship_w}]
  set y2 [expr {$y1 + $::ship_h}]

  if {$px >= $x1 && $px < $x2 && $py >= $y1 && $py < $y2} {
    return 1
  }

  return 0
}

proc invaderShoot { ind } {
  set x [expr {$::invader_x($ind)}]
  set y [expr {$::invader_y($ind) - $::invader_h/2}]

  if {! [info exists ::invader_bullet($ind)]} {
    set obj [getObjectValue $::invader_bullet_obj ref_object]

    setObjectValue $obj name    "invader_bullet.$ind"
    setObjectValue $obj visible 1

    set ::invader_bullet($ind) [addParticle [list $x $y 0] 1 $obj]

    setParticleValue $::invader_bullet($ind) velocity $::invader_shoot_vel
    setParticleValue $::invader_bullet($ind) meta     "invader $ind"
  } else {
    set dead [getParticleValue $::invader_bullet($ind) dead]

    if {$dead} {
      setParticleValue $::invader_bullet($ind) position [vector $x $y 0]
      setParticleValue $::invader_bullet($ind) dead 0
    }
  }
}

set ship_num_bullets 4

for {set i 0} {$i < $ship_num_bullets} {incr i} {
  set ship_bullet($i) ""
}

proc shipShoot { } {
  set bullet_num ""

  for {set i 0} {$i < $::ship_num_bullets} {incr i} {
    if {$::ship_bullet($i) != ""} {
      set dead [getParticleValue $::ship_bullet($i) dead]

      if {$dead} {
        set bullet_num $i
        break
      }
    } else {
      set bullet_num $i
    }
  }

  if {$bullet_num != ""} {
    if {$::ship_bullet($bullet_num) == ""} {
      set ::ship_bullet($bullet_num) [addParticle [list $::ship_x $::ship_y 0]]

      setParticleValue $::ship_bullet($bullet_num) meta ship
      setParticleValue $::ship_bullet($bullet_num) velocity $::ship_shoot_vel
    }

    setParticleValue $::ship_bullet($bullet_num) dead     0
    setParticleValue $::ship_bullet($bullet_num) position [list $::ship_x $::ship_y 0]
  }
}

proc tickProc { args } {
  foreach particle [getAppValue particles] {
    set dead [getParticleValue $particle dead]
    if {$dead} { continue }

    set meta [getParticleValue $particle meta]

    if {$meta == "ship"} {
      set pos [getParticleValue $particle position]

      set ix [hitBase $pos]
  
      if {$ix != ""} {
        echo "Hit Base: $ix"

        damageBase $ix

        setParticleValue $particle dead 1

        continue
      }

      set invader [hitInvaders $pos]

      if {$invader != ""} {
        echo "Hit Invader: $invader"

        set ::score [expr {$::score + [invaderScore $invader]}]

        setTextValue $::score_text text "Score: $::score"

        setParticleValue $particle dead 1
        setObjectValue   $invader  visible 0

        continue
      }

      set y [lindex $pos 1]

      if {$y > 80} {
        setParticleValue $particle dead 1

        continue
      }
    } else {
      set pos [getParticleValue $particle position]

      if {[hitShip $pos]} {
        setParticleValue $particle dead 1

        if {$::lives >= 1} {
          set ::lives [expr {$::lives - 1}]
        }

        if {$::lives == 0} {
          setObjectValue $::ship_obj visible 0

          setTextValue $::game_over_text visible 1
          setAppValue  running 0
        }

        setTextValue $::lives_text text "Lives: $::lives"

        continue
      } else {
        set ix [hitBase $pos]

        if {$ix != ""} {
          echo "Hit Base: $ix"

          damageBase $ix

          setParticleValue $particle dead 1

          continue
        }
      }

      set y [lindex $pos 1]

      if {$y < -80} {
        setParticleValue $particle dead 1

        continue
      }
    }
  }

  set ticks [getAppValue ticks]

  for {set iy 0} {$iy < $::invader_nx} {incr iy} {
    for {set ix 0} {$ix < $::invader_ny} {incr ix} {
      moveInvader $ix $iy

      if {[randIn 0 1] < 0.001} {
        set ind [expr {$iy*$::invader_ny + $ix}]

        invaderShoot $ind
      }

      if {($ticks % 10) == 0} {
        set ind [expr {$iy*$::invader_ny + $ix}]

        set meta [getObjectValue $::invader($ind) meta]

        if {$meta == "invader1"} {
          set arm1 [getObjectValue $::invader($ind) child "Arm1"]
          set arm2 [getObjectValue $::invader($ind) child "Arm2"]
          set leg1 [getObjectValue $::invader($ind) child "Leg1"]
          set leg2 [getObjectValue $::invader($ind) child "Leg2"]

          setObjectValue $arm1 rotate [list 1 0 0] 90
          setObjectValue $arm2 rotate [list 1 0 0] 90
          setObjectValue $leg1 rotate_at [list 0 1 0] [list -3.5 0 0] 180
          setObjectValue $leg2 rotate_at [list 0 1 0] [list  3.5 0 0] 180
        }
      }
    }
  }

  updateInvaderDx
}


proc keyPress { args } {
  set key   [lindex $args 0]
  set ctrl  [lindex $args 1]
  set shift [lindex $args 2]

  # echo "keyPress ($key) ($ctrl) ($shift)"

  if       {$key == "left"} {
    set ::ship_x [expr {$::ship_x - 1}]

    setObjectValue $::ship_obj translate [list $::ship_x $::ship_y 0] set
  } elseif {$key == "right"} {
    set ::ship_x [expr {$::ship_x + 1}]

    setObjectValue $::ship_obj translate [list $::ship_x $::ship_y 0] set
  } elseif {$key == "space"} {
    shipShoot
  }
}
