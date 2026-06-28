# TODO
#  . humans
#  . alien spawn
#  . scanner
#  . alien nearest human
#  . mutant
#  . 9 screens wide
#  . bullet
#  . bullet shader (extra lines)

# ---

namespace eval tcl::mathfunc {
  proc min { args } {
    if {[lindex $args 0] < [lindex $args 1]} {
      return [lindex $args 0]
    }
    return [lindex $args 1]
  }

  proc max { args } {
    if {[lindex $args 0] > [lindex $args 1]} {
      return [lindex $args 0]
    }
    return [lindex $args 1]
  }
}

proc randIn { min max } {
  return [expr {rand()*($max - $min) + $min}]
}

proc hypot { dx dy } {
  return [expr {sqrt($dx*$dx + $dy*$dy)}]
}

# ---

# 3D View

setAppValue edit_type "tcl"

setViewportValue "" camera_distance 128
setViewportValue "" bg_color        "#444444"

setViewportValue "" clip [list 0 -1 0] 48
setViewportValue "" clip [list 0  1 0] 60

set scanner [addViewport [list 0 0.8 1.0 0.9]]

setViewportValue $scanner camera_distance 128
setViewportValue $scanner bg_color        "#222222"

setViewportValue $scanner clip [list -1 0 0] 1024
setViewportValue $scanner clip [list  1 0 0] 1024

set bullet_shader [addShader "defender_bullet.vs" "defender_bullet.gs" "defender_bullet.fs"]

setShaderValue $bullet_shader point 1
setShaderValue $bullet_shader line_width 5
 
# ---

# State

set lives 3
set score 0
set level 1

proc addOverlayText { str pos align } {
  set text [addText $str $pos 0.1]

  setTextValue $text overlay   1
  setTextValue $text billboard 1
  setTextValue $text color     white
  setTextValue $text halign    $align
  setTextValue $text viewport  0

  return $text
}

set lives_text [addOverlayText "Lives: $lives" {-0.9 0.9 0} left  ]
set score_text [addOverlayText "Score: $score" { 0.0 0.9 0} center]
set level_text [addOverlayText "Level: $level" { 0.9 0.9 0} right ]

set game_over_text [addOverlayText "GAME OVER" { 0 0 0} center ]
setTextValue $game_over_text visible 0

# ---

# Invaders

proc loadInvader { filename name } {
  set obj [readModel $filename]

  setObjectValue $obj name $name
  setObjectValue $obj visible 0
  setObjectValue $obj scale  [list 2 2 2]
}

set invader1_obj [loadInvader "data/defender_alien1.obj" "invader1"]
set invader2_obj [loadInvader "data/defender_alien2.obj" "invader2"]
set invader3_obj [loadInvader "data/defender_alien3.obj" "invader3"]

set num_invaders 8
set invader_dx   0
set invader_dy   0
set invader_w    16
set invader_h    16
set invader_v    2

proc addInvader { } {
  set i ""

  for {set ii 0} {$ii < $::num_invaders} {incr ii} {
    if {! [info exists ::invader($ii)]} {
      set meta ""

      set ::invader1($ii) [getObjectValue $::invader1_obj ref_object]
      set ::invader2($ii) [getObjectValue $::invader2_obj ref_object]
      set ::invader3($ii) [getObjectValue $::invader3_obj ref_object]

      set ::invader($ii) $::invader1($ii)

      set meta invader1

      set i $ii

      break
    } else {
      set meta [getObjectValue $::invader($ii) meta]

      if {$meta == "dead"} {
        set meta invader1

        set ::invader($ii) $::invader1($ii)

        set i $ii

        break
      }
    }
  }

  if {$i != ""} {
    set ::invader_x($i) [randIn -64 64]
    set ::invader_y($i) [randIn -64 64]

    setObjectValue $::invader($i) name "invader.${i}"
    setObjectValue $::invader($i) meta $meta
    setObjectValue $::invader($i) visible 1
    setObjectValue $::invader($i) translate [list $::invader_x($i) $::invader_y($i) 0] set

    set ::invader_target($i) "human"

    set ::alien_target_x($i) [randIn -256 256]
    set ::alien_target_y($i) [randIn -64 64]
  }
}

proc invaderScore { invader } {
  set meta [getObjectValue $invader meta]

  if {$meta == "invader3"} { return 100 }
  if {$meta == "invader1"} { return 200 }
  if {$meta == "invader4"} { return 300 }

  return 0
}

proc invaderExists { i } {
  if {[info exists ::invader($i)]} {
    set meta [getObjectValue $::invader($i) meta]

    if {$meta != "dead"} {
      return 1
    }
  }

  return 0
}

proc nearestHuman { pos } {
  set x [lindex $pos 0]
  set y [lindex $pos 1]

  set min_human ""
  set min_dist  0

  for {set i 0} {$i < $::num_humans} {incr i} {
    set meta [getObjectValue $::humans($i) meta]
    if {$meta == "captured"} { continue }

    set dx [expr {$::human_x($i) - $x}]
    set dy [expr {$::human_y($i) - $y}]

    set dist [hypot $dx $dy]

    if {$min_human == "" || $dist < $min_dist} {
      set min_human $i
      set min_dist  $dist
    }
  }

  return $min_human
}

proc moveInvaderToTarget { i } {
  set target_dx [expr {$::alien_target_x($i) - $::invader_x($i)}]
  set target_dy [expr {$::alien_target_y($i) - $::invader_y($i)}]

  set l [hypot $target_dx $target_dy]
  if {$l < 1} { return 1 }

  set ::invader_x($i) [expr {$::invader_x($i) + $::invader_v*$target_dx/$l}]
  set ::invader_y($i) [expr {$::invader_y($i) + $::invader_v*$target_dy/$l}]

  setObjectValue $::invader($i) translate [list $::invader_x($i) $::invader_y($i) 0] set

  return 0
}

proc moveInvader { i } {
  set ::invader_x($i) [expr {$::invader_x($i) - $::ship_vx}]
  # find human
  if     {$::invader_target($i) == "human"} {
    set pos [list $::invader_x($i) $::invader_y($i) 0]

    set ih [nearestHuman $pos]

    if {$ih != ""} {
      set ::alien_target_x($i) $::human_x($ih)
      set ::alien_target_y($i) $::human_y($ih)
    }

    if {[moveInvaderToTarget $i]} {
      # pick up human and escape
      if {$ih != ""} {
        set ::invader_target($i) "escape"
        set ::invader_human($i) $ih

        setObjectValue $::invader($i) visible 0

        set ::invader($i) $::invader2($i)

        setObjectValue $::invader($i) visible 1

        setObjectValue $::humans($ih) meta "captured"
      } else {
        # move to random point
        set ::alien_target_x($i) [randIn -256 256]
        set ::alien_target_y($i) [randIn -64 64]
      }
    }
  } elseif {$::invader_target($i) == "escape"} {
    # move to top of screen
    set ::alien_target_x($i) $::invader_x($i)
    set ::alien_target_y($i) 64

    set human $::humans($::invader_human($i))

    if {[moveInvaderToTarget $i]} {
      # escaped with human, become mutant
      set ::invader_target($i) "mutant"

      setObjectValue $::invader($i) visible 0

      set ::invader($i) $::invader3($i)

      setObjectValue $::invader($i) visible 1

      setObjectValue $human visible 0
    } else {
      # move human with alien
      set pos [list $::invader_x($i) [expr {$::invader_y($i) - 8}] 0]

      setObjectValue $human translate $pos set
    }
  } elseif {$::invader_target($i) == "mutant"} {
    # move to ship
    set ::alien_target_x($i) $::ship_x
    set ::alien_target_y($i) $::ship_y

    if {[moveInvaderToTarget $i]} {
      # hit ship
      setObjectValue $::invader($i) meta   "dead"
      setObjectValue $::invader($i) visible 0
    }
  }
}

proc hitInvaders { pos } {
  set px [lindex $pos 0]
  set py [lindex $pos 1]

  for {set ii 0} {$ii < $::num_invaders} {incr ii} {
    set visible [getObjectValue $::invader($ii) visible]
    if {! $visible} { continue }

    set x1 [expr {$::invader_x($ii) - $::invader_w/2}]
    set y1 [expr {$::invader_y($ii) - $::invader_h/2}]
    set x2 [expr {$x1 + $::invader_w}]
    set y2 [expr {$y1 + $::invader_h}]

    if {$px >= $x1 && $px < $x2 && $py >= $y1 && $py < $y2} {
      return $::invader($ii)
    }
  }

  return ""
}

proc invaderShoot { ind } {
  set x [expr {$::invader_x($ind)}]
  set y [expr {$::invader_y($ind) - $::invader_h/2}]

  if {! [info exists ::invader_bullet($ind)]} {
    set ::invader_bullet($ind) [addParticle [list $x $y 0]]

    setParticleValue $::invader_bullet($ind) meta "invader $ind"
  } else {
    set dead [getParticleValue $::invader_bullet($ind) dead]
    if {! $dead} return

    setParticleValue $::invader_bullet($ind) position [vector $x $y 0]
    setParticleValue $::invader_bullet($ind) dead 0
  }

  set pos [getParticleValue $::invader_bullet($ind) position]

  set target_dx [expr {$::ship_x - [lindex $pos 0]}]
  set target_dy [expr {$::ship_y - [lindex $pos 1]}]
  
  set l [hypot $target_dx $target_dy]
  
  if {$l > 0} {
    set vx [expr {100*$target_dx/$l}]
    set vy [expr {100*$target_dy/$l}]

    setParticleValue $::invader_bullet($ind) velocity [list $vx $vy 0]
  }
}

set num_init_invaders 2

for {set i 0} {$i < $num_init_invaders} {incr i} {
  addInvader
}

# ---

# Ship

#set ship_obj [readModel "data/invader_ship.obj"]
#set ship_obj [readModel "models/v3d/F15.V3D"]
set ship_obj [readModel "models/v3d/InfLoopShip.V3D"]

setObjectValue $ship_obj scale  [list 3 3 3]
#setObjectValue $ship_obj rotate [list 0 1 0] -90
setObjectValue $ship_obj rotate [list 0 1 0] 90

set ship_x 0
set ship_y 0
set ship_w 16
set ship_h 16

setObjectValue $ship_obj translate [list $ship_x $ship_y 0] set

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
    if {$::ship_vx > 0} {
      set bx [expr {$::ship_x + 12}]
    } else {
      set bx [expr {$::ship_x - 36}]
    }

    set by [expr {$::ship_y + [randIn -1 1]}]

    if {$::ship_bullet($bullet_num) == ""} {
      set ::ship_bullet($bullet_num) [addParticle [list $bx $by 0]]

      setParticleValue $::ship_bullet($bullet_num) meta ship
      setParticleValue $::ship_bullet($bullet_num) shader $::bullet_shader
    }

    setParticleValue $::ship_bullet($bullet_num) dead     0
    setParticleValue $::ship_bullet($bullet_num) position [list $bx $by 0]

    set shoot_vel [expr {250*$::ship_vx}]

    set ::ship_shoot_vel [vector $shoot_vel 0 0]

    setParticleValue $::ship_bullet($bullet_num) velocity $::ship_shoot_vel
  }
}

set ship_num_bullets 4

for {set i 0} {$i < $ship_num_bullets} {incr i} {
  set ship_bullet($i) ""
}

# ---

# Ground

set ground_dx   256
set ground_xmin -256
set ground_xmax 256
set ground_w    512
set ground_y    -48

proc addGround { x } {
  set ground_obj [readModel "data/defender_ground.obj"]

  setObjectValue $ground_obj scale     [list 2 2 2] set
  setObjectValue $ground_obj translate [list $x $::ground_y 0] set

  return $ground_obj
}

set num_ground 3

set ground_x(0) -192

for {set i 0} {$i < $num_ground} {incr i} {
  if {$i > 0} {
    set i1 [expr {$i - 1}]

    set ground_x($i) [expr {$ground_x($i1) + $ground_dx}]
  }

  set ground_obj($i) [addGround $::ground_x($i)]
}

set ship_vx 1

# ---

# Human

proc addHuman { i x } {
  set ::humans($i) [readModel "data/humanoid_tri.obj"]

  set ::human_x($i) $x
  set ::human_y($i) $::ground_y

  setObjectValue $::humans($i) scale     [list 0.5 0.5 0.5] set
  setObjectValue $::humans($i) translate [list $::human_x($i) $::human_y($i) 0] set
  setObjectValue $::humans($i) rotate    [list 1 0 0] -90
  setObjectValue $::humans($i) rotate    [list 0 1 0] -90

  return $::humans($i)
}

set num_humans 3

addHuman 0 -64
addHuman 1   0
addHuman 2  64

# ---

proc wrapX { x } {
  if       {$x > $::ground_xmax} {
    set x [expr {$x - $::ground_w}]
  } elseif {$x < $::ground_xmin} {
    set x [expr {$x + $::ground_w}]
  }
  return $x
}

# ---

# init

setViewportValue ""       bbox [list -40 -40 -40 40 40 40]
setViewportValue $scanner bbox [list -40 -40 -40 40 40 40]

# ---

proc tickProc { args } {
  foreach particle [getAppValue particles] {
    set dead [getParticleValue $particle dead]
    if {$dead} { continue }

    set meta [getParticleValue $particle meta]

    if {$meta == "ship"} {
      set pos [getParticleValue $particle position]

if {0} {
      set invader [hitInvaders $pos]

      if {$invader != ""} {
        echo "Hit Invader: $invader"

        set ::score [expr {$::score + [invaderScore $invader]}]

        setTextValue $::score_text text "Score: $::score"

        setParticleValue $particle dead 1
        setObjectValue   $invader  visible 0

        continue
      }
}

      set x [lindex $pos 0]

      set vel [getParticleValue $particle velocity]

      set vx [lindex $vel 0]

      if {$vx > 0} {
        if {$x > $::ground_dx} {
          setParticleValue $particle dead 1
          continue
        }
      } else {
        if {$x < -$::ground_dx} {
          setParticleValue $particle dead 1
          continue
        }
      }
    } else {
if {0} {
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
      }

      set y [lindex $pos 1]

      if {$y < -80} {
        setParticleValue $particle dead 1

        continue
      }
}
    }
  }

  set ticks [getAppValue ticks]

  for {set ii 0} {$ii < $::num_invaders} {incr ii} {
    if {[invaderExists $ii]} {
      moveInvader $ii

if {0} {
      if {[randIn 0 1] < 0.1} {
        invaderShoot $ii
      }
}
    }
  }

  if {[randIn 0 1] < 0.1} {
    addInvader
  }

  # Move Ground
  for {set i 0} {$i < $::num_ground} {incr i} {
    set ::ground_x($i) [wrapX [expr {$::ground_x($i) - $::ship_vx}]]

    setObjectValue $::ground_obj($i) translate [list $::ground_x($i) $::ground_y -1] set
  }

  # Move Humans (on Ground)
  for {set i 0} {$i < $::num_humans} {incr i} {
    set meta [getObjectValue $::humans($i) meta]
    if {$meta == "captured"} continue

    set ::human_x($i) [wrapX [expr {$::human_x($i) - $::ship_vx}]]

    setObjectValue $::humans($i) translate [list $::human_x($i) $::human_y($i) 0] set
  }
}

proc keyPress { args } {
  set key   [lindex $args 0]
  set ctrl  [lindex $args 1]
  set shift [lindex $args 2]

  # echo "keyPress ($key) ($ctrl) ($shift)"

  if       {$key == "q" || $key == "Q"} {
    set ::ship_y [expr {$::ship_y + 1}]

    setObjectValue $::ship_obj translate [list $::ship_x $::ship_y 0] set
  } elseif {$key == "z" || $key == "Z"} {
    set ::ship_y [expr {$::ship_y - 1}]

    setObjectValue $::ship_obj translate [list $::ship_x $::ship_y 0] set
  } elseif {$key == "a" || $key == "A"} {
    setObjectValue $::ship_obj rotate [list 0 1 0] 180

    set ::ship_vx [expr {-$::ship_vx}]
  } elseif {$key == "o" || $key == "O"} {
    if {$::ship_vx > 0} {
      set ::ship_vx [expr {$::ship_vx + 1}]
    } else {
      set ::ship_vx [expr {$::ship_vx - 1}]
    }
  } elseif {$key == "p" || $key == "P"} {
    shipShoot
  }
}
