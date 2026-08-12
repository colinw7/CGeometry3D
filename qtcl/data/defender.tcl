# TODO
#  . alien spawn
#  . better scanner
#  . 9 screens wide
#  . bullet
#  . bullet shader (extra lines)
#  . rotating ground
#  . alien hit ship
#  . alien shoot when no human
#  . explode alien

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

# Globals

set debug 0

# ---

# 3D View

addModelDir "data/defender"

setAppValue running 0

setAppValue edit_type "tcl"

setViewportValue "" camera.distance 128
setViewportValue "" bg_color        "#444444"

setViewportValue "" clip [list 0 -1 0] 48
setViewportValue "" clip [list 0  1 0] 60

set scanner [addViewport [list 0.3 0.05 0.7 0.15]]

setViewportValue $scanner camera.distance 128
setViewportValue $scanner bg_color        "#222222"

setViewportValue $scanner clip [list -1 0 0] 1024
setViewportValue $scanner clip [list  1 0 0] 1024

set bullet_shader [addShader "defender_bullet.vs" "defender_bullet.gs" "defender_bullet.fs"]

setShaderValue $bullet_shader point 1
setShaderValue $bullet_shader line_width 5

# ---

# State

proc addOverlayText { str pos align } {
  set text [addText $str $pos 0.1]

  setTextValue $text overlay   1
  setTextValue $text billboard 1
  setTextValue $text color     white
  setTextValue $text halign    $align
  setTextValue $text viewport  0

  return $text
}

set lives_text [addOverlayText "Lives: 0" {-0.9 0.9 0} left  ]
set score_text [addOverlayText "Score: 0" { 0.0 0.9 0} center]
set level_text [addOverlayText "Level: 0" { 0.9 0.9 0} right ]

set game_over_text [addOverlayText "GAME OVER" { 0 0 0} center ]
setTextValue $game_over_text visible 0

# ---

# Aliens

proc loadAlien { filename name } {
  set obj [readModel $filename]

  setObjectValue $obj name $name
  setObjectValue $obj visible 0
  setObjectValue $obj scale  [list 2 2 2]
}

proc addAlien { } {
  if {$::dead_aliens >= $::num_aliens} {
    return
  }

  if {! [info exists ::alien1_obj]} {
    set ::alien1_obj [loadAlien "data/defender/defender_alien1.obj" "alien1"]
    set ::alien2_obj [loadAlien "data/defender/defender_alien2.obj" "alien2"]
    set ::alien3_obj [loadAlien "data/defender/defender_alien3.obj" "alien3"]
  }

  set i ""

  for {set ii 0} {$ii < $::num_aliens} {incr ii} {
    if {! [info exists ::alien($ii)]} {
      set meta ""

      set ::alien1($ii) [getObjectValue $::alien1_obj ref_object]
      set ::alien2($ii) [getObjectValue $::alien2_obj ref_object]
      set ::alien3($ii) [getObjectValue $::alien3_obj ref_object]

      set ::alien($ii) $::alien1($ii)

      set meta alien1

      set i $ii

      break
    } else {
      set meta [getObjectValue $::alien($ii) meta]

      if {$meta == "dead"} {
        set meta alien1

        set ::alien($ii) $::alien1($ii)

        set i $ii

        break
      }
    }
  }

  if {$i != ""} {
    # echo "Add Alien"

    set ::alien_x($i) [randIn -64 64]
    set ::alien_y($i) [randIn -64 64]

    setObjectValue $::alien($i) name "alien.${i}"
    setObjectValue $::alien($i) meta $meta
    setObjectValue $::alien($i) visible 1
    setObjectValue $::alien($i) translate [list $::alien_x($i) $::alien_y($i) 0] set

    set ::alien_target($i) "human"
    set ::alien_human($i) ""

    set ::alien_target_x($i) [randIn -256 256]
    set ::alien_target_y($i) [randIn -64 64]
  }
}

proc alienScore { alien } {
  set meta [getObjectValue $alien meta]

  if {$meta == "alien1"} { return 100 }
  if {$meta == "alien2"} { return 200 }

  echo "No Score for Alien $meta"

  return 0
}

proc alienExists { i } {
  if {[info exists ::alien($i)]} {
    set meta [getObjectValue $::alien($i) meta]

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
    if {$meta != "ground"} { continue }

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

proc numAliveHumans { } {
  set n 0

  for {set i 0} {$i < $::num_humans} {incr i} {
    set meta [getObjectValue $::humans($i) meta]
    if {$meta != "dead"} { incr n }
  }

  return $n
}

proc mutateAlien { i } {
  echo "Mutate Alien"

  set ih $::alien_human($i)

  set ::alien_target($i) "mutant"
  set ::alien_human($i) ""

  setObjectValue $::alien($i) visible 0

  set ::alien($i) $::alien3($i)

  setObjectValue $::alien($i) visible 1

  setObjectValue $::alien($i) translate [list $::alien_x($i) $::alien_y($i) 0] set

  if {$ih != ""} {
    set human $::humans($ih)

    setObjectValue $human visible 0
    setObjectValue $human meta "dead"
  }

  # move to ship
  set ::alien_target_x($i) $::ship_x
  set ::alien_target_y($i) $::ship_y
}

proc alienPickupHuman { i ih } {
  set ::alien_target($i) "escape"
  set ::alien_human($i) $ih

  setObjectValue $::alien($i) visible 0

  set ::alien($i) $::alien2($i)

  setObjectValue $::alien($i) meta "alien2"
  setObjectValue $::alien($i) visible 1

  setObjectValue $::alien($i) translate [list $::alien_x($i) $::alien_y($i) 0] set

  setObjectValue $::humans($ih) meta "captured"
}

# alien hit ship
proc alienHitShip { i } {
  echo "Hit Ship"

  killAlien $i

  shipLoseLife
}

proc killAlien { i } {
  set ::score [expr {$::score + [alienScore $::alien($i)]}]

  setObjectValue $::alien($i) meta   "dead"
  setObjectValue $::alien($i) visible 0

  incr ::dead_aliens

  setTextValue $::score_text text "Score: $::score"

  if {$::dead_aliens == $::num_aliens} {
    nextLevel
  }
}

proc moveAlienToTarget { i } {
  set target_dx [expr {$::alien_target_x($i) - $::alien_x($i)}]
  set target_dy [expr {$::alien_target_y($i) - $::alien_y($i)}]

  set l [hypot $target_dx $target_dy]

  set ::alien_x($i) [wrapX [expr {$::alien_x($i) + $::alien_v*$target_dx/$l}]]
  set ::alien_y($i) [expr {$::alien_y($i) + $::alien_v*$target_dy/$l}]

  setObjectValue $::alien($i) translate [list $::alien_x($i) $::alien_y($i) 0] set

  if {$l < 1} { return 1 }

  return 0
}

proc moveAlien { i } {
  set ::alien_x($i) [wrapX [expr {$::alien_x($i) - $::ship_vx}]]

  # find human
  if     {$::alien_target($i) == "human"} {
    set n [numAliveHumans]

    if {$n == 0} {
      mutateAlien $i
      return
    }

    set pos [list $::alien_x($i) $::alien_y($i) 0]

    set ih [nearestHuman $pos]

    if {$ih != ""} {
      set ::alien_target_x($i) $::human_x($ih)
      set ::alien_target_y($i) $::human_y($ih)
    }

    if {[moveAlienToTarget $i]} {
      # pick up human and escape
      if {$ih != ""} {
        alienPickupHuman $i $ih
      } else {
        # move to random point
        set ::alien_target_x($i) [randIn -256 256]
        set ::alien_target_y($i) [randIn -64 64]

        if {[randIn 0 1] < 0.1} {
          alienShoot $i
        }
      }
    }
  } elseif {$::alien_target($i) == "escape"} {
    # move to top of screen
    set ::alien_target_x($i) $::alien_x($i)
    set ::alien_target_y($i) 64

    set ih $::alien_human($i)

    set human $::humans($ih)

    if {[moveAlienToTarget $i]} {
      # escaped with human, become mutant
      mutateAlien $i
    } else {
      # move human with alien
      set ::human_x($ih) $::alien_x($i)
      set ::human_y($ih) [expr {$::alien_y($i) - 8}]

      setObjectValue $human translate [list $::human_x($ih) $::human_y($ih) 0] set
    }
  } elseif {$::alien_target($i) == "mutant"} {
    set ::alien_target_x($i) $::ship_x
    set ::alien_target_y($i) $::ship_y

    if {[moveAlienToTarget $i]} {
      # hit ship
      alienHitShip $i
    }
  }
}

proc hitAliens { pos } {
  set px [lindex $pos 0]
  set py [lindex $pos 1]

  for {set ii 0} {$ii < $::num_aliens} {incr ii} {
    if {[info exists ::alien($ii)]} {
      set visible [getObjectValue $::alien($ii) visible]
      if {! $visible} { continue }

      set x1 [expr {$::alien_x($ii) - $::alien_w/2}]
      set y1 [expr {$::alien_y($ii) - $::alien_h/2}]
      set x2 [expr {$x1 + $::alien_w}]
      set y2 [expr {$y1 + $::alien_h}]

      if {$px >= $x1 && $px < $x2 && $py >= $y1 && $py < $y2} {
        return $ii
      }
    }
  }

  return ""
}

proc alienShoot { ind } {
  set x [expr {$::alien_x($ind)}]
  set y [expr {$::alien_y($ind) - $::alien_h/2}]

  if {! [info exists ::alien_bullet($ind)]} {
    set ::alien_bullet($ind) [addParticle [list $x $y 0]]

    setParticleValue $::alien_bullet($ind) meta "alien $ind"
  } else {
    set dead [getParticleValue $::alien_bullet($ind) dead]
    if {! $dead} return

    setParticleValue $::alien_bullet($ind) position [vector $x $y 0]
    setParticleValue $::alien_bullet($ind) dead 0
  }

  set pos [getParticleValue $::alien_bullet($ind) position]

  set target_dx [expr {$::ship_x - [lindex $pos 0]}]
  set target_dy [expr {$::ship_y - [lindex $pos 1]}]

  set l [hypot $target_dx $target_dy]

  if {$l > 0} {
    set vx [expr {100*$target_dx/$l}]
    set vy [expr {100*$target_dy/$l}]

    setParticleValue $::alien_bullet($ind) velocity [list $vx $vy 0]
  }
}

# ---

# Ship

#set ship_obj [readModel "data/alien_ship.obj"]
#set ship_obj [readModel "models/v3d/F15.V3D"]
set ship_obj [readModel "models/v3d/InfLoopShip.V3D"]

setObjectValue $ship_obj scale  [list 2 2 2]
#setObjectValue $ship_obj rotate [list 0 1 0] -90
setObjectValue $ship_obj rotate [list 0 1 0] 90

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

proc gameOver { } {
  setObjectValue $::ship_obj visible 0

  setTextValue $::game_over_text visible 1
  setAppValue  running 0
}

proc nextLevel { } {
  restartGame 1
}

proc restartGame { nextLevel } {
  setAppValue running 0

  initGround

  set ::num_aliens  8
  set ::dead_aliens 0
  set ::alien_dx    0
  set ::alien_dy    0
  set ::alien_w     16
  set ::alien_h     16
  set ::alien_v     2

  for {set i 0} {$i < $::num_aliens} {incr i} {
    if {[info exists ::alien_bullet($i)]} {
      setParticleValue $::alien_bullet($i) dead 0
    }

    if {[info exists ::alien($i)]} {
      setObjectValue $::alien($i) meta "dead"
      setObjectValue $::alien($i) visible 0

      set ::alien($i) $::alien1($i)

      setObjectValue $::alien($i) meta "dead"
      setObjectValue $::alien($i) visible 0
    }

    set ::alien_target($i) "human"
    set ::alien_human($i) ""
  }

  set ::num_init_aliens 2

  for {set i 0} {$i < $::num_init_aliens} {incr i} {
    addAlien
  }

  if {$nextLevel == 0} {
    set ::lives 3
    set ::score 0
    set ::level 1

    setTextValue $::lives_text text "Lives: $::lives"
    setTextValue $::score_text text "Score: $::score"
    setTextValue $::level_text text "Level: $::level"

    if {[info exists ::ship_dir]} {
      if {$::ship_dir != 1} {
        setObjectValue $::ship_obj rotate [list 0 1 0] 180
      }
    }
  } else {
    incr ::level

    setTextValue $::level_text text "Level: $::level"
  }

  if {$nextLevel == 0} {
    set ::ship_dx     64
    set ::ship_w      16
    set ::ship_h      16
    set ::ship_dir    1
    set ::ship_vx     1
    set ::ship_ax     1.5
    set ::ship_min_vx 1
    set ::ship_max_vx 20
    set ::ship_vy     4

    set ::ship_target_x ""
    set ::ship_turning  0

    set ::ship_bullet_xmax 96

    set ::ship_x -$::ship_dx
    set ::ship_y 0

    setObjectValue $::ship_obj translate [list $::ship_x $::ship_y 0] set

    # ---

    set ::ship_num_bullets 4

    for {set i 0} {$i < $::ship_num_bullets} {incr i} {
      set ::ship_bullet($i) ""
    }
  }

  # ---

  set ::num_humans 3

  addHuman 0 -64
  addHuman 1   0
  addHuman 2  64

  # ---

  foreach particle [getAppValue particles] {
    setParticleValue $particle dead 1
  }

  # ---

  setViewportValue ""         bbox [list -40 -40 -40 40 40 40]
  setViewportValue $::scanner bbox [list -40 -40 -40 40 40 40]

  setViewportValue "" camera.distance 128

  # ---

  setAppValue running 1
}

proc shipLoseLife { } {
  if {$::lives >= 1} {
    set ::lives [expr {$::lives - 1}]
  }

  if {$::lives == 0} {
    gameOver
  }

  setTextValue $::lives_text text "Lives: $::lives"
}

proc shipUp { } {
  set ::ship_y [limitShipY [expr {$::ship_y + $::ship_vy}]]

  setObjectValue $::ship_obj translate [list $::ship_x $::ship_y 0] set
}

proc shipDown { } {
  set ::ship_y [limitShipY [expr {$::ship_y - $::ship_vy}]]

  setObjectValue $::ship_obj translate [list $::ship_x $::ship_y 0] set
}

proc shipTurn { } {
  if {! $::ship_turning} {
    set ::ship_turning 1

    setObjectValue $::ship_obj rotate [list 0 1 0] 180

    set ::ship_dir [expr {-$::ship_dir}]
  # set ::ship_vx  [expr {-$::ship_vx}]

    if {$::ship_dir > 0} {
      set ::ship_target_x -$::ship_dx
    } else {
      set ::ship_target_x $::ship_dx
    }

    setObjectValue $::ship_obj translate [list $::ship_x $::ship_y 0] set
  }
}

proc shipThrust { } {
  if {$::ship_dir > 0} {
    set ::ship_vx [expr {$::ship_vx + $::ship_ax}]
  } else {
    set ::ship_vx [expr {$::ship_vx - $::ship_ax}]
  }

  if {$::ship_vx > $::ship_max_vx} {
    set ::ship_vx $::ship_max_vx
  }
  if {$::ship_vx < -$::ship_max_vx} {
    set ::ship_vx -$::ship_max_vx
  }

  if {$::ship_vx <= 0 && $::ship_vx > -$::ship_min_vx} {
    set ::ship_vx -$::ship_min_vx
  }
  if {$::ship_vx >= 0 && $::ship_vx < $::ship_min_vx} {
    set ::ship_vx $::ship_min_vx
  }

  if {0} {
  if {$::ship_vx > 0} {
    set ::ship_x -$::ship_dx
  } else {
    set ::ship_x $::ship_dx
  }

  setObjectValue $::ship_obj translate [list $::ship_x $::ship_y 0] set
  }
}

proc shipShoot { } {
  # get free bullet
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

  if {$bullet_num == ""} {
    return
  }

  # ---

  # set buller start position
  if {$::ship_dir > 0} {
    set bx [expr {$::ship_x + 12}]
  } else {
    set bx [expr {$::ship_x - 36}]
  }

  set by [expr {$::ship_y + [randIn -1 1]}]

  # ---

  # add bullet particle if needed
  if {$::ship_bullet($bullet_num) == ""} {
    set ::ship_bullet($bullet_num) [addParticle [list $bx $by 0]]

    setParticleValue $::ship_bullet($bullet_num) meta ship
    setParticleValue $::ship_bullet($bullet_num) shader $::bullet_shader
  }

  # ---

  # reset bullet state
  setParticleValue $::ship_bullet($bullet_num) dead     0
  setParticleValue $::ship_bullet($bullet_num) position [list $bx $by 0]

  # ---

  # set bullet velocity
  set bullet_vx 800

  set shoot_vel [expr {$::ship_vx + $bullet_vx*$::ship_dir}]

  set ::ship_shoot_vel [vector $shoot_vel 0 0]

  setParticleValue $::ship_bullet($bullet_num) velocity $::ship_shoot_vel
}

# ---

# Ground

proc addGround { x } {
  set ground_obj [readModel "data/defender/defender_ground1.obj"]

  setObjectValue $ground_obj scale     [list 2 2 2] set
  setObjectValue $ground_obj translate [list $x $::ground_y 0] set

  return $ground_obj
}

proc initGround { } {
  set ::ground_dx   256
  set ::ground_xmin -256
  set ::ground_xmax 256
  set ::ground_w    512
  set ::ground_y    -48

  set ::num_ground 3

  set ::ground_x(0) -192

  for {set i 0} {$i < $::num_ground} {incr i} {
    if {$i > 0} {
      set i1 [expr {$i - 1}]

      set ::ground_x($i) [expr {$::ground_x($i1) + $::ground_dx}]
    }

    if {! [info exists ::ground_obj($i)]} {
      set ::ground_obj($i) [addGround $::ground_x($i)]
    }
  }
}

# ---

# Human

proc addHuman { i x } {
  set ::human_x($i) $x
  set ::human_y($i) $::ground_y

  if {! [info exists ::humans($i)]} {
    set ::humans($i) [readModel "data/defender/humanoid_tri.obj"]

    setObjectValue $::humans($i) scale     [list 0.5 0.5 0.5] set
    setObjectValue $::humans($i) translate [list $::human_x($i) $::human_y($i) 0] set
    setObjectValue $::humans($i) rotate    [list 1 0 0] -90
    setObjectValue $::humans($i) rotate    [list 0 1 0] -90
  }

  setObjectValue $::humans($i) meta    "ground"
  setObjectValue $::humans($i) visible 1

  return $::humans($i)
}

# ---

proc wrapX { x } {
  if       {$x > $::ground_xmax} {
    set x [expr {$x - $::ground_w}]
  } elseif {$x < $::ground_xmin} {
    set x [expr {$x + $::ground_w}]
  }
  return $x
}

proc limitShipY { y } {
  if       {$y > 48} {
    set y 48
  } elseif {$y < $::ground_y} {
    set y $::ground_y
  }
  return $y
}

# ---

proc hitAlien { } {
}

proc updateShipBullet { particle } {
  set pos [getParticleValue $particle position]

  # if bullet hit invader then destroy alien and bullet and update any grabbed human
  set ii [hitAliens $pos]

  if {$ii != ""} {
    set alien $::alien($ii)

    echo "Hit Alien: $alien"

    setParticleValue $particle dead 1

    killAlien $ii

    if {$::alien_human($ii) != ""} {
      set human $::humans($::alien_human($ii))

      setObjectValue $human meta "falling"
    }

    set ::alien_target($ii) ""
    set ::alien_human($ii) ""

    return
  }

  set x [lindex $pos 0]

  set vel [getParticleValue $particle velocity]

  set vx [lindex $vel 0]

  # off screen so destroy bullet
  if {$x > $::ship_bullet_xmax || $x < -$::ship_bullet_xmax} {
    setParticleValue $particle dead 1
    return
  }
}

proc updateAlienBullet { particle } {
  set pos [getParticleValue $particle position]

  if {[hitShip $pos]} {
    setParticleValue $particle dead 1

    shipLoseLife

    return
  }

  set y [lindex $pos 1]

  if {$y < -80} {
    setParticleValue $particle dead 1

    return
  }
}

proc updateGround { } {
  # Move Ground
  for {set i 0} {$i < $::num_ground} {incr i} {
    set ::ground_x($i) [wrapX [expr {$::ground_x($i) - $::ship_vx}]]

    setObjectValue $::ground_obj($i) translate [list $::ground_x($i) $::ground_y -1] set
  }
}

proc updateHumans { } {
  for {set i 0} {$i < $::num_humans} {incr i} {
    set meta [getObjectValue $::humans($i) meta]

    if       {$meta == "ground"} {
      # Move Humans (on Ground)
      set ::human_x($i) [wrapX [expr {$::human_x($i) - $::ship_vx}]]

      setObjectValue $::humans($i) translate [list $::human_x($i) $::human_y($i) 0] set
    } elseif {$meta == "captured"} {
      # Updated by moveAlien
    } elseif {$meta == "falling"} {
      set ::human_y($i) [expr {$::human_y($i) - 1}]

      setObjectValue $::humans($i) translate [list $::human_x($i) $::human_y($i) 0] set

      if {$::human_y($i) <= $::ground_y } {
        setObjectValue $::humans($i) meta "ground"
      }
    }
  }
}

proc moveShip { } {
  if {$::ship_turning} {
    set target_dx [expr {$::ship_target_x - $::ship_x}]
    set target_dy 0

    set l [hypot $target_dx $target_dy]

    if {$l > 1} {
      set v [expr {1.5*sqrt($l)}]

      set ::ship_x [expr {$::ship_x + $v*$target_dx/$l}]

      setObjectValue $::ship_obj translate [list $::ship_x $::ship_y 0] set
    } else {
      set ::ship_target_x ""
      set ::ship_turning  0
    }
  }
}

proc tickProc { args } {
  set ::ticks [getAppValue ticks]

  if       {[getAppValue key "q"]} {
    shipUp
  } elseif {[getAppValue key "z"]} {
    shipDown
  }

  if {[getAppValue key "a"]} {
    shipTurn
  }

  if {[getAppValue key "o"]} {
    shipThrust
  }

  if {[getAppValue key "p"]} {
    shipShoot
  }

  #---

  moveShip

  # process particles (bullets)
  foreach particle [getAppValue particles] {
    set dead [getParticleValue $particle dead]
    if {$dead} { continue }

    set meta [getParticleValue $particle meta]

    if {$meta == "ship"} {
      updateShipBullet $particle
    } else {
      updateAlienBullet $particle
    }
  }

  # update aliens
  for {set ii 0} {$ii < $::num_aliens} {incr ii} {
    if {[alienExists $ii]} {
      moveAlien $ii
    }
  }

  # add new alien
  if {[randIn 0 1] < 0.1} {
    addAlien
  }

  # move ground
  updateGround

  # move humans (with ground)
  updateHumans
}

proc keyPress { args } {
  set key [lindex $args 0]

  set ctrl  [lindex $args 1]
  set shift [lindex $args 2]

  # echo "keyPress ($key) ($ctrl) ($shift)"

  if {0} {
  if       {$key == "q" || $key == "Q"} {
    shipUp
  } elseif {$key == "z" || $key == "Z"} {
    shipDown
  } elseif {$key == "a" || $key == "A"} {
    shipTurn
  } elseif {$key == "o" || $key == "O"} {
    shipThrust
  } elseif {$key == "p" || $key == "P"} {
    shipShoot
  }
  }

  if       {$key == "d" || $key == "D"} {
    set ::debug [expr {1 - $::debug}]
  } elseif {$key == "r" || $key == "R"} {
    restartGame 0
  }
}

restartGame 0
