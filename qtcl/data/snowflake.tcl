# injection data
set inject_ticks 5 ; # time to next particle

set velocityXMin -25.00 ; # x velocity min/max
set velocityXMax  25.00
set velocityYMin  50.00 ; # y velocity min/max
set velocityYMax 150.00

set width  100
set height 100

set x1 [expr {0.25*$::width }]
set y1 [expr {0.25*$::height}]
set x2 [expr {0.75*$::width }]
set y2 [expr {0.75*$::height}]

set width2  [expr { $width/2.0}]
set height2 [expr {$height/2.0}]

set num_wind 8

# current state
set tick_count   0
set inject_count 0

#----

proc randIn { min max } {
  return [expr {rand()*($max - $min) + $min}]
}

proc getWind { y } {
  set h [expr {$::num_wind*$y/($::height - 1)}]

  set h1 [expr {int($h)}]
  set h2 [expr {$h1 + 1}]

  if {$h1 < 0 || $h1 >= $::num_wind} {
    return 0.0
  }

  return [expr {$::wind($h1) + ($h - $h1)*($::wind($h2) - $::wind($h1))/($h2 - $h1)}]
}

proc injectParticle { } {
  set x [randIn 0 $::width]
  set y [expr {$::height + 100}]

  set p [addParticle [list $x $y 1]]

  set vx [randIn $::velocityXMin $::velocityXMax]
  set vy [randIn $::velocityYMin $::velocityYMax]

  setParticleValue $p velocity [list $vx -$vy 0.0]

  set r [randIn 0.9 1.0]
  set g [randIn 0.9 1.0]
  set b [randIn 0.8 1.0]

  set size  [randIn 4 12]
  set type  [randIn 0 1]
  set angle [randIn 0 30]
  set alpha [randIn 0.5 1.0]

  setParticleValue $p meta  $type
  setParticleValue $p mass  1
  setParticleValue $p color [list $r $g $b]
  setParticleValue $p size  $size
  setParticleValue $p angle $angle
  setParticleValue $p alpha $alpha

  if       {$type < 0.33} {
    setParticleValue $p texture $::t1
  } elseif {$type < 0.66} {
    setParticleValue $p texture $::t2
  } else {
    setParticleValue $p texture $::t3
  }
}

# remap value (0 - 1) to range (vmin - vmax)
proc remap { v vmin vmax } {
  return [expr {($v - $vmin)/($vmax - $vmin) - 0.5}]
}

proc tickProc { args } {
  incr ::tick_count

  if {$::inject_count == 0} {
    injectParticle
  }

  incr ::inject_count

  if {$::inject_count > $::inject_ticks} {
    set ::inject_count 0
  }

  foreach particle [getAppValue particles] {
    set dead [getParticleValue $particle dead]
    if {$dead} { continue }

    set pos [getParticleValue $particle position]

    set wind [getWind [lindex $pos 0]]

    set angle [getParticleValue $particle angle]

    if {$wind > 0} {
      set angle [expr {$angle + 1}]
    } else {
      set angle [expr {$angle - 1}]
    }

    setParticleValue $particle angle $angle

    set vel [getParticleValue $particle velocity]

    set vx [expr {[lindex $vel 0] + $wind}]
    set vy [lindex $vel 1]
    set vz [lindex $vel 2]

    setParticleValue $particle velocity [list $vx $vy $vz]

    if {[lindex $pos 1] <= -50} {
      setParticleValue $particle dead 1
      continue
    }
  }
}

#----

set tbg [addTexture "data/snowman.png"]

set mbg [addMaterial bg]; setMaterialValue $mbg texture $tbg

set t1 [addTexture "data/snowflake.png"]
set t2 [addTexture "data/snowflake1.png"]
set t3 [addTexture "data/snowflake2.png"]

for {set i 0} {$i <= $num_wind} {incr i} {
  if {$i & 1} {
    set wind($i) [randIn -1 0]
  } else {
    set wind($i) [randIn 0 1]
  }
}

setAppValue fixed_diffuse 1

setAppValue gravity -9.8

setAppValue fov 90

#setAppValue perspective 0

set plane_scale 0.275
#set plane_scale 0.1

set plane [addPlane [expr {$plane_scale*945}] [expr {$plane_scale*649}]]

setObjectValue $plane material  $mbg
setObjectValue $plane rotate    [list 1 0 0] -90
setObjectValue $plane translate [list $width2 $height2 2]

setAppValue bbox [list $::x1 $::y1 $::x1 $::x2 $::y2 $::x2]

for {set i 0} {$i < 1000} {incr i} {
  setAppValue tick 1
}
