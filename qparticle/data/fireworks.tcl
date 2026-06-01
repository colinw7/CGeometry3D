set t1 [addTexture "data/particle.png"]
set t2 [addTexture "data/explode.png"]

set t2s 0.2

setAppValue gravity 0

# injection data
set inject_ticks 10 ; # time to next particle

set velocityXMin -4.00 ; # x velocity min/max
set velocityXMax  4.00
set velocityYMin  4.00  ; # y velocity min/max
set velocityYMax  9.00

# explode data
set explode_ticks    40 ; # time to explode
set explosion_ticks1 40 ; # explosion frame 1
set explosion_ticks2 45 ; # explosion frame 2
set explosion_ticks3 50 ; # explosion frame 3
set explosion_ticks4 55 ; # explosion frame 4
set explosion_ticks5 60 ; # explosion frame 5
set explosion_ticks6 65 ; # explosion frame 6
set explosion_ticks7 70 ; # explosion frame 7
set explosion_ticks8 75 ; # explosion frame 8
set explosion_ticks  80 ; # explosion end

# current state
set tick_count   0
set inject_count 0

set ox 0
set oy -2.0

set oy1 [expr {$oy - 0.1}]

proc randIn { min max } {
  return [expr {rand()*($max - $min) + $min}]
}

proc explodeVelocity { a } {
  set r  2.0
  set dr [expr {$r/10}]

  set vx [expr {$r*cos($a)}]
  set vy [expr {$r*sin($a)}]

  set vx1 [expr {$vx - $dr}]
  set vx2 [expr {$vx + $dr}]
  set vy1 [expr {$vy - $dr}]
  set vy2 [expr {$vy + $dr}]

  return [list [randIn $vx1 $vx2] [randIn $vy1 $vy2] 0.0]
}

proc explodeParticle { p } {
  set pos [getParticleValue $p position]

  set r [randIn 0.6 0.8]
  set g [randIn 0.6 0.8]
  set b [randIn 0.6 0.8]

  # create explosion particles
  set pl {}

  for {set i 0} {$i < 8} {incr i} {
    set p [addParticle $pos]

    setParticleValue $p meta  "explode"
    setParticleValue $p age   [expr $::explode_ticks + 1]
    setParticleValue $p color [list $r $g $b]

    set angle [expr {$i*45}]

    # set velocity in random directions
    setParticleValue $p velocity [explodeVelocity $angle]]

    setParticleValue $p texture $::t2
    setParticleValue $p size    0.5
    setParticleValue $p angle   $angle
    setParticleValue $p tpos    [list 0 0.66]
    setParticleValue $p tsize   [list 0.2 0.33]

    lappend pl $p
  }

if {0} {
  foreach p $pl {
    echo [getParticleValue $p velocity]
  }
}
}

proc injectParticle { } {
  set p [addParticle [list $::ox $::oy 0]]

  set vx [randIn $::velocityXMin $::velocityXMax]
  set vy [randIn $::velocityYMin $::velocityYMax]

  setParticleValue $p velocity [list $vx $vy 0.0]

  set r [randIn 0.5 1.0]
  set g [randIn 0.5 1.0]
  set b [randIn 0.5 1.0]

  setParticleValue $p meta    "inject"
  setParticleValue $p color   [list $r $g $b]
  setParticleValue $p age     0
  setParticleValue $p texture $::t1
  setParticleValue $p size    0.2
}

# remap value (0 - 1) to range (vmin - vmax)
proc remap { v vmin vmax } {
  return [expr {($v - $vmin)/($vmax - $vmin) - 0.5}]
}

proc tickProc { args } {
  #echo "tickProc"

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

    if {[lindex $pos 1] <= $::oy1} {
      setParticleValue $particle dead 1
      continue
    }

    set age [getParticleValue $particle age]

    set meta [getParticleValue $particle meta]

    if {$meta == "inject"} {
      if       {$age == $::explode_ticks} {
        explodeParticle $particle

        setParticleValue $particle dead 1
      } else {
        set age1 [expr {$age + 1.0}]

        setParticleValue $particle age $age1
      }
    } else {
      set age1 [expr {$age + 1.0}]

      setParticleValue $particle age $age1

      set alpha [expr {($::explosion_ticks - $age1)/($::explosion_ticks - $::explode_ticks)}]
      setParticleValue $particle alpha $alpha

      if       {$age1 > $::explosion_ticks} {
        setParticleValue $particle dead 1
      } elseif {$age1 >= $::explosion_ticks8} {
        setParticleValue $particle tpos [list 0.8 0.33]
        setParticleValue $particle size 0.80
      } elseif {$age1 >= $::explosion_ticks7} {
        setParticleValue $particle tpos [list 0.6 0.33]
        setParticleValue $particle size 0.70
      } elseif {$age1 >= $::explosion_ticks6} {
        setParticleValue $particle tpos [list 0.4 0.33]
        setParticleValue $particle size 0.60
      } elseif {$age1 >= $::explosion_ticks5} {
        setParticleValue $particle tpos [list 0.2 0.33]
        setParticleValue $particle size 0.50
      } elseif {$age1 >= $::explosion_ticks4} {
        setParticleValue $particle tpos [list 0.8 0.66]
        setParticleValue $particle size 0.40
      } elseif {$age1 >= $::explosion_ticks3} {
        setParticleValue $particle tpos [list 0.6 0.66]
        setParticleValue $particle size 0.40
      } elseif {$age1 >= $::explosion_ticks2} {
        setParticleValue $particle tpos [list 0.4 0.66]
        setParticleValue $particle size 0.20
      } elseif {$age1 >= $::explosion_ticks1} {
        setParticleValue $particle tpos [list 0.2 0.66]
        setParticleValue $particle size 0.10
      }
    }
  }
}
