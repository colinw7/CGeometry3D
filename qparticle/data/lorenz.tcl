setAppValue point_size 3

setAppValue gravity 0.0

set in 0
set n  8000

set ix 0.0
set iy 1.0
set iz 0.0

set dt 0.01

set A { 10.0 }
set B { 28.0 }
set C [expr { 8.0/3.0 }]

proc calcX { t x y z } {
  return [expr {$::A*($y - $x)}]
}

proc calcY { t x y z } {
  return [expr {$x*($::B - $z) - $y}]
}

proc calcZ { t x y z } {
  return [expr {$x*$y - $::C*$z}]
}

set rk_t 0.0
set rk_x 0.0
set rk_y 1.0
set rk_z 0.0

set rk_dt 0.01

set colors {}
set color_num 0

if {0} {
lappend colors [getAppValue color "green"]
lappend colors [getAppValue color "blue"]
lappend colors [getAppValue color "cyan"]
lappend colors [getAppValue color "magenta"]
lappend colors [getAppValue color "yellow"]
lappend colors [getAppValue color "orange"]
lappend colors [getAppValue color "brown"]
lappend colors [getAppValue color "grey50"]
lappend colors [getAppValue color "pink"]
lappend colors [getAppValue color "purple"]
lappend colors [getAppValue color "darkgreen"]
lappend colors [getAppValue color "lightblue"]
lappend colors [getAppValue color "seagreen"]
lappend colors [getAppValue color "lightsteelblue"]
lappend colors [getAppValue color "tan"]
lappend colors [getAppValue color "black"]
lappend colors [getAppValue color "white"]
} else {
set nc 32
for {set i 0} {$i < $nc} {incr i} {
  lappend colors [list [expr {(1.0*$i)/$nc}] 0 0]
}
for {set i 0} {$i < $nc} {incr i} {
  lappend colors [list 0 [expr {(1.0*$i)/$nc}] 0]
}
for {set i 0} {$i < $nc} {incr i} {
  lappend colors [list 0 0 [expr {(1.0*$i)/$nc}]]
}
}

set t1 [addTexture "data/particle.png"]

proc RungeKuttaStep { } {
  set kx1 [calcX $::rk_t $::rk_x $::rk_y $::rk_z]
  set ky1 [calcY $::rk_t $::rk_x $::rk_y $::rk_z]
  set kz1 [calcZ $::rk_t $::rk_x $::rk_y $::rk_z]

  set dt2 [expr {$::rk_dt/2.0}]
  set dt6 [expr {$::rk_dt/6.0}]

  set dt [expr {$::rk_t + $dt2}]

  set xt [expr {$::rk_x + $::rk_dt*$kx1/2.0}]
  set yt [expr {$::rk_y + $::rk_dt*$ky1/2.0}]
  set zt [expr {$::rk_z + $::rk_dt*$kz1/2.0}]

  set kx2 [calcX $dt $xt $yt $zt]
  set ky2 [calcY $dt $xt $yt $zt]
  set kz2 [calcZ $dt $xt $yt $zt]

  set xt [expr {$::rk_x + $::rk_dt*$kx2/2.0}]
  set yt [expr {$::rk_y + $::rk_dt*$ky2/2.0}]
  set zt [expr {$::rk_z + $::rk_dt*$kz2/2.0}]

  set kx3 [calcX $dt $xt $yt $zt]
  set ky3 [calcY $dt $xt $yt $zt]
  set kz3 [calcZ $dt $xt $yt $zt]

  set xt [expr {$::rk_x + $::rk_dt*$kx3}]
  set yt [expr {$::rk_y + $::rk_dt*$ky3}]
  set zt [expr {$::rk_z + $::rk_dt*$kz3}]

  set kx4 [calcX $dt $xt $yt $zt]
  set ky4 [calcY $dt $xt $yt $zt]
  set kz4 [calcZ $dt $xt $yt $zt]

  set ::rk_x [expr {$::rk_x + $dt6*($kx1 + 2.0*$kx2 + 2.0*$kx3 + $kx4)}]
  set ::rk_y [expr {$::rk_y + $dt6*($ky1 + 2.0*$ky2 + 2.0*$ky3 + $ky4)}]
  set ::rk_z [expr {$::rk_z + $dt6*($kz1 + 2.0*$kz2 + 2.0*$kz3 + $kz4)}]
  set ::rk_t [expr {$::rk_t + $::rk_dt}]
}

proc nextValue { } {
  RungeKuttaStep

  return [list $::rk_x $::rk_y $::rk_z]
}

proc remap { v vmin vmax } {
  return [expr {($v - $vmin)/($vmax - $vmin) - 0.5}]
}

set xmin -17.99
set ymin -24.15
set zmin   0.00
set xmax  19.83
set ymax  27.64
set zmax  48.31

set x1 0.0
set y1 0.0
set z1 0.0

proc nextColor { } {
  incr ::color_num

  if {$::color_num >= [llength $::colors]} {
    set ::color_num 0
  } 
}

for {set i 0} {$i < $n} {incr i} {
  set pos [nextValue]

  if {$i >= $in} {
    set x [remap [lindex $pos 0] $xmin $xmax]
    set y [remap [lindex $pos 1] $ymin $ymax]
    set z [remap [lindex $pos 2] $zmin $zmax]

    # echo "$x $y $z"

    set p [addParticle [list $x $y $z]]

if {0} {
    if {$x1*$x < 0 || $y1*$y < 0 || $z1*$z < 0} {
      nextColor
    }
} else {
    if {$x1*$x < 0} {
      nextColor
    }
}

    set color [lindex $colors $color_num]

    setParticleValue $p color $color
    setParticleValue $p texture $t1

    set x1 $x
    set y1 $y
    set z1 $z
  }
}
