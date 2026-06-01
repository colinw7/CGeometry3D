set numBoids  200
set numFlocks 4

set MaxSpeed       1.0
set MinUrgency     0.05
set MaxUrgency     0.1
set MaxChange      [expr {$MaxSpeed*$MaxUrgency}]
set DesiredSpeed   [expr {$MaxSpeed/2.0}]
set KeepAwayDist   6.0
set SeparationDist 3.0
set WorldSize      50.0

set Max_Friends_Visible      10
set Default_Perception_Range 8.0

set UseTruth       0
set ReactToEnemies 1

set nearestFriend     ""
set nearestFriendDist 0

set nearestEnemy     ""
set nearestEnemyDist 0

set HalfWorldSize [expr {$::WorldSize/2.0}]

set t1 [addTexture "data/bird.png"]

setAppValue gravity    0
setAppValue point_size 1

setAppValue bbox [list -$HalfWorldSize -$HalfWorldSize -$HalfWorldSize $HalfWorldSize $HalfWorldSize $HalfWorldSize]

proc tcl::mathfunc::sign {x} {
  if {$x < 0} { return -1 }
  if {$x > 0} { return  1 }
  return 0
}

proc randIn { min max } {
  return [expr {rand()*($max - $min) + $min}]
}

proc addBoid { ind } {
  set x [randIn -$::HalfWorldSize $::HalfWorldSize]
  set y [randIn -$::HalfWorldSize $::HalfWorldSize]
  set z [randIn -$::HalfWorldSize $::HalfWorldSize]

  set p [addParticle [list $x $y $z]]

  set xv [randIn -1 1]
  set yv [randIn -1 1]

  setParticleValue $p velocity [list $xv $yv 0]
  setParticleValue $p angle    0
  setParticleValue $p size     2
  setParticleValue $p texture  $::t1

  return $p
}

proc addFlock { ind } {
  return $ind
}

proc addFlockBoid { flock boid c } {
  setParticleValue $boid meta  $flock
  setParticleValue $boid color $c

  lappend ::flock_boids($flock) $boid
}

for {set i 0} {$i < $::numBoids} {incr i} {
  set boids($i) [addBoid $i]
}

for {set i 0} {$i < $::numFlocks} {incr i} {
  set flocks($i) [addFlock $i]
}

set i 0

set c [list 1 0 0]

for {set i1 0} {$i1 < 50} {incr i1} {
  addFlockBoid $flocks(0) $boids($i) $c

  incr i
}

set c [list 0 1 0]

for {set i1 0} {$i1 < 120} {incr i1} {
  addFlockBoid $flocks(1) $boids($i) $c

  incr i
}

set c [list 0 0 1]

for {set i1 0} {$i1 < 20} {incr i1} {
  addFlockBoid $flocks(2) $boids($i) $c

  incr i
}

set c [list 1 1 0]

for {set i1 0} {$i1 < 10} {incr i1} {
  addFlockBoid $flocks(3) $boids($i) $c

  incr i
}

proc addPoint { p1 p2 } {
  set x1 [lindex $p1 0]
  set y1 [lindex $p1 1]
  set z1 [lindex $p1 2]

  set x2 [lindex $p2 0]
  set y2 [lindex $p2 1]
  set z2 [lindex $p2 2]

  return [list [expr {$x1 + $x2}] [expr {$y1 + $y2}] [expr {$z1 + $z2}]]
}

proc diffPoint { p1 p2 } {
  set x1 [lindex $p1 0]
  set y1 [lindex $p1 1]
  set z1 [lindex $p1 2]

  set x2 [lindex $p2 0]
  set y2 [lindex $p2 1]
  set z2 [lindex $p2 2]

  return [list [expr {$x1 - $x2}] [expr {$y1 - $y2}] [expr {$z1 - $z2}]]
}

proc dividePoint { p s } {
  set x [lindex $p 0]
  set y [lindex $p 1]
  set z [lindex $p 2]

  return [list [expr {$x/$s}] [expr {$y/$s}] [expr {$z/$s}]]
}

proc PointDistance { p1 p2 } {
  set dx [expr {[lindex $p1 0] - [lindex $p2 0]}]
  set dy [expr {[lindex $p1 1] - [lindex $p2 1]}]
  set dz [expr {[lindex $p1 2] - [lindex $p2 2]}]

  return [expr {sqrt($dx*$dx + $dy*$dy + $dz*$dz)}]
}

proc BoidDistance { boid1 boid2 } {
  set pos1 [getParticleValue $boid1 position]
  set pos2 [getParticleValue $boid2 position]

  return [PointDistance $pos1 $pos2]
}

proc SeeFriends { boid } {
  set flock [getParticleValue $boid meta]

  set ::nearestFriend     ""
  set ::nearestFriendDist 0
  set ::friendCenter      [list 0 0 0]

  set visible {}

  foreach boid1 $::flock_boids($flock) {
    if {$boid == $boid1} {
      continue
    }

    set d [BoidDistance $boid $boid1]

    if {$d < $::Default_Perception_Range} {
      lappend visible $boid1

      if {$::nearestFriend == "" || $d < $::nearestFriendDist  } {
        set ::nearestFriend     $boid1
        set ::nearestFriendDist $d
      }
    }

    set pos [getParticleValue $boid1 position]

    set ::friendCenter [addPoint $::friendCenter $pos]
  }

  set nv [llength $::flock_boids($flock)]

  set ::friendCenter [dividePoint $::friendCenter $nv]
}

proc SeeEnemies { boid } {
  set flock [getParticleValue $boid meta]

  set ::nearestEnemy    ""
  set ::nearestEnemyDist 0

  set visible {}

  for {set flock1 0} {$flock1 < $::numFlocks} {incr flock1} {
    if {$flock == $flock1} {
      continue
    }

    foreach boid1 $::flock_boids($flock1) {
      set d [BoidDistance $boid $boid1]

      if {$d < $::Default_Perception_Range} {
        lappend visible $boid1

        if {$::nearestEnemy == "" || $d < $::nearestEnemyDist} {
          set ::nearestEnemy     $boid1
          set ::nearestEnemyDist $d
        }
      }
    }
  }

  return [llength $visible]
}

proc vectorLength { v } {
  set x [lindex $v 0]
  set y [lindex $v 1]
  set z [lindex $v 2]

  return [expr {sqrt($x*$x + $y*$y + $z*$z)}]
}

proc scaleVector { v s } {
  set x [lindex $v 0]
  set y [lindex $v 1]
  set z [lindex $v 2]
  
  return [list [expr {$s*$x}] [expr {$s*$y}] [expr {$s*$z}]]
} 

proc scaleVectorLength { v s } {
  set x [lindex $v 0]
  set y [lindex $v 1]
  set z [lindex $v 2]

  set magnitude [expr {sqrt($x*$x + $y*$y + $z*$z)}]

  set s1 [expr {$s/$magnitude}]

  return [list [expr {$s1*$x}] [expr {$s1*$y}] [expr {$s1*$z}]]
}

proc KeepDistance { boid } {
  if {$::nearestFriend == ""} {
    return [list 0 0 0]
  }

  set v [diffPoint [getParticleValue $::nearestFriend position] [getParticleValue $boid position]]

  set ratio [expr {$::nearestFriendDist/$::SeparationDist}]

  if {$ratio < $::MinUrgency} { set ratio $::MinUrgency }
  if {$ratio > $::MaxUrgency} { set ratio $::MaxUrgency }

  if       {$::nearestFriendDist < $::SeparationDist} {
    set v [scaleVectorLength $v -$ratio]
  } elseif {$::nearestFriendDist > $::SeparationDist} {
    set v [scaleVectorLength $v  $ratio]
  } else {
    set v [list 0 0 0]
  }

  return $v
}

proc MatchHeading { boid } {
  if {$::nearestFriend == ""} {
    return [list 0 0 0]
  }

  set v [getParticleValue $::nearestFriend velocity]

  set v [scaleVectorLength $v $::MinUrgency]

  return $v
}

proc SteerToCenter { boid } {
  set v [diffPoint $::friendCenter [getParticleValue $boid position]]

  set v [scaleVectorLength $v $::MinUrgency]

  return $v
}

proc FleeEnemies { boid } {
  set change [list 0 0 0]

  if {$::nearestEnemy != "" && $::nearestEnemyDist < $::KeepAwayDist} {
    set p1 [getParticleValue $boid position]
    set p2 [getParticleValue $::nearestEnemy position]

    set change [diffPoint $p1 $p2]
  }

  return $change
}   

proc Cruising { boid } {
  set vel [getParticleValue $boid velocity]

  set speed [vectorLength $vel]

  set change $vel

  set diff [expr {($speed - $::DesiredSpeed)/$::MaxSpeed}]

  set urgency [expr {abs($diff)}]

  if {$urgency < $::MinUrgency} { set urgency $::MinUrgency }
  if {$urgency > $::MaxUrgency} { set urgency $::MaxUrgency }

  set jitter [randIn 0 1]

  set xc [lindex $change 0]
  set yc [lindex $change 1]
  set zc [lindex $change 2]

  if       {$jitter < 0.45} {
    set xc [expr {$xc + $::MinUrgency*sign($diff)}]
  } elseif {$jitter < 0.90} {
    set zc [expr {$zc + $::MinUrgency*sign($diff)}]
  } else {
    set yc [expr {$yc + $::MinUrgency*sign($diff)}]
  }

  set change [list $xc $yc $zc]

  scaleVectorLength $change [expr {$urgency*sign($diff)}]

  return $change
}

proc ComputeRPY { boid oldvel } {
  set vel [getParticleValue $boid velocity]

  set lateralDir [crossProduct [crossProduct $vel [diffPoint $vel $oldvel]] $vel]
  
  set lateralDir [normalize $lateralDir]

  set lateralMag [dotProduct [diffPoint $vel_ $oldvel] $lateralDir]

  set roll 0.0

  if {$lateralMag != 0} {
    set roll {[expr -atan2($::GRAVITY, $lateralMag) + $::HALF_PI]}
  }
  
  set vx [lindex $vel 0]
  set vy [lindex $vel 1]
  set vz [lindex $vel 2]

  set pitch [expr {-atan($vy/sqrt(($vz*$vZ) + ($vx*$vx)))}]
  set yaw   [expr {atan2($vx, $vz)}]

  set angle [list $pitch $yaw $roll]

  setParticleValue $boid angle $angle
}

proc WorldBound { boid } {
  set pos [getParticleValue $boid position]

  set x [lindex $pos 0]
  set y [lindex $pos 1]
  set z [lindex $pos 2]

  if {$x >  $::WorldSize} { set x  $::WorldSize }
  if {$x < -$::WorldSize} { set x -$::WorldSize }
  if {$y >  $::WorldSize} { set y  $::WorldSize }
  if {$y < -$::WorldSize} { set y -$::WorldSize }
  if {$z >  $::WorldSize} { set z  $::WorldSize }
  if {$z < -$::WorldSize} { set z -$::WorldSize }

  setParticleValue $boid position [list $x $y $z]
}

proc updateBoid { boid } {
  set dv [list 0 0 0]

  SeeFriends $boid

  set dv [addPoint $dv [KeepDistance  $boid]]
  set dv [addPoint $dv [MatchHeading  $boid]]
  set dv [addPoint $dv [SteerToCenter $boid]]

  if {$::ReactToEnemies} {
    SeeEnemies $boid

    set dv [addPoint $dv [FleeEnemies $boid]]
  }

  set dv [addPoint $dv [Cruising $boid]]

  if {[vectorLength $dv] > $::MaxChange} {
    set dv [scaleVectorLength $dv $::MaxChange]
  }

  set oldvel [getParticleValue $boid velocity]

  set v [addPoint $oldvel $dv]

  if {[vectorLength $v] > $::MaxSpeed} {
    set v [scaleVectorLength $v $::MaxSpeed]
  }

  setParticleValue $boid velocity $v

  # ComputeRPY $boid $oldvel

  WorldBound $boid
}

proc tickProc { args } {
  for {set i 0} {$i < $::numBoids} {incr i} {
    updateBoid $::boids($i)
  }
}
