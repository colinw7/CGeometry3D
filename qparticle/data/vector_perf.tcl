proc randIn { min max } {
  return [expr {rand()*($max - $min) + $min}]
}

proc printVector1 { v } {
  echo "[lindex $v 0] [lindex $v 1] [lindex $v 1]"
}

proc printVector2 { v } {
  echo "[get_vector $v x] [get_vector $v y] [get_vector $v z]"
}

set n 10000

set vectors1 {}
set vectors2 {}

for {set i 0} {$i < $n} {incr i} {
  set x [randIn -1 1]
  set y [randIn -1 1]
  set z [randIn -1 1]

  lappend vectors1 [list   $x $y $z]
  lappend vectors2 [vector $x $y $z]
}

if {0} {
foreach v $vectors1 {
  printVector1 $v
}

foreach v $vectors2 {
  printVector2 $v
}
}

proc addVector1 { v1 v2 } {
  set x1 [lindex $v1 0]
  set y1 [lindex $v1 1]
  set z1 [lindex $v1 2]
  
  set x2 [lindex $v2 0]
  set y2 [lindex $v2 1]
  set z2 [lindex $v2 2]
  
  return [list [expr {$x1 + $x2}] [expr {$y1 + $y2}] [expr {$z1 + $z2}]]
} 

proc sum1 { } {
  set res [list 0 0 0]

  foreach v $::vectors1 {
    set res [addVector1 $res $v]
  }

  echo "$res"

  return $res
}

proc sum2 { } {
  set res [vector 0 0 0]

  foreach v $::vectors2 {
    set_vector $res + $v
  }

  echo "$res"

  return $res
}

echo [time sum1]
echo [time sum2]

echo [addVector1 [list   1 2 3] [list   3 4 5]]
echo [set_vector [vector 1 2 3] + [vector 3 4 5]]
