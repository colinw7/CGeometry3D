proc createFacelet { pos x y z } {
  set s 0.95

  set cube [addCube 1]

  set d 0.5
  set t 0.1

  set xt [expr {[lindex $pos 0] + $d*$x}]
  set yt [expr {[lindex $pos 1] + $d*$y}]
  set zt [expr {[lindex $pos 2] + $d*$z}]

  set xs [expr {(abs($x)*$t + (1 - abs($x)))*$s}]
  set ys [expr {(abs($y)*$t + (1 - abs($y)))*$s}]
  set zs [expr {(abs($z)*$t + (1 - abs($z)))*$s}]

  setObjectValue $cube scale     [list $xs $ys $zs]
  setObjectValue $cube translate [list $xt $yt $zt]

  set color [expr {$x + 2*$y + 3*$z}]
  if {$color < 0} {
    set color [expr {$color + 3}]
  } else {
    set color [expr {$color + 2}]
  }
  #echo $color

  if       {$color == 0} {
    setObjectValue $cube material $::m1
  } elseif {$color == 1} {
    setObjectValue $cube material $::m2
  } elseif {$color == 2} {
    setObjectValue $cube material $::m3
  } elseif {$color == 3} {
    setObjectValue $cube material $::m4
  } elseif {$color == 4} {
    setObjectValue $cube material $::m5
  } elseif {$color == 5} {
    setObjectValue $cube material $::m6
  }
}

proc createCube { } {
  set s 0.99

  for {set iz -1} {$iz <= 1} {incr iz} {
    for {set iy -1} {$iy <= 1} {incr iy} {
      for {set ix -1} {$ix <= 1} {incr ix} {
        if {$ix == 0 && $iy == 0 && $iz == 0} {
          continue
        }

        set cube [addCube 1]

        set pos [list $ix $iy $iz]

        setObjectValue $cube scale     [list $s $s $s]
        setObjectValue $cube translate $pos

        setObjectValue $cube material $::m0

        if {$ix != 0} { createFacelet $pos $ix 0 0 }
        if {$iy != 0} { createFacelet $pos 0 $iy 0 }
        if {$iz != 0} { createFacelet $pos 0 0 $iz }
      }
    }
  }
}

set m0 [addMaterial black ]; setMaterialValue $m0 diffuse black ; #puts "Material: $m0"
set m1 [addMaterial red   ]; setMaterialValue $m1 diffuse red   ; #puts "Material: $m1"
set m2 [addMaterial green ]; setMaterialValue $m2 diffuse green ; #puts "Material: $m2"
set m3 [addMaterial blue  ]; setMaterialValue $m3 diffuse blue  ; #puts "Material: $m3"
set m4 [addMaterial yellow]; setMaterialValue $m4 diffuse yellow; #puts "Material: $m4"
set m5 [addMaterial orange]; setMaterialValue $m5 diffuse orange; #puts "Material: $m5"
set m6 [addMaterial white ]; setMaterialValue $m6 diffuse white ; #puts "Material: $m6"

createCube

writeObj "rubik.obj"
