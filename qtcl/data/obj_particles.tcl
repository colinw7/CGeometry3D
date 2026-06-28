#set obj [readModel "models/v3d/F15.V3D"]

set obj [addSphere 1]

set m1 [addMaterial red]
set t1 [addTexture "data/moon_texture.jpg"]

setMaterialValue $m1 diffuse_texture $t1
setObjectValue $obj material $m1

echo [getObjectValue $obj bbox]

set c [getObjectValue $obj center]
setObjectValue $obj translate [list -[lindex $c 0] -[lindex $c 1] -[lindex $c 2]]
echo [getObjectValue $obj bbox]
setObjectValue $obj visible 0

#setObjectValue $obj scale [list 0.1 0.1 0.1]

setAppValue gravity 0

setAppValue bbox [list -10 -10 -10 10 10 10]

proc randIn { min max } {
  return [expr {rand()*($max - $min) + $min}]
}

proc injectParticle { i } {
  set obj1 [getObjectValue $::obj ref_object]

  setObjectValue $obj1 visible 1

  #set s [randIn 0.5 2.0]
  #setObjectValue $obj1 scale $s

  set p [addParticle [list 0 0 0] 1 $obj1]

  set vx [randIn -4 4]
  set vy [randIn -4 4]
  set vz [randIn -4 4]

  setParticleValue $p velocity [list $vx $vy $vz]
  setParticleValue $p color    [list 1 0 0]
  setParticleValue $p size     0.03

  set ::particles($i) $p
}

set np 100

for {set i 0} {$i < $np} {incr i} {
  injectParticle $i
}

proc tickProc { args } {
  #set pos [getParticleValue $::particles(0) position]
  #echo $pos
}
