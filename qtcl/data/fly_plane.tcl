source "data/showNormals.tcl"

set plane [readModel "models/v3d/F15.V3D"]

set terrain [addTerrain 10 10 5]

set m1 [addMaterial red]
set t1 [addTexture "data/moon_texture.jpg"]

setMaterialValue $m1 diffuse_texture $t1

setObjectValue $terrain translate [list -50 0 -50]
setObjectValue $plane   translate [list 0 35 0]

setObjectValue $terrain material $m1

setAppValue pitch           90
setAppValue camera_distance 75

showNormals

proc tickProc { args } {
}

proc keyPress { args } {
  set key   [lindex $args 0]
  set ctrl  [lindex $args 1]
  set shift [lindex $args 2]
  
  echo "keyPress ($key) ($ctrl) ($shift)"

  if       {$key == "a" || $key == "A"} {
    setObjectValue $::plane translate [list -1 0 0]
  } elseif {$key == "d" || $key == "D"} {
    setObjectValue $::plane translate [list 1 0 0]
  } elseif {$key == "w" || $key == "W"} {
    setObjectValue $::plane translate [list 0 0 -1]
  } elseif {$key == "s" || $key == "S"} {
    setObjectValue $::plane translate [list 0 0 1]
  } elseif {$key == "q" || $key == "Q"} {
    setObjectValue $::plane rotate [list 0 0 1] 1
  } elseif {$key == "e" || $key == "E"} {
    setObjectValue $::plane rotate [list 0 0 1] -1
  } elseif {$key == "up"} {
    setObjectValue $::plane translate [list 0 -1 0]
  } elseif {$key == "down"} {
    setObjectValue $::plane translate [list 0 1 0]
  }
}
