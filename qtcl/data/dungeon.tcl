proc loadModel { filename name { s 1.0 } } {
  set obj [readModel $filename]

  setObjectValue $obj name    $name
  setObjectValue $obj visible 0

  if {$s != 1.0} {
    setObjectValue $obj scale [list $s $s $s] set
  }

  return $obj
}

proc mapPos { pos } {
  set x [lindex $pos 0]
  set y [lindex $pos 1]
  set z [lindex $pos 2]

  set x1 [expr {$x*$::tileDx - $::mapDx}]
  set y1 [expr {$y*$::tileDy - $::mapDy}]
  set z1 [expr {$z*$::tileDz - $::mapDz}]

  return [list $x1 $y1 $z1]
}

proc addTiles { model } {
  echo "addTiles $model"

  for {set iy 0} {$iy < $::ny} {incr iy} {
    for {set ix 0} {$ix < $::nx} {incr ix} {
      set tile($ix,$iy) [getObjectValue $model ref_object]

      set pos [mapPos [list $ix 0 $iy]]

      setObjectValue $tile($ix,$iy) translate $pos set

      setObjectValue $tile($ix,$iy) visible 1

      # echo [getObjectValue $tile($ix,$iy) bbox]
    }
  }
}

proc addWalls { wallObj cornerObj } {
  echo "addWalls $wallObj $cornerObj"

  set ix1 [expr {$::nx - 1}]
  set iy1 [expr {$::ny - 1}]

  for {set iy 0} {$iy < $::ny} {incr iy} {
    set tb [expr {$iy == 0 || $iy == $iy1}]

    for {set ix 0} {$ix < $::nx} {incr ix} {
      set lr [expr {$ix == 0 || $ix == $ix1}]
      if {! $lr && ! $tb} { continue }

      if {$lr && $tb} {
        set tile($ix,$iy) [getObjectValue $cornerObj ref_object]

        if       {$ix == 0 && $iy == 0} {
          setObjectValue $tile($ix,$iy) rotate [list 0 1 0] 90 set
        } elseif {$ix == 0 && $iy == $iy1} {
          setObjectValue $tile($ix,$iy) rotate [list 0 1 0] 180 set
        } elseif {$ix == $ix1 && $iy == 0} {
          setObjectValue $tile($ix,$iy) rotate [list 0 1 0] 0 set
        } elseif {$ix == $ix1 && $iy == $iy1} {
          setObjectValue $tile($ix,$iy) rotate [list 0 1 0] 270 set
        }
      } else {
        set tile($ix,$iy) [getObjectValue $wallObj ref_object]

        if {$lr} {
          setObjectValue $tile($ix,$iy) rotate [list 0 1 0] 90 set
        }
      }

      set pos [mapPos [list $ix 0 $iy]]

      setObjectValue $tile($ix,$iy) translate $pos set

      setObjectValue $tile($ix,$iy) visible 1

      # echo [getObjectValue $tile($ix,$iy) bbox]
    }
  }
}

proc addObject { model ind pos } {
  set ::obj($ind) [getObjectValue $model ref_object]

  set pos [mapPos $pos]

  setObjectValue $::obj($ind) translate $pos set

  setObjectValue $::obj($ind) visible 1

  # echo [getObjectValue $::obj($ind) bbox]
}

set ::cameraSet 0

set ::tileDx 4.1
set ::tileDy 0
set ::tileDz 4.1

addModelDir "data/Dungeon_Assets/obj"

if {0} {
loadModel "data/Dungeon_Assets/obj/floor_dirt_large.obj"                  "floor"]
loadModel "data/Dungeon_Assets/obj/floor_dirt_large_rocky.obj"            "floor"]
loadModel "data/Dungeon_Assets/obj/floor_tile_extralarge_grates.obj"      "floor"]
loadModel "data/Dungeon_Assets/obj/floor_tile_extralarge_grates_open.obj" "floor"]
loadModel "data/Dungeon_Assets/obj/floor_tile_large.obj"                  "floor"]
loadModel "data/Dungeon_Assets/obj/floor_tile_large_rocks.obj"            "floor"]
loadModel "data/Dungeon_Assets/obj/floor_wood_large_dark.obj"             "floor"]
loadModel "data/Dungeon_Assets/obj/floor_wood_large.obj"                  "floor"]
}

set tileObj       [loadModel "data/Dungeon_Assets/obj/floor_wood_large.obj"       "tile"  ]
set barrelObj     [loadModel "data/Dungeon_Assets/obj/barrel_large_decorated.obj" "barrel"]
set chest1Obj     [loadModel "data/Dungeon_Assets/obj/chest_mimic.obj"            "chest1"]
set chest2Obj     [loadModel "data/Dungeon_Assets/obj/chest_mimic_lid.obj"        "chest2"]
set wallObj       [loadModel "data/Dungeon_Assets/obj/wall.obj"                   "wall"]
set wallPillarObj [loadModel "data/Dungeon_Assets/obj/wall_pillar.obj"            "wall_pillar"]
set wallCornerObj [loadModel "data/Dungeon_Assets/obj/wall_corner.obj"            "wall_corner"]

set ::nx 10
set ::ny 10

set ::mapDx [expr {$::nx*$::tileDx/2.0}]
set ::mapDy 0
set ::mapDz [expr {$::ny*$::tileDz/2.0}]

addTiles $tileObj
addWalls $wallObj $wallCornerObj

set ind 0

addObject $barrelObj $ind [list 4 0 4] ; incr ind

addObject $chest1Obj $ind [list 5 0 5] ; incr ind
addObject $chest2Obj $ind [list 5 1 5] ; incr ind

setViewportValue "" bbox [list -10 -10 -10 10 10 10]

proc tickProc { args } {
  if {! $::cameraSet} {
    setViewportValue "" camera.distance 20

    setViewportValue "" camera.origin.x 0
    setViewportValue "" camera.origin.y 0
    setViewportValue "" camera.origin.z 0

    setViewportValue "" camera.position.x 0
    setViewportValue "" camera.position.y 10
    setViewportValue "" camera.position.z 20

    set ::cameraSet 1
  }
}
