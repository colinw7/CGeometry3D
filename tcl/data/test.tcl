set obj [addObject]; puts "Object: $obj"

set v1 [addVertex $obj { 1 -1  1}]; puts "Vertex: $v1"
set v2 [addVertex $obj { 1 -1 -1}]; puts "Vertex: $v2"
set v3 [addVertex $obj { 1  1 -1}]; puts "Vertex: $v3"
set v4 [addVertex $obj { 1  1  1}]; puts "Vertex: $v4"

set v5 [addVertex $obj {-1 -1  1}]; puts "Vertex: $v5"
set v6 [addVertex $obj {-1 -1 -1}]; puts "Vertex: $v6"
set v7 [addVertex $obj {-1  1 -1}]; puts "Vertex: $v7"
set v8 [addVertex $obj {-1  1  1}]; puts "Vertex: $v8"

set f1 [addFace $obj [list $v1 $v2 $v3 $v4]]; puts "Face: $f1"
set f2 [addFace $obj [list $v2 $v6 $v7 $v3]]; puts "Face: $f2"
set f3 [addFace $obj [list $v6 $v5 $v8 $v7]]; puts "Face: $f3"
set f4 [addFace $obj [list $v5 $v1 $v4 $v8]]; puts "Face: $f4"
set f5 [addFace $obj [list $v4 $v3 $v7 $v8]]; puts "Face: $f5"
set f6 [addFace $obj [list $v5 $v6 $v2 $v1]]; puts "Face: $f6"

set m1 [addMaterial red  ]; setMaterialValue $m1 diffuse red  ; puts "Material: $m1"
set m2 [addMaterial green]; setMaterialValue $m2 diffuse green; puts "Material: $m2"
set m3 [addMaterial blue ]; setMaterialValue $m3 diffuse blue ; puts "Material: $m3"

setFaceValue $f1 material $m1
setFaceValue $f2 material $m2
setFaceValue $f3 material $m3
setFaceValue $f4 material $m1
setFaceValue $f5 material $m2
setFaceValue $f6 material $m3

writeObj "test.obj"
