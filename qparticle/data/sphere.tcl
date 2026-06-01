set obj [addSphere 1]

set m1 [addMaterial red]
set t1 [addTexture "data/moon_texture.jpg"]

#setMaterialValue $m1 diffuse red
setMaterialValue $m1 diffuse_texture $t1

setObjectValue $obj material $m1

writeObj "test.obj"
