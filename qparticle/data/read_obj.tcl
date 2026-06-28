set obj1 [readModel "models/v3d/F15.V3D"]
set obj2 [readModel "models/v3d/F15.V3D"]

setObjectValue $obj1 translate { 10 0 0}
setObjectValue $obj2 translate {-10 0 0}

writeObj "read_obj.obj"
