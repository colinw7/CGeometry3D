set obj1 [readObj "/home/colinw/dev/models/v3d/F15.V3D"]
set obj2 [readObj "/home/colinw/dev/models/v3d/F15.V3D"]

setObjectValue $obj1 translate { 1 0 0}
setObjectValue $obj2 translate {-1 0 0}

writeObj "read_obj.obj"
