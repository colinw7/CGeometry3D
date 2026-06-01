setAppValue cursor {0 0 0}

set plane [addPlane 6 4]

setAppValue cursor {1 1 1}

set plane [addPlane 4 6]

writeObj "cursor.obj"
