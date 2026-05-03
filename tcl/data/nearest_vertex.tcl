set cube [addCube 1]

set vertices [getObjectValue $cube vertices]

echo "vertices: $vertices"

set vertex1 [getObjectValue $cube nearest_vertex {-1  1 -1}]
set vertex2 [getObjectValue $cube nearest_vertex { 1  1 -1}]
set vertex3 [getObjectValue $cube nearest_vertex { 1  1  1}]
set vertex4 [getObjectValue $cube nearest_vertex {-1  1  1}]
set vertex5 [getObjectValue $cube nearest_vertex {-1 -1 -1}]
set vertex6 [getObjectValue $cube nearest_vertex { 1 -1 -1}]
set vertex7 [getObjectValue $cube nearest_vertex { 1 -1  1}]
set vertex8 [getObjectValue $cube nearest_vertex {-1 -1  1}]

echo "$vertex1 $vertex2 $vertex3 $vertex4 $vertex5 $vertex6 $vertex7 $vertex8"
