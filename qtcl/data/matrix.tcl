proc printMatrix { msg m } {
  #echo "[get_matrix $v x] [get_matrix $v y] [get_matrix $v z]"
  echo "$msg: $m"
}

set m [matrix [list 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15]]
printMatrix "matrix" $m

set m1 [matrix]
set m2 [matrix]
set m3 [matrix]

set_matrix $m1 translation [vector 1 2 3]
printMatrix "translation" $m1

set_matrix $m2 scale [vector 1 2 3]
printMatrix "scale" $m2

set_matrix $m3 rotation [vector 0 0 1] 30
printMatrix "rotation" $m3

set mm [matrix "identity"]
printMatrix "matrix" $mm

set mm [set_matrix $mm multiply $m1]
set mm [set_matrix $mm multiply $m2]
set mm [set_matrix $mm multiply $m3]
printMatrix "m1*m2*m3" $mm

set mi [set_matrix $mm inverse]
printMatrix "inverse" $mm

set m4 [set_matrix $mm multiply $mi]
printMatrix "identity" $m4

#set_matrix $m + [matrix 1 2 3]
#printMatrix $m

#set_matrix $m - [matrix 1 2 3]
#printMatrix $m

#set_matrix $m * 0.1
#printMatrix $m
