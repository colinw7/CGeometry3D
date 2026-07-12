proc printMatrix { m } {
  #echo "[get_matrix $v x] [get_matrix $v y] [get_matrix $v z]"
  echo "$m"
}

set m [matrix [list 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15]]
printMatrix $m

set m [matrix]
set_matrix $m translation [vector 1 2 3]
printMatrix $m

#set_matrix $m + [matrix 1 2 3]
#printMatrix $m

#set_matrix $m - [matrix 1 2 3]
#printMatrix $m

#set_matrix $m * 0.1
#printMatrix $m
