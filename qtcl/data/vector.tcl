proc printVector { v } {
  echo "[get_vector $v x] [get_vector $v y] [get_vector $v z]"
}

set v [vector [list 1 2 3]]
printVector $v

set_vector $v x 3
set_vector $v y 1
set_vector $v z 2

printVector $v

set_vector $v + [vector 1 2 3]
printVector $v

set_vector $v - [vector 1 2 3]
printVector $v

set_vector $v * 0.1
printVector $v
