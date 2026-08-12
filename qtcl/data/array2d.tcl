proc printArray2D { msg m } {
  echo "$msg: $m"
}

set m [array2D {{0 1 2 3} {4 5 6 7} {8 9 10 11} {12 13 14 15}}]
printArray2D "array" $m

set m1 [array2D]
