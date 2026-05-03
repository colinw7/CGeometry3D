set cube [addCube 1]

set edges [getObjectValue $cube edges]

foreach edge $edges {
  set start [getEdgeValue $edge start]
  set end   [getEdgeValue $edge end]

  echo "$edge: $start -> $end"
}
