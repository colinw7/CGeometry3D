#setAppValue bbox [list -2 -2 -2 2 2 2]

set cube [addCube 1]

set vertices [getObjectValue $cube vertices]
echo "vertices: $vertices"

foreach v $vertices {
  set pos [getVertexValue $v model]

  set text [addText $v $pos 0.1]

  #setTextValue $text overlay 1
  setTextValue $text billboard 1
  setTextValue $text color     red

  set str  [getTextValue $text text]
  set pos  [getTextValue $text position]
  set size [getTextValue $text size]

  echo "text: $text '$str' $pos $size"
}
