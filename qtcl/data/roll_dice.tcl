proc randIn { min max } {
  return [expr {rand()*($max - $min) + $min}]
}

set dice [readModel "data/dice.obj"]

setObjectValue $dice translate [list 0 -1 0]

echo [getObjectValue $dice bbox]

set x_angle [randIn -10 10]
set y_angle [randIn -10 10]
set z_angle [randIn -10 10]

proc tickProc { args } {
  setObjectValue $::dice rotate [list 1 0 0] $::x_angle
  setObjectValue $::dice rotate [list 0 1 0] $::y_angle
  setObjectValue $::dice rotate [list 0 0 1] $::z_angle
}
