# voltage level being read into the RoboClaws. The value is 10 x [Battery Voltage(V)]
int64     battery
# status of an error being present on each RoboClaw. 1 if Error, 0 otherwise
int64[5]  error_status
# Temperature of the motor controllers. The value is 10 x (Temperature[C])
int64[5]  temp
# Sensed current from the motor controllers. current drawn on each motor. The value is 10 x (Current(A)
int64[10] current
