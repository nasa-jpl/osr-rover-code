# voltage level being read into the RoboClaws, in Volts
float32 battery
# status of an error being present on each RoboClaw. 1 if Error, 0 otherwise
int32[5] error_status
# Temperature of the motor controllers expressed in Celsius
float32[5] temp
# Sensed current from the motor controllers in Ampere
float32[10] current
