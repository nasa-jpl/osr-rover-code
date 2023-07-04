# voltage level being read into the RoboClaws, in Volts
float32 battery
# status of an error being present on each RoboClaw. Stringified hex
str[5] error_status
# Temperature of the motor controllers expressed in Celsius
float32[5] temp
# Sensed current from the motor controllers in Ampere
float32[10] current
