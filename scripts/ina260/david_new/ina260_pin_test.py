import INA260
import RPi.GPIO as gpio

pm = INA260.INA260(INA260.INA260_I2CADDR_DEFAULT)

gpio.setmode(gpio.BCM)
gpio.setup(27, gpio.OUT)
gpio.output(27, 0)


