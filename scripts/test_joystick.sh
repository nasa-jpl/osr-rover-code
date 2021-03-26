# script to compile and run test_joystick.c
# if you have a joystick attached while running it, it will print out actions from the joystick

# compile it, if it hasn't been yet
if [ ! -e "joystick" ]; then
    gcc test_joystick.c -o joystick
fi

./joystick