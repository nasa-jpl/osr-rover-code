# Flashing the Arduino code

In this section we will be flashing the code that runs on the arduino to control the LED matrix in the head. The following steps should be performed on your laptop or development machine (not the raspberry pi)

1. Install the [Arduino IDE](https://www.arduino.cc/en/Main/Software) used for loading code onto the arduino
2. Download the Arduino code:
    1. Navigate to [the repo](https://github.com/nasa-jpl/osr-rover-code) and click the green "Clone or download"
button. Choose "Download ZIP".
    2. Unzip/extract and open the downloaded zip file. Then, select the Arduino folder and create a
new zip file of just that Arduino folder. Name it OsrScreen.zip
3. Load the sketch onto the Arduino
    1. Unplug the Arduino shield JST cable so the Arduino isn’t powered by the control board
    2. Connect the Arduino to your development machine with USB cable
    3. Open Arduino IDE
    4. Select Sketch -> Include Library -> Add .Zip Library
    5. Select the OsrScreen.zip folder created previously
    6. Click the Upload button in the Sketch Window
4. To load the example in the Arduino IDE: File -> Examples -> OsrScreen -> OsrScreen
