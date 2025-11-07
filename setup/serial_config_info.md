Configuration can be a bit of a pain for using the serial port on the raspberry pi with ubuntu. 

# On the Pi3/Pi4

Difference between `/dev/serial0` and `/dev/serial1`:
- `/dev/serial0` maps to `/dev/ttyS0`, which handles serial communication through a software "mini-UART" implementation.  Connected to GPIO 14/15 by default.  If `dtoverlay=disable-bt` is set, this is not connected.
- The mini-UART (UART1) has a reduced set of features
- `/dev/serial1` maps to `/dev/ttyAMA0`, which handles serial communication through a hardware PL011 UART implementation.  Connected to Bluetooth by default.  If `dtoverlay=disable-bt` is set, connected to GPIO 14/15.
- The PL011 UART (UART0) is a full-featured universal asynchronous receiver/transmitter for serial communication

If you are using a Raspberry Pi 3/4, add the following lines to `/boot/firmware/config.txt`:
```
 enable_uart=1
 dtoverlay=disable-bt
```
This will disconnect `/dev/ttyS0` (and `/dev/serial0`) from the mini-UART, and more importantly, connect `/dev/ttyAMA0` (and `/dev/serial1`) to the hardware UART on GPIO 14/15 that the roboclaws are using.

See [this discussion](https://www.engineersgarage.com/microcontroller-projects/articles-raspberry-pi-serial-communication-uart-protocol-serial-linux-devices/) and [this discussion](https://spellfoundry.com/2016/05/29/configuring-gpio-serial-port-raspbian-jessie-including-pi-3-4/) for more information.

- when using `/dev/serial0` for serial communications instead of `/dev/serial1`, you're using a less capable communications device. For whatever reason, this worked under ubuntu 18.04, but but seems to have a lot of trouble under ubuntu 20.04. So it's best to use `/dev/serial1`.

- However, `/dev/serial1` is also connected to the bluetooth controller, so we need to disable bluetooth. 

- We also need to remove `console=serial0,115200` from cmdline.txt, because it also seems to prevent the operation of `serial1` for serial comms.

To find out which one you should be using, run [the test script](../scripts/roboclawtest.py). If you want to use `serial0` instead of `serial1`, you must change
the default in the [config yaml](../osr_bringup/config/roboclaw_params.yaml).

# On the Pi5

Difference between `/dev/serial0` and `/dev/serial1`:
- `/dev/serial0` maps to `/dev/ttyS0`, which handles serial communication through a hardware PL011 UART implementation.  Connected to GPIO 14/15 by default.  If `dtoverlay=disable-bt` is set, this is routed to the debug UART between HDMI0 and HDMI1.
- `/dev/serial1` maps to `/dev/ttyAMA0`, which handles serial communication through a hardware PL011 UART implementation.  Connected to Bluetooth by default.  If `dtoverlay=disable-bt` is set, connected to GPIO 14/15.

If you are using a Raspberry Pi 5, add the following lines to `/boot/firmware/config.txt`:
```
 enable_uart=1
 dtoverlay=disable-bt
 dtoverlay=uart0
```
This will connect `/dev/ttyS0` (and `/dev/serial0`) to the debug UART port, and more importantly, connect `/dev/ttyAMA0` (and `/dev/serial1`) to the hardware UART on GPIO 14/15 that the roboclaws are using.
