Configuration can be a bit of a pain for using the serial port on the raspberry pi with ubuntu.

For ubuntu 20.04, we found that using the previous OSR default port (`/dev/serial0`) for serial communication with the roboclaw motor drivers did not perform as expected - even though it worked just fine under ubuntu 18.04.

Important things to know:
- `/dev/serial0` maps to `/dev/ttyS0`, which handles serial communication through a software "mini-UART" implementation.
- `/dev/serial1` maps to `/dev/ttyAMA0`, which handles serial communication through a hardware PL011 UART implementation.
- The PL011 UART (UART0) is a full-featured universal asynchronous receiver/transmitter for serial communication
- the mini-UART (UART1) has a reduced set of features
- On the RPi4, the PL011 UART is also connected to the bluetooth controller.
<!-- - On the RPi4, the mini-UART is primary, and the PL011 UART is secondary -->

Check out [this discussion](https://www.engineersgarage.com/microcontroller-projects/articles-raspberry-pi-serial-communication-uart-protocol-serial-linux-devices/) and [this discussion](https://spellfoundry.com/2016/05/29/configuring-gpio-serial-port-raspbian-jessie-including-pi-3-4/) for more information.

The upshot of all of this is:
- when using `/dev/serial0` for serial communications instead of `/dev/serial1`, you're using a less capable communications device. For whatever reason, this worked under ubuntu 18.04, but but seems to have a lot of trouble under ubuntu 20.04. So it's best to use `/dev/serial1`.
- However, `/dev/serial1` is also connected to the bluetooth controller, so we need to disable bluetooth. This is why we use the `dtoverlay=disable-bt` overlay in config.txt (see [rpi.md](rpi.md))
- We also need to remove `console=serial0,115200` from cmdline.txt, because it also seems to prevent the operation of `serial1` for serial comms.