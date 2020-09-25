# Using Raspbian on the RPi instead of Ubuntu

**These instructions are up-to-date as of 8/17/2020, but will not be regularly updated**

## 1 Intalling Raspbian

To install Raspbian, we recommend following the "Getting started with Raspberry Pi" tutorial below. 

**NOTE: These instructions were tested and verified using Raspbian Stretch. Newer versions of Raspbian (such as Buster) may cause complications with these instructions. We therefore recommend installing Raspbian Stretch instead of the latest/default version of Raspbian, which the tutorial below will recommend. Do not flash NOOBS onto your card; instead download the Raspbian Streth image below and skip the NOOBS step in the tutorial.** 

In addition, we have created a pre-built image with both Raspbian Stretch and ROS already installed. You can download this image as a last resort if you can’t get Raspbian and ROS installed and working on your own, but please note that you will miss out on some the learning opportunities that come with debugging a software installation by doing so. 

- [Tutorial](https://projects.raspberrypi.org/en/projects/raspberry-pi-getting-started) for installing the latest version of Raspbian on a new Raspberry Pi
- Direct downloadable Raspbian Stretch [image](https://downloads.raspberrypi.org/raspbian/images/raspbian-2019-04-09/2019-04-08-raspbian-stretch.zip)
- Custom JPL [image](https://drive.google.com/drive/folders/1RbYWDRthpcktqkPEHb7qVqDiy27EiAc1) with both Raspbian Stretch and ROS pre-built

**NOTE: If you install the Custom JPL image with Raspbian Stretch and ROS pre-built, skip to step 2.4.**

Once you finish the above tutorial and install Raspbian Stretch, you should be able to boot your Raspberry Pi and see a desktop!





## 2 Installing ROS Kinetic on Raspbian

Use these instructions if you followed subsection 2.1.2 above.

If you do not download the pre-built Raspbian Stretch / ROS image, you will need to build ROS yourself.

First, make sure you update all the packages on your pi:

```
sudo apt-get update
sudo apt-get upgrade
```

(The above commands may take a few minutes to run.)

Next, we will install ROS (and specifically, the version called ’Kinetic’).

**NOTE: Follow the ROS installation directions in the link below CAREFULLY! Do not ignore warnings, as they will lead to issues later in the installation. Some known ”gotchas” to note before you get started are:**

- Make sure you install the Kinetic ”desktop full” version of ROS, not the desktop ros-comm, or other versions.
- ROS changed their GPG keys as of June 7 2019. The ROS install instructions do not mention this as of July 2019 and will not work unless you update your GPG keys. See [here](https://discourse.ros.org/t/new-gpg-keys-deployed-for-packages-ros-org/9454) for instructions on how to update your GPG keys.
- Make sure you follow directions for KINETIC, not Indigo, Melodic, or any other ROS version.

Once you familiarize yourself with the above issues, go ahead and complete your ROS installation by following these [instructions](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi)

Refer to the [ubuntu setup guide](rpi.md#3-setting-up-ros-environment-and-building-the-rover-code) for further instructions on setting up the ROS environment and building the rover code.

## 3 Setting up serial communication on the RPi under Raspbian

Run the following commands on the Pi to setup/enable the serial communication for the RoboClaws:  ```sudo raspi-config```

In the raspi-config menu, set the following options:

- Interface Options −> Serial
- Would you like a login shell to be accessible over serial? −> No
- Would you like the serial port hardware to be enabled? −> Yes
- Would you like to reboot now? −> Yes


Once the Pi reboots, open up a terminal again and look at the serial devices:
```
ls -l /dev/serial*
```

Make sure that this shows `serial0` -> `ttyS0` . If it does not, ensure that you have followed every step in this tutorial in order. Next, edit the `/boot/cmdline.txt` file:
```
sudo nano /boot/cmdline.txt
```

Change ONLY the part with `console = ...` to read `console=tty1` and remove any other instance where it references console. The first bit of that line should look similar to the below:
```
dwc_otg.lpm_enable=0 console=tty1 root=/dev/mmcblk0p7
```

See [here](https://spellfoundry.com/2016/05/29/configuring-gpio-serial-port-raspbian-jessie-including-pi-3/) if you need additional help with above steps.

Once you’ve completed the above steps, reboot the Pi.
```
sudo reboot
```

Refer to the [ubuntu setup guide](rpi.md#5-testing-serial-comm-with-the-roboclaw-motors-controllers) for further instructions on testing serial comms with the roboclaw controllers.