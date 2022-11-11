# HACKATHON CHALLENGE - DRIVING

When automating the cropping of daffodils, the robot needs to be able to navigate along the row without damaging the crop. The rows may have slight curves or the robot may drift, so simply driving straight forward is not an option. A camera has been mounted on the top of the robot facing forward which can be used to navigate safely. We have provided this library and lengths of artificial grass so that we can model the situation in the field in the comfort of the Launchpad Studio! Use OpenCV (or other machine vision package) to direct the robot while following the 'row'. When you reach the end safely the challenge is complete. Happy Coding!

Artificial grass donated by [Cornwall Artifical Lawns](https://cornwallartificiallawns.co.uk/)

The rows of daffodils in the real life scenario will look like however both these pictures are growing Agapanthus not Daffodil:

![The Rows Of Agapanthus](row1.jpeg)

![A Single Row Of Agapanthus](row2.jpeg)

**Image Here**

 ## SETUP

1) Install [Python 3.11.0](https://www.python.org/downloads/release/python-3110/), the download links will be at the bottom of the page, download 32-bit or 64-bit dependent on your machine.

2) We recommend using [visual studio code](https://code.visualstudio.com/) to manage the code in the project.

3) Install the python extension for Visual Studio Code, this can be done by clicking on the three blocks on the left side bar inside visual studio and searching for python, it will be the first one on the list which has the microsoft blue tick under it.

4) Restart your computer to ensure that python is fully installed and correctly PATHed

 ## Linux Installation

 To install this package you must first clone this directory to a location of your choice, then we will install the package by navigating to that directory and running pip install.

 ```
 cd ~
 git clone https://github.com/Kernow-Robotics/cormoran-driver.git
 cd cormoran-driver
pip install -e .
```

cormoran-driver should now be installed to the default python interpreter on your system. this can be tested by openning a python interpreter and entering `import cormoran`. If the module is imported, installation was a success.

## Windows Installation 
1. Fork the Repo and download it. (use github desktop/fork)

2. Install these in terminal with PIP:
 ```
 pip install --upgrade odrive
 pip install serial
```
3. Configure Odrive(See set-up instructions below)

3. Look at Simple Drive.py for basic controls and instructions.

cormoran-driver should now be installed to the default python interpreter on your system. this can be tested by openning a python interpreter and entering `import cormoran`. If the module is imported, installation was a success.

## Odrive Setup

This can be a complicated procedure so if you get stuck please do not hesitate to ask for help.

[== Odrive Troubleshooting Guide==](https://docs.odriverobotics.com/v/latest/troubleshooting.html#)

1. install [Zadig](https://zadig.akeo.ie/)
2. open Zadig
3. click options>list all devices
4. plug your laptop into the robot under supervision
5. four usb devices should pop up starting with 3.5 and 3.6,select 1 of them check what driver pops up on the left side, if it isnt WInUSB then click replace driver
6. repeat the previous step with the other 3 usb devices.
7. Restart your laptop

## Tips and Commands
> A speed of 0.1 is recommended (same as the example code)

> Angle works in radians

> use computer vision to find the path the robot has to take.

> if the code  does not upload to the robot , use zadig to double check you have the correct drivers

> if you get this error then restart your device.:
> ```
>12:21:42.368436000 [USB] Could not claim interface 0 on USB device: -12
> ```


Kernow Robotics
