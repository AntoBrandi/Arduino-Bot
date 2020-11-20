<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<p align="center">
   <img src="images/logo.jpg" alt="Logo" width="200" height="200">

  <h3 align="center">arduinobot</h3>

  <p align="center">
    3D printed robot arm powered by ROS and Arduino Uno
    <br />
    <a href="https://github.com/AntoBrandi/arduinobot/"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/AntoBrandi/arduinobot/">View Demo</a>
    ·
    <a href="https://github.com/AntoBrandi/arduinobot/issues">Report Bug</a>
    ·
    <a href="https://github.com/AntoBrandi/arduinobot/issues">Request Feature</a>
  </p>
</p>

[![Product Name Screen Shot][product-screenshot]](https://example.com)

<!-- TABLE OF CONTENTS -->
## Table of Contents

* [About the Project](#about-the-project)
  * [Built With](#built-with)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installation](#installation)
* [Usage](#usage)
* [Contributing](#contributing)
* [License](#license)
* [Contact](#contact)
* [Acknowledgements](#acknowledgements)



<!-- ABOUT THE PROJECT -->
## About The Project

[![Product Name Screen Shot 2][product-screenshot-2]](https://example.com)

This project aim is to build a 3 axis robot arm with simple and cheap hardware that can be easily and remotely controlled.
Despite all the other robot arms built with an Aruidno Uno that are controlled by applying joint angles, this can be controlled assigning X, Y, Z coordinates in the workspace because it implements the inverse kinamatic.
Furthermore, it implements a speech recognition module that can run on any computer with a microphone that is connected to the same Wi-Fi network 
of the robot.

### Built With
This robot is powered by:
* Raspberry Pi 3/4 or PC with Ubuntu 18.04
* Arduino UNO
* SG90 Servo Motors (x4)
* Prototype Expansion Board or Breadboard

And is controlled by:
* ROS Melodic

[![Product Name Screen Shot Real][product-screenshot-real]](https://example.com)


<!-- GETTING STARTED -->
## Getting Started

Once it's printed and assembled according and connected, there are few configuration to be made in both Raspberry Pi side and Arduino UNO.

### Prerequisites

Make sure you install correctly the following required tools before continuing
* Install Ubuntu 18.04 on Raspberry Pi 3/4 or on PC
Use the [Ubuntu Raspberry Pi](https://ubuntu.com/download/raspberry-pi) for your Raspberry Pi or [Ubuntu 18.04](https://ubuntu.com/download/alternative-downloads) for your PC
* Install [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) on your Ubuntu 18.04
* Install ROS missing libraries. Some libraries that are used in this project are not in the standard ROS package. Install them with:
```sh
sudo apt-get install ros-kinetic-rosserial
```
```sh
sudo apt-get install ros-kinetic-rosserial-arduino
```
```sh
sudo apt-get install ros-kinetic-rosbridge-server
```
```sh
sudo apt-get install ros-kinetic-moveit
```
* Install VS Code and Arduino IDE on your PC in order to build and load the Arduino code on the device


### Installation

1. Clone the repo
```sh
git clone https://github.com/AntoBrandi/arduinobot.git
```
2. Build the ROS workspace
```sh
cd ~/arduinobot/arduinobot_ws
```
```sh
catkin_make
```
3. Source the project
```sh
sudo nano ~/.bashrc
```
And add the following at the end of the document
```sh
source ~/arduinobot/arduinobot_ws/devel/setup.bash
```
4. Connect the Arduino UNO on your PC with VS Code and Arduino IDE. Open the [folder](https://github.com/AntoBrandi/arduinobot/tree/master/arduino)
containing the code for the Arduino controller.



<!-- USAGE EXAMPLES -->
## Usage

To launch the ROS simulated robot
```sh
roslaunch arduinobot_bringup sim_complete.launch
```

To launch the real robot, connect the Arduino to the PC and upload the code in the [folder](https://github.com/AntoBrandi/arduinobot/tree/master/arduino/ros_robot_control) on the Arduino controller.
Then launch the real robot
```sh
roslaunch arduinobot_bringup complete.launch
```




<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.



<!-- CONTACT -->
## Contact

Antonio Brandi - [LinkedIn]([linkedin-url]) - antonio.brandi@outlook.it

My Projects: [https://github.com/AntoBrandi](https://github.com/AntoBrandi)



<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements
* [Arduino 3D-Printed Robotic Arm](https://create.arduino.cc/projecthub/mircemk/arduino-3d-printed-robotic-arm-e824d8?ref=search&ref_id=robot%20arm&offset=86)
* [EEZYbotARM](https://www.thingiverse.com/thing:1015238)





<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/antonio-brandi-512166bb/
[product-screenshot]: images/BB3A0020.jpg
[product-screenshot-2]: images/BB3A0026.jpg
[product-screenshot-real]: images/screen_video.png
