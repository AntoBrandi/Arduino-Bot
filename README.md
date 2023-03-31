[![LinkedIn][linkedin-shield]][linkedin-url]
[![Udemy][udemy-shield]][udemy-url]
[![Skillshare][skillshare-shield]][skillshare-url]


<!-- PROJECT LOGO -->
<br />
<p align="center">
   <img src="images/logo.png" alt="Logo">

  <h3 align="center">Arduinobot</h3>

  <p align="center">
    3D Printed robot arm powered by ROS and Arduino and controlled via MoveIt! and Amazon Alexa.
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

This Robot is Build and Developed in the online course called: **Robotics and ROS - Learn by Doing! Manipulators** and is currently available on the following platforms:

* [Udemy](https://www.udemy.com/course/robotics-and-ros-learn-by-doing-manipulators/?couponCode=LEARNBYDOING)
* [Skillshare](https://skl.sh/3UAFaXW)

If you want more information on the course or just want to check the course material this [link](https://github.com/AntoBrandi/Robotics-and-ROS-Learn-by-Doing-Manipulators) is for you


### Built With
This robot is powered by:
* PC with Ubuntu 20.04 with ROS Noetic
* Arduino UNO
* SG90 Servo Motors (x4)

And is controlled by:
* ROS Noetic

[![Product Name Screen Shot Real][product-screenshot-real]](https://example.com)


<!-- GETTING STARTED -->
## Getting Started

Once it's printed and assembled according and connected, there are few configuration to be made both in Ubuntu and in the Arduino UNO.

### Prerequisites

Make sure you install correctly the following required tools before continuing
* Install Ubuntu 20.04 on PC or in Virtual Machine
Download the ISO [Ubuntu 20.04](https://ubuntu.com/download/alternative-downloads) for your PC
* Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) on your Ubuntu 20.04
* Install ROS missing libraries. Some libraries that are used in this project are not in the standard ROS package. Install them with:
```sh
sudo apt-get update && sudo apt-get install -y \
     ros-noetic-rosserial \
     ros-noetic-gazebo-ros-control \
     ros-noetic-joint-state-publisher-gui \
     ros-noetic-rosserial-arduino \
     ros-noetic-moveit \
     ros-noetic-actionlib-tools
```
* Install VS Code and Arduino IDE on your PC in order to build and load the Arduino code on the device


### Installation

1. Clone the repo
```sh
git clone https://github.com/AntoBrandi/arduinobot.git
```
2. Build the ROS workspace
```sh
cd ~/Arduino-Bot/arduinobot_ws
```
```sh
catkin_make
```
3. Source the ROS Workspace
```sh
source devel/setup.bash
```
4. Connect the Arduino UNO to your PC and open the Arduino IDE. Open the [folder](https://github.com/AntoBrandi/Arduino-Bot/tree/noetic/arduinobot_ws/src/arduinobot_controller/arduino)
containing the code for the Arduino controller.



<!-- USAGE EXAMPLES -->
## Usage

To launch the ROS simulated robot
```sh
roslaunch arduinobot_bringup sim_complete.launch
```

To launch the real robot, connect the Arduino to the PC and upload the code in the [folder](https://github.com/AntoBrandi/Arduino-Bot/tree/noetic/arduinobot_ws/src/arduinobot_controller/arduino/ros_robot_control) on the Arduino controller.
Then launch the real robot
```sh
roslaunch arduinobot_bringup complete.launch
```

To launch the interface with Alexa download [ngrok](https://ngrok.com/download) and create an [account](https://dashboard.ngrok.com/signup) then setup ngrok with your key
```sh
./ngrok authtoken <YOUR-KEY>
```
Then start the ngrok web server with
```sh
./ngrok http 5000
```
Copy the link that provides ngrok and paste it in the section Endpoint of your Alexa Developer account




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
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/antonio-brandi-512166bb/
[udemy-shield]: https://img.shields.io/badge/-Udemy-black.svg?style=flat-square&logo=udemy&colorB=555
[udemy-url]: https://www.udemy.com/user/antonio-brandi/
[skillshare-shield]: https://img.shields.io/badge/-Skillshare-black.svg?style=flat-square&logo=skillshare&colorB=555
[skillshare-url]: https://www.skillshare.com/en/profile/Antonio-Brandi/471799472
[product-screenshot]: images/BB3A0020.jpg
[product-screenshot-2]: images/BB3A0026.jpg
[product-screenshot-real]: images/screen_video.png
