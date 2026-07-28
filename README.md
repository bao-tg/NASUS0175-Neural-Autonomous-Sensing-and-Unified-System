# Project Description

NASUS0175 is the Capstone project as the fullfilment of the Bachelor of Electrical Engineering and Bachelor of Computer Science at VinUniversity. 

The creators included:

+ Chau Hoang Phuc
+ Dinh Bao Dan
+ Nguyen Huu Chi
+ Truong Gia Bao

There are three main features in our project:
+ Voice_command feature
+ Vision tracking feature
+ Control via controller 

[The video demonstration of our project can be found here](https://drive.google.com/file/d/196Zn1yN19xxayYDoAQtbUYIs4UL_btg8/view?usp=sharing)

# Getting Started

## Hardware Setup

### Mechanical Setup

### Electrical Setup

## Bluetooth Setup

## Software Setup

First, you need to build the Docker image via

```bash
sudo docker build .
```

Run the Docker container

```bash
bash run_docker.sh
```

Inside the Docker container, you need to firstly export your OpenAI API's key

```bash
export OPENAI_API_KEY="YOUR_KEY"
```

Run our master node

```bash
roslaunch dingo dingo_main.launch
```

> You need to wait around 10-20s for the system to start.

# Project Structure
```bash
├── assets                                    Images used in the readme file
├── dingo_nano                                Code for the Arduino Nano V3 to read sensor data and send it to the Raspberry Pi
└── dingo_ws                                  ROS workspace containing all required packages
   └── src
     ├── dingo                                Package containing node and launch files for running the robot
     ├── dingo_AI                             Package containing all the implementations for vision tracking feature
     ├── dingo_control                        Package containing all files related to control, including kinematics and default trot controller
     ├── dingo_description                    Package containing simulation files (URDF file and meshes)
     ├── dingo_gazebo                         Package containing gazebo files
     ├── dingo_hardware_interfacing
     |  ├── dingo_input_interfacing           Package containing files for receiving and interpreting commands (From a joystick or keyboard)
     |  ├── dingo_peripheral_interfacing      Package containing files for interfacing with the Arduino Nano, LCD screen and IMU
     |  └── dingo_servo_interfacing           Package containing the hardware interface for sending joint angles to the servo motors
     ├── dingo_utilities                      Package containing useful utilities
     └── dingo_voice                          Package containing the implementations for using voice command feature
```
# Acknowledgement

We would like to express sincere attitude to the StanfordQuadruped for creating such an incredible project DingoQuadruped, which we develop on top.

