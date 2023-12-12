# RadRoverRescue
RadioactiveRoverRescue - A Robot to pick up radioactive waste

## ENPM 808X Final Project
![CICD Workflow status](https://github.com/akasparm/RadRoverRescue/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)
[![codecov](https://codecov.io/gh/akasparm/RadRoverRescue/branch/development/graph/badge.svg)](https://codecov.io/gh/akasparm/RadRoverRescue)
[![License](https://img.shields.io/badge/license-Apache_2.0-blue.svg)](LICENSE)

## Student Details

|Name|Akash Parmar|Sai Teja Gilukara|
|---|:---:|:---:|
|UID|118737430|119369623|
|Directory ID|akasparm|saitejag|
|Email ID|akasparm@umd.edu|saitejag@umd.edu|

## Introduction

The project aims to innovate in the field of radioactive waste management using advanced robotic systems. This initiative is particularly crucial for handling waste from nuclear power plants, medical facilities, and various industries, where traditional methods pose significant risks due to radiation exposure. The robot is designed to operate safely in hazardous environments, perform precise tasks, and significantly reduce the risk to human workers. Key functionalities include navigating and locating radioactive waste sites, detecting these sites using computer vision algorithms, safely collecting and containing the waste, and transporting it to designated disposal areas.

## Dependencies used

|Module|Version|
|---|:---:|
|C++|14|
|ros2|Humble|
|Gazebo|Default|
|OpenCV|Default|

## Instructions to run the code

#### Navigate to the repository: 

```  
git clone https://github.com/akasparm/RadRoverRescue.git
```
#### Configure the project and generate a native build system:
  #### Must re-run this command whenever any CMakeLists.txt file has been changed.
  ```
  cd ~/ros2_ws
  ```
  #### Check dependencies
  ```
  rosdep install -i --from-path src --rosdistro humble -y
  ```
  #### and build the package
  ```
  source /opt/ros/humble/setup.bash
  ```
  ```
  colcon build --packages-select RadRoverRescue
  ```

## Testing the package
  ```
  cd ~/ros2_ws
  source install/setup.bash
  colcon test --event-handlers console_direct+ --packages-select cpp_pubsub
  ```

## Links

|Item|Link|
|---|:---:|
|Project Proposal Document|[here](/Proposal/Project_Proposal.pdf)|
|Quad Chart|[here](/Proposal/quad%20chart.pdf)|
|UML Diagram|[here](/UML/revised/Revised%20UML_diagram.pdf)|
|Flow Chart|[here](/Proposal/Flowchart.pdf)|
|AIP Sheet|[here](https://docs.google.com/spreadsheets/u/0/d/1XzHJKPvlp2NBZCuSRj7cfIvq3t03doPlDkwKnDLFQbU/edit?pli=1#gid=1967651968)|
|Week-1 Sprint|[here](https://docs.google.com/document/d/1_zHlnaphnwkMOBOH4WXWq80tkCj9v3kwDQVp3Jc9LKU/edit)|
|Videos|[here](https://drive.google.com/drive/folders/1_GK_JMNFahxSQeobAj8jg4wJFANAf9b8?usp=sharing)|
