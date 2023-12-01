# The IceCube Haptic Gripper System

The Haptic Gripper System Framework aims to integrate both a way to receive control signal, steering a robot and receive data which is used as a haptic feedback for the operator, into one frame which is easily modified and expanded. By using ROS2 as a dds communication operating system, a modular system is created where the different hardware interfaces of the systems are kept separate as well as the calculations sections. Additionally, the modularity means sections of the system can be used separately for bug fixes or for single stage use.

The first version of the system framework consist of a machined learned camera tracing using the built-in webcam of the host computer to determinate if the operator's hand is open or closed, a qbrobotics Soft Hand Research as the mechanical robotic griper device, and the IceThimble as the haptic feedback device. All these three devices are installed as separate packages and should therefor be easy to change for different devices if needed. 

## Operational Instruction

After assembling and connecting all the hardware for the system, the ROS2 environment and packages should be installed. ROS2 are assumed pre-installed, if not this guide could be followed for a ROS2 humble installation. [Instalation ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

After installing ROS2 a workspace with all the necessary packages should be created. With the commands shown below, a new workspace can be created, and the necessary packages cloned from the GitHub repository. 

```text
mkdir haptic_ws
cd haptic_ws
mkdir src
cd src

git clone https://github.com/Microttus/qbshr_ctr.git
git clone https://github.com/Microttus/hand_gesture_publisher.git
git clone https://github.com/Microttus/haptic_gripper_system.git
```


There are in total three necessary repositories to clone. 
The code is divided as the packages could be used separate for different purpose. 
The packages are linked when building the library with the command shown below. 
The packages should compile, 
but due to the flaws in the qbrobotics library the _qbshr_ctr_ package may break. 
If so, follow the guide provided by qbrobotics for building the library with cmake in a folder in the library folder. 
This will manually build the missing library file, which should have been build while compiling the package. 

You might have to install the required pyton packages before executing. Use:

```text
pip install tensorflow
pip install mediapipe
pip install cvzone
```

Add the USER to the dialout group to enable use of USB. 
NOTE: remember to restart for this to take effect. 

```text
sudo gpasswd --add ${USER} dialout
```

Build the workspace.

```text
cd ../..
source /opt/ros/humble/setup.bash
colcon build
```


If the workspace builds successfully, the system is ready to be used. It is assumed at this stage that the IceThimble is set up as described in the IceThimble project report or in the README [here](https://github.com/Microttus/IceThimble). After the agent is launched, the IceThimble is powered on and the launch sequence for the servos are executed, the topics for applying force should be available. The next step is to set up the hand connection, which is executed by sourcing the local build and the _hand_gipper_system_ ran as shown in code below. Note that this must be in a different terminal window than the agent, as this must be running at all times. Additionally, it is good practice to launch it in a different window than the _colcon build_ was executed. 

```text
. install/setup
ros2 run haptic_gripper_system qbhand_control
```

If successfully connected to a controllable device, several lines conferring the serial port, the number of available devices and setup success should be seen in the terminal. If the \textit{Segmentation fault} is accusing. End the session by pressing ```Ctrl+c```, and try again with the same ros2 run line. If the hand is connected successfully, a new terminal window can be opened. In this window source, and run the executable for the hand gesture tracking, see code below. A window showing the image captured, with hand placement marked, should be visible. The topic _/hand_gesture_ can be inspected for which gesture is identified. 

```text
. install/setup
ros2 run hand_gesture_pub hand_gest_pub
```


The system should now be ready to use. If a hand is identified in the camera, the pose should be replicated with the robotic hand. If the robotic hand is exposed to any kind of force or object resisting the movement, the estimated force should be applied to the operator's fingers with the IceThimble. 
